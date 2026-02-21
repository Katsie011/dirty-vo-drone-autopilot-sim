# α-β Physics Filter Integration

## What Changed

**v0.0.2 → v0.0.03:** Added physics-based state estimation with α-β filtering.

### Before (v3 — Pure VO)
```
Frame → ORB → Match → Essential Matrix → R, t
                                          ↓
                                    EMA smoothing
                                          ↓
                                   Display pose
```

### After (v0.0.3 — VO + Physics)
```
Frame → ORB → Match → Essential Matrix → R, t (raw measurement)
                                          ↓
                            ┌─────────────┴──────────────┐
                            │  α-β Physics Filter        │
                            │  PREDICT: p + v*dt         │
                            │  UPDATE:  α-β fusion       │
                            └─────────────┬──────────────┘
                                          ↓
                            Smoothed (position, velocity)
```

## Why Physics?

1. **Velocity estimates** — previously we had position only. Now we have `vx, vy, vz` which can:
   - Feed into a planner (know if we're moving or still)
   - Detect anomalies (unexpected accelerations)
   - Predict where we'll be in the next frame (better RANSAC seed)

2. **Temporal consistency** — VO measurements jitter frame-to-frame. Physics smooths this by assuming constant velocity between measurements.

3. **Measurement rejection** — if VO suddenly jumps 3 metres in one frame (outlier), the physics gate rejects it and holds the prediction instead.

4. **Upgrade path to sensor fusion** — when we add an IMU, the α-β filter structure extends cleanly to a full Extended Kalman Filter (EKF).

## The α-β Filter Explained

Two steps per frame:

### PREDICT (physics propagation)
```python
p_predicted = p + v * dt
q_predicted = q  # (no gyro yet — hold orientation)
```

### UPDATE (measurement fusion)
```python
residual = measured_p - p_predicted  # Innovation (error)

# Sanity gates
if |residual| > 2.0 m:  REJECT measurement
if |residual/dt| > 5 m/s:  REJECT (induced velocity too high)

# α-β update
p_new = p_predicted + α * residual          # Blend position
v_new = v + (β/dt) * residual               # Adjust velocity
q_new = SLERP(q_predicted, q_measured, γ)  # Blend orientation
```

## Tuning Parameters

In `utils/physics_state.py` → `FilterGains`:

| Parameter | Default | What it does |
|-----------|---------|-------------|
| `alpha` | 0.6 | Position trust (0=ignore measurements, 1=full trust) |
| `beta` | 0.4 | Velocity adjustment gain |
| `gamma` | 0.7 | Orientation blend weight |
| `max_position_jump_m` | 2.0 | Reject if measurement > 2m from prediction |
| `max_velocity_m_s` | 5.0 | Reject if induced velocity > 5 m/s |

### How to tune

- **If trajectory is too jittery:** increase `alpha` (trust measurements more)
- **If trajectory lags behind real motion:** decrease `alpha`, increase `beta`
- **If measurements are being rejected too often:** increase `max_position_jump_m`
- **If bad measurements are leaking through:** decrease gate thresholds

## New State Variables

The VO now publishes:

```python
pose = vo.update(frame)

pose["x"]         # metres (ENU frame)
pose["y"]
pose["z"]
pose["yaw_deg"]   # degrees

# NEW in v0.0.3:
pose["vx"]        # m/s (velocity in x)
pose["vy"]        # m/s
pose["vz"]        # m/s (always ~0 for now — no altitude change)
pose["measurement_rejected"]  # bool — True if physics gate rejected VO this frame
```

## Diagnostics

The physics filter tracks:
- `prediction_error` — |measured - predicted| in metres
- `rejected_count` — total measurements rejected by sanity gates
- `dt` — actual time delta per frame (should be ~0.033s at 30fps)

Access via:
```python
state = vo._physics.get_state()
print(f"Prediction error: {state['pred_err']:.3f} m")
print(f"dt: {state['dt']:.3f} s")
```

## Disabling Physics (for comparison)

To run without the physics filter (pure VO like v3):

```python
vo = VisualOdometry(K, use_physics=False)
```

Or add a CLI flag in `main.py`:
```python
ap.add_argument("--no-physics", action="store_true")
...
vo = VisualOdometry(K, use_physics=not args.no_physics)
```

## Next Steps

1. **IMU integration** — add gyro/accel measurements to the filter
   - `predict()` uses gyro for orientation propagation
   - `update()` fuses accel with VO velocity
   
2. **Extended Kalman Filter (EKF)** — upgrade from α-β to full covariance tracking
   - Uncertainty estimates on position/velocity
   - Optimal sensor fusion (Kalman gain computed from covariances)

3. **Barometer** — replace `ASSUMED_HEIGHT_M` with real altitude measurements
   - Enables metric scale recovery (no more hardcoded 0.08 scale factor)

## File Structure

```
utils/
├── __init__.py
├── config.py             # All tunable parameters
├── orb_vo.py            # VO with physics integration
├── physics_state.py     # α-β filter implementation
├── obj_detector.py      # YOLOv8 background thread
└── visualizer.py        # (not yet migrated — still in main.py)
```

## Performance Impact

**CPU overhead:** ~2% (physics prediction + update is cheap)
**Memory:** +200 bytes per frame (state vector)
**Latency:** None (filter runs synchronously in the same thread)

The α-β filter is specifically chosen for real-time performance — no matrix inversions, no iterative solvers, just arithmetic.
