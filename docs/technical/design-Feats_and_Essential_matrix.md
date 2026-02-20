# Design Decision — Why Feature-Based & Essential Matrix?

### **Decision: Moving away from LK Flow**

While Lucas-Kanade (LK) flow is fast, it’s "template-based." It struggles with large rotations and loses track if the drone moves too quickly (motion blur). For a commercial-grade drone, we need **Feature-Based Odometry.**

### **1. Why Feature-Based (ORB/FAST)?**

Unlike raw pixel tracking, feature-based methods extract "descriptors" (unique digital fingerprints) for each point.

- **Rotation Invariance:** If the drone spins, the descriptor still looks the same.
- **Re-localization:** If we lose tracking for a split second, we can "find" the same points again in the next frame by matching their fingerprints.
- **Efficiency:** We only process 500-1000 high-quality points instead of the whole image, making it perfect for our MacBook Air and future ARM-based flight controllers.

### **2. Why the Essential Matrix () Solver?**

The Essential Matrix is the geometric "bridge" between two camera views. It relates a point in Frame A to its position in Frame B using only the camera's internal calibration and its relative motion.

**The Geometry:**

- **Degrees of Freedom:** has 5 degrees of freedom (3 for rotation, 2 for translation direction).
- **Outcome:** By solving for , we can decompose it (via SVD) into a rotation matrix () and a translation vector (). This gives us the drone's actual path through 3D space.

### **Final Verdict**

This architecture is chosen because it’s hopefully the **Goldilocks Zone** for drone dev: robust enough to handle jerky flight, but light enough to run at 30+ FPS without needing a dedicated GPU.
