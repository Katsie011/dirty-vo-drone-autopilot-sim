xhost +

podman run -it --rm \
  -p 5760:5760 \
  -p 14550:14550 \
  --platform linux/amd64 \
  --env VEHICLE=ArduCopter \
  orthuk/ardupilot-sitl \
  ./Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild \
  --out udp:host.docker.internal:14550



# podman run -it orthuk/ardupilot-sitl \
# -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
# -e DISPLAY=host.docker.internal:0 \
# ./Tools/autotest/sim_vehicle.py -v ArduPlane \
#  --frame quadplane --map --console