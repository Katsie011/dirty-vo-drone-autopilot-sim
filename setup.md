create a conda env
conda create -n aeroperception PYTHON=3.13

Install the px4 dev tools

```
brew tap PX4/px4
brew install px4-dev
```

Install the required python packages

```
# install required packages using pip3
python3 -m pip install --user pyserial empty toml numpy pandas jinja2 pyyaml pyros-genmsg packaging kconfiglib future jsonschema
# if this fails with a permissions error, your Python install is in a system path - use this command instead:
sudo -H python3 -m pip install --user pyserial empty toml numpy pandas jinja2 pyyaml pyros-genmsg packaging kconfiglib future jsonschema
```

GOT HERE

Or using conda:

```
conda install pyserial  toml numpy pandas jinja2 pyyaml  packaging  future jsonschema
pip install empty pyros-genmsg kconfiglib
```

Need to set up gazebo:
https://docs.px4.io/main/en/dev_setup/dev_env_mac#gazebo-classic-simulation

brew install tbb

brew install --cask temurin
brew install --cask xquartz
brew install px4-sim-gazebo

## Boost install error:

cd External/PX4-Autopilot/Tools/setup
sh macos.sh

was failing on this error:

```log
Can't find boost headers. Please check the location of the boost
distribution and rerun configure using the --with-boost=DIR option.
...
...
These open issues may also help:
px4-dev fails to build on M1 (Rosetta) macOS: "Can't find boost headers" during asio@1.10.8 configure step https://github.com/PX4/homebrew-px4/issues/98
Error: No such keg: /opt/homebrew/Cellar/arm-gcc-bin@13
```

Tried reinstalling boost:
brew install boost

```
    --> Warning: boost 1.90.0_1 is already installed and up-to-date.
```

Found the files in:

```
/opt/homebrew/Cellar/boost
```

Added these to my ~/.zshrc

```
export CPPFLAGS="-I/opt/homebrew/include"
export LDFLAGS="-L/opt/homebrew/lib"
```

Still did not work so ran the install command with:

```
sh macos.sh --with-boost=/opt/homebrew/Cellar/boost
```

Same error now with `Error: No such keg: /opt/homebrew/Cellar/arm-gcc-bin@13`

Running commands manually, when building px4-sim:

```log
Error: Could not symlink include/QtDeviceDiscoverySupport/6.9.1/QtDeviceDiscoverySupport/private/qdevicediscovery_dummy_p.h
Target /opt/homebrew/include/QtDeviceDiscoverySupport/6.9.1/QtDeviceDiscoverySupport/private/qdevicediscovery_dummy_p.h
is a symlink belonging to qt. You can unlink it:
  brew unlink qt

To force the link and overwrite all conflicting files:
  brew link --overwrite qt

To list all files that would be deleted:
  brew link --overwrite qt --dry-run
```

```
brew link --overwrite qt --dry-run
Warning: Already linked: /opt/homebrew/Cellar/qt/6.9.1
To relink, run:
  brew unlink qt && brew link qt
```

Ran: `brew unlink qt`
then: `brew install p4x-sim`

Seems we have a mismatch in boost versions:

```

```

So:
brew uninstall boost
brew cleanup

CPPFLAGS="-I/opt/homebrew/opt/boost@1.85/include" \
LDFLAGS="-L/opt/homebrew/opt/boost@1.85/lib" \
CMAKE_PREFIX_PATH="/opt/homebrew/opt/boost@1.85" \
brew install px4-sim

Still does not run.
So gave up around here and switched to using the docker container

## Docker setup

./Tools/docker_run.sh 'make px4_sitl_default'

# OpenVIs

```bash
export VERSION=ros2_22_04 # which docker file version you want (ROS1 vs ROS2 and ubuntu version)
docker build -t ov_$VERSION -f Dockerfile_$VERSION .
```

Fails:

```log
 - InvalidBaseImagePlatform: Base image osrf/ros:humble-desktop was pulled with platform "linux/amd64", expected "linux/arm64" for current build (line 1)
Dockerfile_ros2_22_04:16
--------------------
  14 |     # Dependencies we use, catkin tools is very good build system
  15 |     # Also some helper utilities for fast in terminal edits (nano etc)
  16 | >>> RUN apt-get update && apt-get install -y libeigen3-dev nano git
  17 |
  18 |     # Ceres solver install and setup
--------------------
ERROR: failed to solve: failed to compute cache key: chmod /var/lib/desktop-containerd/daemon/io.containerd.snapshotter.v1.overlayfs/snapshots/93/fs/usr/share/libwacom/elan-2a70.tablet: input/output error: unknown
```
