#!/usr/bin/env bash
set -e

cd /home/mario-cirtesu/blueboat_ws

source /opt/ros/humble/setup.bash

git submodule update --init --recursive || true
rosdep update --rosdistro humble || true
rosdep install --from-paths src --ignore-src -r -y || true

grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || \
  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

grep -qxF 'if [ -f /home/mario-cirtesu/blueboat_ws/install/setup.bash ]; then source /home/mario-cirtesu/blueboat_ws/install/setup.bash; fi' ~/.bashrc || \
  echo 'if [ -f /home/mario-cirtesu/blueboat_ws/install/setup.bash ]; then source /home/mario-cirtesu/blueboat_ws/install/setup.bash; fi' >> ~/.bashrc