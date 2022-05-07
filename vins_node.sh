#!/usr/bin/env bash
cd /home/wws/catkin
source /home/wws/catkin/devel/setup.bash
# rosrun --prefix 'gdb -ex run --args' vins vins_node ~/remoteSensing2020/src/VINS-Fusion/config/vi_car/vi_car.yaml
rosrun vins vins_node ~/catkin/src/VINS-Fusion/config/vi_car/vi_car.yaml