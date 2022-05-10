#!/usr/bin/env bash 
gnome-terminal -- bash -c "./rviz.sh;exec bash;"
gnome-terminal -- bash -c "./vins_node.sh;exec bash;"
# gnome-terminal -x bash -c "./globalFusion.sh;exec bash;"
gnome-terminal -- bash -c "./openbag.sh;exec bash;"
# gnome-terminal -- bash -c "./map2rviz.sh;exec bash;"