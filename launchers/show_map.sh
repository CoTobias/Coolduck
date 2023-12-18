#!/bin/bash

source /environment.sh

# launch parameter server
# rosparam load $(rospack find my_package)/config/rviz_parameters.yaml

# launch RViz
# shellcheck disable=SC2046
rosrun rviz rviz -d $(rospack find my_package)/config/rviz/map_view.rviz &

# launch subscriber
rosrun my_package show_map.py

# wait for app to end
dt-launchfile-join

