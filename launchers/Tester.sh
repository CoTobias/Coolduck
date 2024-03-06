#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun my_package Tester.py

# wait for app to end
dt-launchfile-join