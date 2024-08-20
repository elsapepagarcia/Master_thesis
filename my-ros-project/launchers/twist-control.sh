#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun my_package twist_control_node.py
#rosrun my_package twist_control_and_subscriber.launch

# wait for app to end
dt-launchfile-join
