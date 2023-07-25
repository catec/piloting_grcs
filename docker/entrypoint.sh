#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

echo ""
echo "---------------------------------------------------------"
echo "-----   PILOTING General Robot Control Station   --------"
echo "-----             Version: $GRCS_VERSION                 --------"
echo "---------------------------------------------------------"
echo ""

roscore > /dev/null 2>&1 &
echo "Initialization may takes 10 seconds. Please, wait."
sleep 8
piloting_grcs_node

# Useful to run any command when container starts
# exec "$@"
