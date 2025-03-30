#!/bin/bash

# Before running this program, make it executable with the command:
# chmod +x ~/run_flightstack.sh

# Change to the ROS2 workspace directory
# cd ~/ros2_ws

# Uncomment for ROS2 humble
echo "odroid" | sudo -S bash -c "source /opt/ros/galactic/setup.bash && source install/local_setup.bash && chrt -f 99 ros2 run flightstack flightstack"

# Source the ROS2 and workspace setup scripts
# echo "odroid" | sudo -S bash -c "source /opt/ros/galactic/setup.bash && source install/local_setup.bash"

# Run the Flightstack with Scheduling policy: SCHED_FIFO and Scheduling priority: 99
# sudo chrt -f 99 ros2 run flightstack flightstack



