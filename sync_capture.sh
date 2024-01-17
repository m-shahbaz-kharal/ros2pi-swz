#!/bin/bash
source utils_ws/install/local_setup.bash && ros2 run post_process sync_via_approx_time_policy &
sleep 10
ros2 bag play /home/me/out