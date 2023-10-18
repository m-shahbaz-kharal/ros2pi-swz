#!/bin/bash
sudo ip link set dev eth0 mtu 9000
source flir_ws/install/local_setup.bash && ros2 launch spinnaker_camera_driver driver_node.launch.py &
sleep 10
source ouster_ws/install/local_setup.bash && ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=192.168.1.12 viz:=false proc_mask:="IMG|PCL|IMU" timestamp_mode:=TIME_FROM_ROS_TIME &
sleep 10
