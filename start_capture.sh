#!/bin/bash
sudo ip link set dev eth0 mtu 9000
source flir_ws/install/local_setup.bash && ros2 launch spinnaker_camera_driver driver_node.launch.py camera_type:=blackfly_s serial:="'23422874'" &
sleep 10
source ouster_ws/install/local_setup.bash && ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=192.168.1.12 viz:=false proc_mask:="IMG|PCL|IMU" timestamp_mode:=TIME_FROM_ROS_TIME use_system_default_qos:=true ouster_ns:=lidar &
sleep 10
ros2 bag record -o /mnt/output /flir_camera/camera_info /flir_camera/image_raw /lidar/points
