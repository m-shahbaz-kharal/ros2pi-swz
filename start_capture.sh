#!/bin/bash
echo "#########################################################"
echo "SETTING ETHERNET MTU 9000"
echo "#########################################################"
sudo ip link set dev eth0 mtu 9000

echo "#########################################################"
echo "Starting CAMERA DRIVER"
echo "#########################################################"
source flir_ws/install/local_setup.bash && ros2 launch spinnaker_camera_driver driver_node.launch.py camera_type:=blackfly_s serial:="'23422874'" &

sleep 10
echo "#########################################################"
echo "STARTING LIDAR DRIVER"
echo "#########################################################"
source ouster_ws/install/local_setup.bash && ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=192.168.1.12 viz:=false proc_mask:="IMG|PCL|IMU" timestamp_mode:=TIME_FROM_ROS_TIME use_system_default_qos:=true ouster_ns:=lidar &

# or
# source hesai_ws/install/local_setup.bash && ros2 launch hesai_ros_driver start.py &

sleep 10
echo "#########################################################"
echo "STARTING DATA CAPTURE"
echo "#########################################################"
ros2 bag record -o /mnt/output /flir_camera/camera_info /flir_camera/image_raw /lidar/points
