#!/bin/bash
echo "#########################################################"
echo "INSTALLING MTU 9000 Patch"
echo "#########################################################"
sudo apt update -y

if grep -q "done" mtu.tmp
then echo "Already installed."
else
sudo apt install -y linux-raspi-headers-5.15.0-1040 linux-modules-5.15.0-1040-raspi
sudo dpkg -i linux-headers-5.15.0-1040-raspi_5.15.0-1040.43_arm64.deb
sudo dpkg -i linux-image-5.15.0-1040-raspi_5.15.0-1040.43_arm64.deb
sudo dpkg -i linux-modules-5.15.0-1040-raspi_5.15.0-1040.43_arm64.deb
sudo dpkg -i linux-modules-extra-5.15.0-1040-raspi_5.15.0-1040.43_arm64.deb
sudo dpkg -i linux-raspi-headers-5.15.0-1040_5.15.0-1040.43_arm64.deb
echo done > mtu.tmp
sudo reboot
fi

# installing ros2
echo "#########################################################"
echo "INSTALLING ROS2 HUMBLE"
echo "#########################################################"
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update -y
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update -y
sudo apt install ros-humble-ros-base -y
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep2 -y
rosdep update -y

if grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc
then echo ""
else echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo "#########################################################"
echo "INSTALLING OUSTER DRIVERS"
echo "#########################################################"
# installing ouster deps
sudo apt install -y             \
    ros-humble-pcl-ros     \
    ros-humble-tf2-eigen   \
    ros-humble-rviz2
sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake

# installing ouster driver
source /opt/ros/humble/setup.bash
mkdir -p ouster_ws/src && cd ouster_ws/src
git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ..

echo "#########################################################"
echo "INSTALLING FLIR DRIVERS"
echo "#########################################################"
source /opt/ros/humble/setup.bash
mkdir -p flir_ws/src && cd flir_ws/src
git clone --branch humble-devel https://github.com/ros-drivers/flir_camera_driver
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cd ..
mkdir -p ~/.ros/camera_info/
cp flir_calib_data/ost.yaml ~/.ros/camera_info/flir_camera.yaml
cp flir_node.launch.py flir_ws/install/spinnaker_camera_driver/share/spinnaker_camera_driver/launch/driver_node.launch.py

echo "#########################################################"
echo "SETTING UP NETWORK"
echo "#########################################################"
sudo cp network_plan.yaml /etc/netplan/99_config.yaml
sudo netplan apply
sudo reboot
