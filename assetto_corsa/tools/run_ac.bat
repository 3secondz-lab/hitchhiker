set ROS_CATKIN_WS=~/ros_catkin_ws
set ROS_WS=~/catkin_ws
set USB_IP=/mnt/c/usbip

set RUN_CMD="source %ROS_CATKIN_WS%/install/setup.bash && source %ROS_WS%/devel/setup.bash && roslaunch assetto_corsa run_ac.launch & /mnt/c/usbip/usbip.exe -a localhost "1-1" 1>nul "

echo "Run ROS packages..."
bash -c %RUN_CMD%