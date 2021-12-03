export ROS_MASTER_URI=http://10.25.111.56:11311
export ROS_IP=10.25.111.56
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/video0
roslaunch agrobot agro_sensors.launch
