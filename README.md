Agrobot
=======

Package dedicated to run the Agrobot project. A robot that can semi 
navigate in an unknown environment, detecting tomatoes.

Table of content
----------------
- [Installation](#installation)
- [Quickstart](#quickstart)

Installation
------------

1. Install ROS noetic
2. Create your workspace `~/catkin_ws`
3. Get [hector_slam](http://wiki.ros.org/hector_slam)
4. Inside the `src` folder
   1. Clone agrobot package
   ```
    git@github.com:soyantonio/agrobot.git
   ```
   3. Clone nav2d
   ```
    git clone https://github.com/skasperski/navigation_2d.git
   ```
5. In `~/catkin_ws` build the project
   ```
   catkin_make
   ```
6. Copy the file `remote-jackal.sh` to your home
   1. Change the IPs to match your configuration
7. Copy the file `lidar.sh` to the jackal home
   1. Change the IPs to match your configuration


Quickstart
----------

> The password is: **clearpath**

Set up the sensors
1. Open a connection to the jackal `ssh administrator@10.25.111.56`
2. Connect the LIDAR and the USB Camera
3. Configure ROS IPs and start sensors `sh ~/lidar.sh`

Start a program
1. Load remote connection `source ~/remote-jackal.sh`
2. Chose a program
   - Semi Navigation with tomatoes
   ```
   roslaunch agrobot full_app.launch
   ```
   - Follow the closest object
   ```
   roslaunch agrobot agro_follower.launch
   ```
   - Teleoperate mode
   ```
   roslaunch agrobot agro_teleop.launch
   ```
   - Mapping mode
   ```
   roslaunch agrobot agro_mapping.launch
   ```

