This project is a MAVROS based implementation of an autonomous drone runs on ArduPilot firmware with a path in form of a square.

Requirements: ROS(Melodic), Gazebo, MAVROS and ArduPilot 

This document mentions how to run node "Square" step by step.
___________________________________________________________
Ros:  
$ roscore  

Gazebo:  
$ gazebo --verbose worlds/iris_arducopter_runway.world  

Ardupilot  
$ cd ~/ardupilot/ArduCopter  
$ ../Tools/autotest/sim_vehicle.py -f gazebo-iris   

Mavros:  
$ rosrun mavros mavros_node _fcu_url:=tcp://localhost:5762  

Make communicate possible:  
$ rosrun mavros mavsys rate --all 10  

Compile and run the code:  
$ cd ~/tsa_assignment  
$ catkin_make  
$ source ./devel/setup.bash  
$ rosrun tsa_assignment square  

To echo next position published by square node (it will start publishing after take off):  
$ rostopic echo /drone/next_waypoint  
______________________________________________________________
Elif Balcı  
i.elif.balci@gmail.com  
elifbalci.com
