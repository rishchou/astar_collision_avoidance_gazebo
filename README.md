# astar_collision_avoidance_gazebo
Implementation of A* algorithm on a differential drive (non-holonomic) robot using ROS/GAZEBO

## Directory structure:
	-> catkin package (project-3)
	-> Submission video
	-> README

The python script is under project-3/scripts/turtlebot.py
The clearance of robot is taken as bot radius + 15 cm
# Build instructions:
1. Download and extract the catkin package (project-3) submitted as part of this folder and build a catkin_ws using the following commands:

-> mkdir -p catkin_ws/src
-> cd catkin_ws
-> catkin_make
-> cp project-3 catkin_ws/src
-> catkin_make

This will build the package inside catkin_ws

Now source the setup.bash from devel folder

-> cd catkin_ws
-> source devel/setup.bash

Make the turtlebot.py script executable
-> chmod u+x catkin_ws/src/project-3/scripts/turtlebot.py 

## Run instructions:
To run the code, use roslaunch as given below
(source setup.bash prior to this)

Add env variable TURTLEBOT3_MODEL=waffle in the roslaunch terminal 

Then run:
-> roslaunch project-3 turtlebot3.launch 

This will initialize the bot with (100 100)(cms) start position in gazebo.

If you wish to change the initial gazebo start position of bot, use

-> roslaunch project-3 turtlebot3.launch x_pos:=<x_value> y_pos:=<y_value>

This will launch both gazebo and python script, which will ask for user inputs of start point(in cms), goal point (in cms) and rpm values. The code has been tuned for rpm values (40,60) perfectly. Similar range values can be tried. 

NOTE: The origin of half planes is chosen as bottom left corner. The python script will take values in accordance with the bottom left corner origin. 

The gazebo origin is at center by default. To change the initial position of the turtlebot in gazebo pass the arguments in roslaunch with respect to the center origin as shown below:

-> roslaunch project-3 turtlebot3.launch x_pos:=-3.55 y_pos:=-3.05 (This will initialize the robot pose as (200, 200)(cms) (relative to center)

 THe video submission shows output for start point as (100 100) and goal point as (555 950)