# TurtleNet

TurtleNet is a project out of the Autonomous Networks Research Group at the University of Southern California investigating multi-robot exploration and mapping via ultrawide-band trilateration.

Our testbed is comprised of several Turtlebot3 Burgers integrated with Pozyx Anchors.

Turtlebot3 Burgers  
specs: http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/#specifications  
source code: https://github.com/ROBOTIS-GIT/turtlebot3  

Simulating in Gazebo  
tutorial: http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-gazebo  

Pozyx Anchor  
specs: https://www.pozyx.io/shop/product/creator-anchor-69  
python library: https://pypozyx.readthedocs.io/en/develop/  

### This repository

This repository contains code to run on your remote PC and launch TurtleNet simulations in Gazebo.

To use this repository, review the turtlebot3_simulations installation instructions here: http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#ros-1-simulation

In your ~/catkin_ws/src directory, where you would have cloned the turtlebot3_simulations package, clone this repository and re-build your workspace.

Original source code:  
http://wiki.ros.org/turtlebot3_simulations  

#### turtlebot3_gazebo/launch
`multi_turltebot3_world.launch` launches TurtleNet in Gazebo and specifies the `.world` file to use.
`turtlebot3_simulation.launch` launches the `turtlebot3_drive` ROS node
`multi_turtlebot3_simulation.launch` launches a drive ROS node on each of the 4 TurtleNet robots and can be used to control the robots in reality as well, with `sim = False; real = True`
`multi_map_merge.launch` launches a `map_merge` ROS node from the `multirobot_map_merge` package (http://wiki.ros.org/multirobot_map_merge)

#### turtlebot3_gazebo/src
`turtlebot3_drive.cpp` contains minor changes to the drive ROS node from the source code
`new_drive.cpp` contains code for driving TurtleNet in reality, in which case each robot has a 4 second drive turn (no two robots drive at once)
