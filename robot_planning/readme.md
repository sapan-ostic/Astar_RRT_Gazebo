# Turtlebot Random walker with obstacle avoidance

![GitHub Logo](/results/turtlebot_random_walker.gif)

The Turtlebot moves forward until an obstacle is detected in its vicinity. When the obstacle is detected, the robot rotates until it finds a free path and once found, it moves forward again.

## Prerequisites
The following software packages need to be installed before preceeding with this demo. The setup is installed on Ubuntu 16.04. 
1. Ros Kinetic: http://wiki.ros.org/kinetic/Installation
2. Turtlebot simulation packages:

```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
3. Gazebo: http://gazebosim.org/tutorials?tut=install_ubuntu
4. Catkin: http://wiki.ros.org/catkin
5. Hokuyu Lidar Sensor: http://wiki.ros.org/turtlebot/Tutorials/indigo/Adding%20a%20lidar%20to%20the%20turtlebot%20using%20hector_models%20%28Hokuyo%20UTM-30LX%29
```
sudo apt-get install ros-kinetic-urg-node
```
6. Follow the ROS tutorials if you are new to ROS: http://wiki.ros.org/ROS/Tutorials

## Running the code
Download the repository and build it.
'''
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cd src/
git clone 
cd ..
catkin_make
'''

Tip: Include source ~/catkin_ws/devel/setup.bash in the ~/.bashrc file to run it everytime a new terminal is created.

## Running the simulation
To simulate the turtlebot in Gazebo:

```
cd ~/catkin_ws
roslaunch turtlebot_random_walker walker.launch
```
Press ctrl+c to quit the simulation.

## To record your own rosbag
To log data on topics, launch the code with `runRosbag:=true`
```
roslaunch turtlebot_random_walker walkler.launch runRosbag:=true
```
Check the `bags` folder for a ROSbag file called walker.bag. This file has a record of all topics from the simulated turtlebot.

## Observing topics in the ROS bag
To run the rosbag file:
```
cd ~/catkin_ws/src/turtlebot_random_walker/bags
rosbag play walker.bag
```

To view the topics that have been recorded, run the rosbag first:
```
rostopic list
```

or
```
cd ~/catkin_ws/src/turtlebot_random_walker/results
rosbag info walker.bag
```
