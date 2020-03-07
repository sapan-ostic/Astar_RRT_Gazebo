#include <iostream>
#include "planner.h"

#include "ros/ros.h"
#include "std_msgs/Float32.h"

using namespace std;

int main(int argc, char **argv){

  ros::init(argc, argv, "planner");  
  ros::Rate looprate(100); 

  Planner robot;
  
  while(ros::ok()){
  	int connected;
  	ros::param::get("robot_info/grid_connections", connected);
  	ROS_INFO("%d",connected);
  	break;
	robot.move();
	
	ros::spinOnce();
	looprate.sleep();		
  }

  return 0;
}