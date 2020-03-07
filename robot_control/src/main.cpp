#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include <stack>  

using namespace std;

class Controller{
   
  public:
    stack < pair<float,float> > path; // store path top: current node, bottom: goal node

    // Controller variables
    pair<int, int> state; // Current State
    pair<int, int> prev_state; // Previous State

    float kp;
    float kd;
    float ki;


    Controller(ros::NodeHandle &nh);  // constructor
    void RobotStateCb(nav_msgs::Odometry::ConstPtr msg); // Callback for odom
    void PIDController();
};

Controller::Controller(ros::NodeHandle &nh){ //constructor

  ROS_INFO("Controller module initialized ...");
  
  // Get control parameters
  if(nh.hasParam("control_param/kp")){
    nh.getParam("control_param/kp", kp);
    nh.getParam("control_param/kd", kd);
    nh.getParam("control_param/ki", ki);
  }  
  else
    ROS_ERROR("Did not find control parameters");
}

// void Sensing::getPath(){

//   sNode *nodeStart = &nodes[10][10];
//   sNode *nodeEnd = &nodes[15][15];
//   sNode *p = nodeEnd;
  
//   while (p->parent != nullptr)
//   {
//     path.push(make_pair(p->x,p->y)); 
//     // Set next node to this node's parent
//     p = p->parent;
//   }

// }

void Sensing::RobotStateCbk(nav_msgs::Odometry::ConstPtr &msg){
  
  state.first  = msg->pose.position.x;
  state.second = msg->pose.position.y;

}

void Sensing::PIDController(){


}

int main(int argc, char **argv){

  ros::init(argc, argv, "robot_control");
  ros::NodeHandle n;
  
  Controller robot_control(n);

  ros::Subscriber sub_odom = n.subscribe("/odom", 1, &Controller::RobotStateCb, &robot_control);
  
  ros::spin();
  
  return 0;
}