#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <robot_planning/trajData.h>
#include "robot_planning/state.h"
#include <stack>  
#include <cmath>

using namespace std;

class Controller{
   
  public:

    // Controller variables
    robot_planning::trajData planned_path;
    
    robot_planning::state targetState;
    robot_planning::state currentState;
    robot_planning::state prevState;

    float desired_heading;
    float heading;

    float kp_lin = 0.08;
    float kd_lin = 0.1;
    float ki_lin = 0.01;

    float kp_ang = 0.6;
    float kd_ang = 0.1;
    float ki_ang = 0.001;

    float TARGET_EPS = 0.05;
    float MAX_VEL = 0.3;

    float prev_err_linear = 0;
    float d_err_linear = 0;
    float int_err_linear = 0;

    float prev_err_angular = 0;
    float d_err_angular = 0;
    float int_err_angular = 0;

    float linear_vel = 0;
    float angular_vel = 0;

    ros::Publisher pub_vel;

    Controller(ros::NodeHandle &nh);  // constructor
    void RobotStateCbk(nav_msgs::Odometry::ConstPtr msg); // Callback for odom
    void RobotPathCbk(robot_planning::trajData::ConstPtr msg);
    float distance();
    void PIDController();
    void publishVel();
};

Controller::Controller(ros::NodeHandle &nh){ //constructor

  ROS_INFO("Controller module initialized ...");
  prevState.x = 0;
  prevState.x = 0;

  ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

 }

void Controller::RobotStateCbk(nav_msgs::Odometry::ConstPtr msg){
  
  currentState.x  = msg->pose.pose.position.x;
  currentState.y = msg->pose.pose.position.y;

  float qz = msg->pose.pose.orientation.z;
  float qw = msg->pose.pose.orientation.w;

  heading = atan2(2*(qw*qz), (1 - 2*qz*qz));
}

float Controller::distance(){
  float dist;
  dist = sqrt(pow(targetState.x - currentState.x,2) + pow(targetState.y - currentState.y,2));
  
  return dist;  
}

void Controller::RobotPathCbk(robot_planning::trajData::ConstPtr msg){
  
  planned_path.data = msg->data;
  targetState = (planned_path.data)[0];
  
  if(distance() < TARGET_EPS)
    targetState = (planned_path.data)[1];

  cout << targetState.x << endl;
}

void Controller::PIDController(){
  
  desired_heading = atan2(currentState.y - targetState.y, currentState.x - targetState.x); 
  
  float err_linear = distance();
  d_err_linear = err_linear - prev_err_linear;
  int_err_linear = int_err_linear + err_linear;
  
  float err_angular = heading - desired_heading;
  d_err_angular = err_angular - prev_err_angular;
  int_err_angular = int_err_angular + err_angular;
  
  linear_vel = kp_lin * err_linear + kd_lin * d_err_linear + ki_lin*int_err_linear;
  angular_vel = kp_ang * err_angular + kd_ang * d_err_angular + ki_ang*int_err_angular;
  
  linear_vel = max(min(linear_vel,  MAX_VEL) ,-MAX_VEL);
  
  prev_err_linear = err_linear;
  prev_err_angular = err_angular;

}

void::Controller::publishVel(){
  geometry_msgs::Twist cmd;
  cmd.linear.x = linear_vel;
  cmd.angular.z = angular_vel;
  pub_vel.publish(cmd);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "robot_control");
  ros::NodeHandle n;
  
  Controller robot_control(n);

  ros::Subscriber subs_odom = n.subscribe("/odom", 1, &Controller::RobotStateCbk, &robot_control);
  ros::Subscriber subs_path = n.subscribe("/planned_path", 1, &Controller::RobotPathCbk, &robot_control);
  ros::Rate loop_rate(100); // Control Frequency

  while(ros::ok()){
    
    robot_control.publishVel();
    ros::spinOnce();
    
    loop_rate.sleep();
  }
  
  return 0;
}