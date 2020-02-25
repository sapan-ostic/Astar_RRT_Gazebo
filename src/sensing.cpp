#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace std;

class Sensing{
   
  public:
    double grid_resolution;
    int grid_size;
    int grid_connections;

    struct sNode{
      bool bObstacle = false;       // Is the node an obstruction?
      bool bVisited = false;        // Have we searched this node before?
      float fGlobalGoal;            // Distance to goal so far
      float fLocalGoal;             // Distance to goal if we took the alternative route
      float x;                        // Nodes position in 2D space
      float y;

      vector<sNode*> vecNeighbours; // Connections to neighbours
      sNode* parent;                // Node connecting to this node that offers shortest parent
    };

    vector<vector<sNode>> nodes; // initializing map to represent all nodes

    Sensing(ros::NodeHandle &nh);  // constructor
    void costmapCb(const nav_msgs::OccupancyGridConstPtr grid); // Callback for costmap


  private:
    void make_connections();      // add neighbors to the nodes 

};

Sensing::Sensing(ros::NodeHandle &nh){ //constructor

  if(nh.hasParam("costmap_node/costmap/width")){
    nh.getParam("costmap_node/costmap/width", grid_size);
    nh.getParam("costmap_node/costmap/resolution", grid_resolution);
    nh.getParam("robot_info/grid_connections", grid_connections);
  }  
  else
    ROS_ERROR("Did not find parameters");

  grid_size /= grid_resolution;

  nodes.resize(grid_size, vector<sNode>(grid_size));   // allocating memory to initialized vector 
  
  // Add neighbors
  make_connections();
}

void Sensing::make_connections(){
  // Add neighbors

  for (int i {0}; i< grid_size; i++){
    for (int j {0}; j< grid_size; j++){
      
      // 4-connected grid
      if (i>0)
        nodes[i][j].vecNeighbours.push_back(&nodes[i-1][j+0]);
      if (i<grid_size)
        nodes[i][j].vecNeighbours.push_back(&nodes[i+1][j+0]);
      if (j>0)
        nodes[i][j].vecNeighbours.push_back(&nodes[i+0][j-1]);
      if (j<grid_size)
        nodes[i][j].vecNeighbours.push_back(&nodes[i+0][j+1]);

      if(grid_connections == 8){
        // 8-connected grid
        if (i>0 && j>0)
          nodes[i][j].vecNeighbours.push_back(&nodes[i-1][j-1]);
        if (i<grid_size && j>0)
          nodes[i][j].vecNeighbours.push_back(&nodes[i+1][j-1]);
        if (i>0 && j<grid_size)
          nodes[i][j].vecNeighbours.push_back(&nodes[i-1][j+1]);
        if (j<grid_size && j<grid_size)
          nodes[i][j].vecNeighbours.push_back(&nodes[i+1][j+1]);
      }
    
    } // j
  } // i
}

void Sensing::costmapCb(const nav_msgs::OccupancyGridConstPtr grid){ 
  
  grid_resolution = grid->info.resolution;   
 
  float map_x = grid->info.origin.position.x;
  float map_y = grid->info.origin.position.y; 

  auto map =  grid->data; 
  
  for (int i {0}; i< grid_size; i++){
    for (int j {0}; j< grid_size; j++){      
    
      nodes[i][j].x = map_x + i*grid_resolution; 
      nodes[i][j].y = map_y + j*grid_resolution;

      // Updating obstacle information
      if ((int) map[grid_size*j+i] == 100){ // 100 = obstacle
        nodes[i][j].bObstacle = true;
      }   
    }//j
  } //i 


}


int main(int argc, char **argv){

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  
  Sensing sensing(n);

  ros::Subscriber sub_costmap = n.subscribe("/costmap_node/costmap/costmap", 1, &Sensing::costmapCb, &sensing);
  
  ros::spin();
  
  return 0;
}