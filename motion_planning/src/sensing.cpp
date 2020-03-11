#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <stack>  

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
    stack < pair<float,float> > path; // store path top: current node, bottom: goal node

    // Controller variables
    pair<int, int> state; // Current State
    pair<int, int> prev_state; // Previous State

    Sensing(ros::NodeHandle &nh);  // constructor
    void costmapCb(const nav_msgs::OccupancyGridConstPtr grid); // Callback for costmap
    bool solve_astar();
    void printPath();
    void getPath();
    // void PIDController();
    // void RobotState();

  private:
    void make_connections();      // add neighbors to the nodes 

};

Sensing::Sensing(ros::NodeHandle &nh){ //constructor

  ROS_INFO("Sensing node initialized ...");
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

bool Sensing::solve_astar(){
  sNode *nodeStart = &nodes[10][10];
  sNode *nodeEnd = &nodes[15][15];

  cout << "Solve astar started" << endl;
  // Reset Navigation Graph - default all node states
  for (int x = 0; x < grid_size; x++)
    for (int y = 0; y < grid_size; y++)
    {
      nodes[x][y].bVisited = false;
      nodes[x][y].fGlobalGoal = INFINITY;
      nodes[x][y].fLocalGoal = INFINITY;
      nodes[x][y].parent = nullptr;  // No parents
    }

    auto distance = [](sNode* a, sNode* b){ // For convenience
      return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
    };

    auto heuristic = [distance](sNode* a, sNode* b){ // So we can experiment with heuristic
      return distance(a, b);
    };

    // Setup starting conditions
    sNode *nodeCurrent = nodeStart;
    nodeStart->fLocalGoal = 0.0f;
    nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);

    // Add start node to not tested list - this will ensure it gets tested.
    // As the algorithm progresses, newly discovered nodes get added to this
    // list, and will themselves be tested later
    list<sNode*> listNotTestedNodes;
    listNotTestedNodes.push_back(nodeStart);

    // if the not tested list contains nodes, there may be better paths
    // which have not yet been explored. However, we will also stop 
    // searching when we reach the target - there may well be better
    // paths but this one will do - it wont be the longest.
    while (!listNotTestedNodes.empty() && nodeCurrent != nodeEnd)// Find absolutely shortest path // && nodeCurrent != nodeEnd)
    {
      // Sort Untested nodes by global goal, so lowest is first
      listNotTestedNodes.sort([](const sNode* lhs, const sNode* rhs){ return lhs->fGlobalGoal < rhs->fGlobalGoal; } );
      
      // Front of listNotTestedNodes is potentially the lowest distance node. Our
      // list may also contain nodes that have been visited, so ditch these...
      while(!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
        listNotTestedNodes.pop_front();

      // ...or abort because there are no valid nodes left to test
      if (listNotTestedNodes.empty())
        break;

      nodeCurrent = listNotTestedNodes.front();
      nodeCurrent->bVisited = true; // We only explore a node once
      
          
      // Check each of this node's neighbours...
      for (auto nodeNeighbour : nodeCurrent->vecNeighbours)
      {
        // ... and only if the neighbour is not visited and is 
        // not an obstacle, add it to NotTested List
        if (!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == 0)
          listNotTestedNodes.push_back(nodeNeighbour);

        // Calculate the neighbours potential lowest parent distance
        float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour);

        // If choosing to path through this node is a lower distance than what 
        // the neighbour currently has set, update the neighbour to use this node
        // as the path source, and set its distance scores as necessary
        if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal)
        {
          nodeNeighbour->parent = nodeCurrent;
          nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;

          // The best path length to the neighbour being tested has changed, so
          // update the neighbour's score. The heuristic is used to globally bias
          // the path algorithm, so it knows if its getting better or worse. At some
          // point the algo will realise this path is worse and abandon it, and then go
          // and search along the next best path.
          nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, nodeEnd);
        }
      } 
    }

    return true;
}


void Sensing::printPath(){
  sNode *nodeStart = &nodes[10][10];
  sNode *nodeEnd = &nodes[15][15];
  sNode *p = nodeEnd;
  
  while (p->parent != nullptr)
  {
    cout << "x: " << p->x << " y: " << p->y; 
    // Set next node to this node's parent
    p = p->parent;
  }
  cout << endl;
}

void Sensing::getPath(){

  sNode *nodeStart = &nodes[10][10];
  sNode *nodeEnd = &nodes[15][15];
  sNode *p = nodeEnd;
  
  while (p->parent != nullptr)
  {
    path.push(make_pair(p->x,p->y)); 
    // Set next node to this node's parent
    p = p->parent;
  }

}

// void Sensing::RobotStateCbk(nav_msgs::Odometry::ConstPtr msg){
  
//   state.first  = msg->pose.position.x;
//   state.second = msg->pose.position.y;

// }

// void Sensing::PIDController(){


// }

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


  // cout << nodes[1][1].x << endl;
  solve_astar();
  printPath();

}

int main(int argc, char **argv){

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  
  Sensing sensing(n);

  ros::Subscriber sub_costmap = n.subscribe("/costmap_node/costmap/costmap", 1, &Sensing::costmapCb, &sensing);
  
  ros::spin();
  
  return 0;
}