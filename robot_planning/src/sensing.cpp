#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"


#include "robot_planning/trajData.h"
#include "robot_planning/state.h"


using namespace std;

class Sensing{
   
  public:
    double grid_resolution;
    
    int grid_size;
    int grid_connections;

    std::vector<float> current {0,0};
    std::vector<float> goal {0,0};

    float map_x = 0;
    float map_y = 0; 

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
    
    sNode *nodeStart {};
    sNode *nodeEnd {};

    vector< pair<float,float> > path; // store path top: current node, bottom: goal node
    // Publisher and msg for the controller
    ros::Publisher pub_path;
    robot_planning::trajData path_msg;

    Sensing(ros::NodeHandle &nh);  // constructor
    void costmapCb(const nav_msgs::OccupancyGridConstPtr grid); // Callback for costmap
    void odomCb(nav_msgs::Odometry::ConstPtr msg); // Callback for odom
    
    void init_global_map();
    bool solve_astar();
    void printPath();
    void getPath();
    void publishPath();
    void get_start_end_nodes();

  private:
    void make_connections();      // add neighbors to the nodes 

};

Sensing::Sensing(ros::NodeHandle &nh){ //constructor
  pub_path = nh.advertise<robot_planning::trajData>("planned_path", 1);

  ROS_INFO("Sensing node initialized ...");
  if(nh.hasParam("costmap_node/costmap/width")){
    nh.getParam("costmap_node/costmap/width", grid_size);
    nh.getParam("costmap_node/costmap/resolution", grid_resolution);
    nh.getParam("planner/grid_connections", grid_connections);
    // nh.getParam("planner/start", start);
    nh.getParam("planner/goal", goal);
  }  
  else
    ROS_ERROR("Did not find parame");

  grid_size /= grid_resolution;

  nodes.resize(grid_size, vector<sNode>(grid_size));   // allocating memory to initialized vector 
  // cout<<"goal => x: " << goal[0] << " y: " << goal[1] << endl;
  // Add neighbors
  // ROS_INFO("Connection");
  make_connections();

  // get_start_end_nodes();

}

void Sensing::get_start_end_nodes(){
  int start_node_i = floor((current[0]-map_x)/grid_resolution);
  int start_node_j = floor((current[1]-map_y)/grid_resolution);

  int goal_node_i = floor((goal[0]-map_x)/grid_resolution);
  int goal_node_j = floor((goal[1]-map_y)/grid_resolution);

  nodeStart = &nodes[start_node_i][start_node_j];
  nodeEnd = &nodes[goal_node_i][goal_node_j];

  cout<<"start=> x: " << current[0] << " y: " << current[1] << endl;
  cout<<"start=> i: " << start_node_i << " j: " << start_node_j << endl;
  cout<<"goal => x: " << goal[0] << " y: " << goal[1] << endl;
  cout<<"goal => i: " << goal_node_i << " j: " << goal_node_j << endl;
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
  get_start_end_nodes();
  // cout<<"start=> x: " << nodeStart->x << " y: " << nodeStart->y << endl;
  // cout<<"end=> x: " << nodeEnd->x << " y: " << nodeEnd->y << endl;
  
  // ROS_INFO("Solve astar started");
  
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
      // cout << "nodecurrent: " << nodeCurrent->x << "  "<<nodeCurrent->y << endl; 
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
    if(nodeCurrent == nodeEnd){
      ROS_INFO("Found path to Goal");
      return true;
    }
    else if(listNotTestedNodes.empty()){
      ROS_ERROR("List Got Empty");
      return false;
    }
    cout << nodeEnd->parent << endl;
    return true;
}


void Sensing::printPath(){
  sNode *p = nodeEnd;

  cout << "-----------------------------------" << endl;
  while (p->parent != nullptr)
  {
    cout << " x: " << p->x << " y: " << p->y; 
    // Set next node to this node's parent
    p = p->parent;
  }
  cout << endl;
  cout << "-----------------------------------" << endl;
}

void Sensing::getPath(){
  sNode *p = nodeEnd;
  cout << p->parent << endl;
  robot_planning::state state_msg;
  path_msg.data.clear();

  while (p->parent != nullptr)
  { 
        
    // ROS_INFO("Getting Path . . .");
    state_msg.x = p->x;
    state_msg.y = p->y;
    
    // cout << state_msg.y << endl;

    path_msg.data.insert(path_msg.data.begin(), state_msg); 
    // Set next node to this node's parent
    p = p->parent;
  }

  // cout << path_msg.data[0].x << endl;
}

void Sensing::publishPath(){
  if (!(path_msg.data).empty()){
    printPath();
    pub_path.publish(path_msg);
  }
  else
    ROS_ERROR("Path not found !! ");
}


void Sensing::odomCb(nav_msgs::Odometry::ConstPtr msg){
  
  current[0] = msg->pose.pose.position.x;
  current[1] = msg->pose.pose.position.y;

}

void Sensing::costmapCb(const nav_msgs::OccupancyGridConstPtr grid){ 
  
  grid_resolution = grid->info.resolution;   
  
  // local map location in odom frame (fixed)
  map_x = grid->info.origin.position.x;
  map_y = grid->info.origin.position.y; 

  auto map =  grid->data; 
  
  // Define global nodes positions in terms of x and y
  for (int i {0}; i< grid_size; i++){
    for (int j {0}; j< grid_size; j++){      
    
      nodes[i][j].x = map_x + i*grid_resolution + grid_resolution/2; 
      nodes[i][j].y = map_y + j*grid_resolution + grid_resolution/2;

      // Updating obstacle information
      if ((int) map[grid_size*j+i] > 5){ // 100 = obstacle
        nodes[i][j].bObstacle = true;
      }
      else
        nodes[i][j].bObstacle = false;    
    }//j
  } //i 

  bool GotPath = solve_astar();
  if (GotPath){
    ROS_INFO("Got Path !!");
    getPath();
  }  
  else
    ROS_ERROR("Astar failed !!");
  // printPath();

}

int main(int argc, char **argv){

  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  
  Sensing sensing(nh);

  ros::Subscriber sub_costmap = nh.subscribe("/costmap_node/costmap/costmap", 1, &Sensing::costmapCb, &sensing);
  ros::Subscriber subs_odom = nh.subscribe("/odom", 1, &Sensing::odomCb, &sensing);

  int PLANNING_FREQ = 10;
  nh.getParam("planner/planning_freq", PLANNING_FREQ);

  ros::Rate loop_rate(PLANNING_FREQ);

  while(ros::ok()){

    ros::spinOnce();
    sensing.publishPath();
    
    loop_rate.sleep();
  }

  return 0;
}