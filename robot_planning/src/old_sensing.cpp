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
    int global_grid_size;
    int grid_connections;

    std::vector<float> gframe; // origin of global frame wrt map frame
    // std::vector<float> lframe;

    nav_msgs::OccupancyGrid global_map;

    std::vector<float> start;
    std::vector<float> goal;

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
    vector< pair<float,float> > path; // store path top: current node, bottom: goal node

    // Publisher and msg for the controller
    ros::Publisher pub_path;
    ros::Publisher pub_global_costmap;
    nav_msgs::OccupancyGrid global_costmap;

    robot_planning::trajData path_msg;

    Sensing(ros::NodeHandle &nh);  // constructor
    void costmapCb(const nav_msgs::OccupancyGridConstPtr grid); // Callback for costmap
    void init_global_map();
    bool solve_astar();
    void printPath();
    void getPath();
    void publishPath();
    void publishGlobalMap();
    void get_start_goal_nodes();


    // void PIDController();
    // void RobotState();

  private:
    void make_connections();      // add neighbors to the nodes 

};

Sensing::Sensing(ros::NodeHandle &nh){ //constructor
  pub_path = nh.advertise<robot_planning::trajData>("planned_path", 1);
  ros::Publisher pub_global_costmap = nh.advertise<nav_msgs::OccupancyGrid>("global_costmap",1);

  ROS_INFO("Sensing node initialized ...");
  if(nh.hasParam("costmap_node/costmap/width")){
    nh.getParam("costmap_node/costmap/width", grid_size);
    nh.getParam("costmap_node/costmap/resolution", grid_resolution);
    nh.getParam("robot_info/grid_connections", grid_connections);
    nh.getParam("robot_info/start", start);
    nh.getParam("robot_info/goal", goal);
    nh.getParam("robot_info/global_origin", gframe);
  }  
  else
    ROS_ERROR("Did not find parameters");

  grid_size /= grid_resolution;
  global_grid_size /= grid_resolution;

  nodes.resize(global_grid_size, vector<sNode>(global_grid_size));   // allocating memory to initialized vector 
  
  
  init_global_map();


  // Add neighbors
  // make_connections();

  // get_start_goal_nodes();

  //Global map


  // std_msgs::Header header;
  // header.frame_id = "map";

  // nav_msgs::MapMetaData info;
  // info.width = global_grid_size;
  // info.height = global_grid_size;
  // info.resolution = grid_resolution;
  
  // for (unsigned int i = 0; i < info.width; i++)
  //   for (unsigned int j = 0; j < info.height; j++)
  //     map.Insert(Cell(i , j, info.width, msg->data[x+ info.width * y]));

  // global_map.info.origin.position.x = gframe[0];
  // global_map.info.origin.position.y = gframe[1];
  // global_map.info.origin.position.z = 0;
  
  // global_map.info.origin.orientation.x = 0;
  // global_map.info.origin.orientation.y = 0;
  // global_map.info.origin.orientation.z = 0;
  // global_map.info.origin.orientation.w = 1;

  // vector<int> temp_var(global_grid_size*global_grid_size, -1);
  // global_map.data; 


}

void Sensing::init_global_map(){
	global_costmap.info.resolution = global_grid_size;
    global_costmap.info.origin.position.x = gframe[0];
    global_costmap.info.origin.position.y = gframe[1];
    global_costmap.info.width             = global_grid_size;
    global_costmap.info.height            = global_grid_size;


    size_t size = global_costmap.info.width * global_costmap.info.height;
    global_costmap.data.reserve(size);
    for(int i=0;i<size;i++)
    {
      global_costmap.data.push_back(-1);
    }
}

void Sensing::get_start_goal_nodes(){
  float xstart = start[0];
  float ystart = start[1]; 
  float xgoal = goal[0];
  float ygoal = goal[1];

  // start_i = floor(xstart/grid_resolution);
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
  
  robot_planning::state state_msg;
  path_msg.data.clear();

  while (p->parent != nullptr)
  { 
        
    state_msg.x = p->x;
    state_msg.y = p->y;

    path_msg.data.insert(path_msg.data.begin()  ,state_msg); 
    // Set next node to this node's parent
    p = p->parent;
  }

}

void Sensing::publishPath(){
  getPath();
  pub_path.publish(path_msg);
}

void Sensing::publishGlobalMap(){
  pub_global_costmap.publish(global_costmap);
}

void Sensing::costmapCb(const nav_msgs::OccupancyGridConstPtr grid){ 
  
  grid_resolution = grid->info.resolution;   
  
  // local map location in odom frame (fixed)
  float map_x = grid->info.origin.position.x;
  float map_y = grid->info.origin.position.y; 

  // local map location in global map (fixed)
  float gl_x = map_x - gframe[0]; 
  float gl_y = map_y - gframe[1];

  auto local_map =  grid->data; 
  
  // Define global nodes positions in terms of x and y
  for (int i {0}; i< grid_size; i++){
    for (int j {0}; j< grid_size; j++){      
    
      // nodes[i][j].x = gframe[0] + i*grid_resolution; 
      // nodes[i][j].y = gframe[1] + j*grid_resolution;

      // Converting local indices to global indices
      int gi = i + int(gl_x/grid_resolution);
      int gj = j + int(gl_y/grid_resolution);

      nodes[gi][gj].x = gframe[0] + gi*grid_resolution; 
      nodes[gi][gj].y = gframe[1] + gj*grid_resolution;

      global_map.data[global_grid_size*gj+gi] = local_map[grid_size*j+i];

      // Updating obstacle information
      if ((int) local_map[grid_size*j+i] == 100){ // 100 = obstacle
        nodes[gi][gj].bObstacle = true;
      }   
    }//j
  } //i 

  cout<< "map_x: " << map_x << endl;
  cout<< "map_y: " << map_y << endl;

  // cout << nodes[1][1].x << endl;
  // solve_astar();
  // printPath();

}

int main(int argc, char **argv){

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  
  Sensing sensing(n);

  ros::Subscriber sub_costmap = n.subscribe("/costmap_node/costmap/costmap", 1, &Sensing::costmapCb, &sensing);
  
  ros::Rate loop_rate(10);

  while(ros::ok()){

    // sensing.publishPath();
    sensing.publishGlobalMap();
    ros::spinOnce();
    
    loop_rate.sleep();
  }

  return 0;
}