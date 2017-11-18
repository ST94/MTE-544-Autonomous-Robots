//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 3
//
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>

ros::Publisher marker_pub;

#define PI 3.14159265
#define TAGID 0
#define NumWayPoints 300
#define NumWayPointsInLine 15
#define NumConnectedNodes 5
#define NumTravelPoints 2


int pathId = 5000000;

enum WayPointType { generated, given, unoccupied, occupied};

class WayPoint
{
  public:
    int id;
    WayPointType type;
    double x, y;
    int childNodes [5];
    double distances [5];
    int previousNode;
    double cost;
};

std::vector<int> occupancyGrid;
std::vector<WayPoint> waypoints;
std::vector<int> travelPoints;
double mapSizeX;
double mapSizeY;
double mapResolution;
double currentX;
double currentY;
bool firstMapCallBack = true;
bool graphCreated = false;
bool pathFound = false;

//Drawing Marker Messages and Utility Functions
double randomGenerate()
{
  double random = (double)rand()/ RAND_MAX;
  return random;
}


visualization_msgs::Marker build_marker( double X, double Y, double Yaw) {
    visualization_msgs::Marker marker;
       marker.header.frame_id = "base_link";
       marker.header.stamp = ros::Time();
       marker.id = pathId;
       //pathId+=1;
       marker.type = visualization_msgs::Marker::ARROW;
       marker.action = visualization_msgs::Marker::ADD;
       marker.pose.position.x = X;
       marker.pose.position.y = Y;
       marker.pose.position.z = 0;
       marker.pose.orientation = tf::createQuaternionMsgFromYaw(Yaw);
       marker.scale.x = 0.3;
       marker.scale.y = 0.1;
       marker.scale.z = 0.1;
       marker.color.a = 1.0; // Don't forget to set the alpha!
       marker.color.r = 0.0;
       marker.color.g = 1.0;
       marker.color.b = 0.0;
      //only if using a MESH_RESOURCE marker type:
       marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
   return marker;
}

visualization_msgs::Marker build_waypoint_marker(WayPoint waypoint, int id) {
    visualization_msgs::Marker marker;
       marker.header.frame_id = "map";
       marker.header.stamp = ros::Time();
       marker.id = id;
       //pathId+=1;
       marker.type = visualization_msgs::Marker::SPHERE;
       marker.action = visualization_msgs::Marker::ADD;
       marker.pose.position.x = waypoint.x;
       marker.pose.position.y = waypoint.y;
       marker.pose.position.z = 0;
       // marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
       marker.scale.x = 0.2;
       marker.scale.y = 0.2;
       marker.scale.z = 0.2;
       marker.color.a = 1.0; // Don't forget to set the alpha!
       marker.color.r = 0.0;
       marker.color.g = 0.0;
       marker.color.b = 0.0;

       switch(waypoint.type){
        case unoccupied: marker.color.b = 1.0;
          // ROS_INFO("free");
          break;
        case occupied: marker.color.r = 1.0;
          marker.lifetime = ros::Duration(10.0);
          // ROS_INFO("occupied"); 
          break;
        case given: marker.color.b = 1.0;
          marker.color.r = 1.0;
          break;
       }
      //only if using a MESH_RESOURCE marker type:
       ROS_INFO("Measurements: X: [%f], Y: [%f], Id: [%d]",waypoint.x , waypoint.y, waypoint.id);
       marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
   return marker;
}

visualization_msgs::Marker build_graph_lines(WayPoint waypoint) {
    visualization_msgs::Marker marker;
       marker.header.frame_id = "base_link";
       marker.header.stamp = ros::Time();
       marker.id = waypoint.id+10000;
       marker.type = visualization_msgs::Marker::LINE_LIST;
       marker.action = visualization_msgs::Marker::ADD;
       marker.pose.position.z = 0;
       marker.scale.x = 0.01;
       marker.color.a = 1.0; // Don't forget to set the alpha!
       marker.color.r = 0.0;
       marker.color.g = 1.0;
       marker.color.b = 0.0;

       geometry_msgs::Point origin;
       origin.x = waypoint.x;
       origin.y = waypoint.y;
        ROS_WARN("Origin: X: [%f], Y: [%f], Id: [%d]",waypoint.x , waypoint.y, waypoint.id);
       for(int i = 0; i < NumConnectedNodes; i++) {
         geometry_msgs::Point p;
         p.x = waypoints[waypoint.childNodes[i]].x;
         p.y = waypoints[waypoint.childNodes[i]].y;
         p.z = 0; //not used
         marker.points.push_back(p);
         marker.points.push_back(origin);
         ROS_INFO("nearest point: X: [%f], Y: [%f], id: [%i]",p.x , p.y, waypoints[waypoint.childNodes[i]].id);
   }
      //only if using a MESH_RESOURCE marker type:
       marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
   return marker;
}

visualization_msgs::Marker build_path_line(std::vector<WayPoint> path) {
  visualization_msgs::Marker marker;
       marker.header.frame_id = "base_link";
       marker.header.stamp = ros::Time();
       marker.id = 20000;
       marker.type = visualization_msgs::Marker::LINE_LIST;
       marker.action = visualization_msgs::Marker::ADD;
       marker.pose.position.z = 0;
       marker.scale.x = 0.05;
       marker.color.a = 1.0; // Don't forget to set the alpha!
       marker.color.r = 0.0;
       marker.color.g = 1.0;
       marker.color.b = 1.0;
       for(int i = 0; i < path.size()-1; i++) {
         geometry_msgs::Point point1;
         point1.x = path[i].x;
         point1.y = path[i].y;
         point1.z = 0; //not used
         geometry_msgs::Point point2;
         point2.x = path[i+1].x;
         point2.y = path[i+1].y;
         point2.z = 0; //not used
         marker.points.push_back(point1);
         marker.points.push_back(point2);
   }
      //only if using a MESH_RESOURCE marker type:
       marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
   return marker;
}

void drawCurve(int k) {
   // Curves are drawn as a series of stright lines
   // Simply sample your curves into a series of points

   double x = 0;
   double y = 0;
   double steps = 50;

   visualization_msgs::Marker lines;
   lines.header.frame_id = "/map";
   lines.id = k; //each curve must have a unique id or you will overwrite an old ones
   lines.type = visualization_msgs::Marker::LINE_STRIP;
   lines.action = visualization_msgs::Marker::ADD;
   lines.ns = "curves";
   lines.scale.x = 0.1;
   lines.color.r = 1.0;
   lines.color.b = 0.2*k;
   lines.color.a = 1.0;

   //generate curve points
   for(int i = 0; i < steps; i++) {
       geometry_msgs::Point p;
       p.x = x;
       p.y = y;
       p.z = 0; //not used
       lines.points.push_back(p); 

       //curve model
       x = x+0.1;
       y = sin(0.1*i*k);   
   }

   //publish new curve
   marker_pub.publish(lines);
}

bool wayPointIsFree(WayPoint waypoint) {
  int xCeiling = ceil((waypoint.x + 0.9) * 10.0);
  int xFloor = floor((waypoint.x + 0.9) * 10.0);
  int yCeiling = ceil((waypoint.y + 4.9) * 10.0);
  int yFloor = floor((waypoint.y + 4.9) * 10.0);

  //check the four squares closest to the waypoint
  if (occupancyGrid[xFloor + yFloor * 100] > 75)
  {
    return false;
  }

   if (occupancyGrid[xFloor + yCeiling * 100] > 75)
  {
    return false;
  }

    if (occupancyGrid[xCeiling + yFloor * 100] > 75)
  {
    return false;
  }

    if (occupancyGrid[xCeiling + yCeiling * 100] > 75)
  {
    return false;
  }

  return true;
}

bool pathIsFree(WayPoint A, WayPoint B) {
  for (int i = 1; i < NumWayPointsInLine; ++i)
  {
    WayPoint intermediate;
    intermediate.x = A.x + ((B.x-A.x)*i)/NumWayPointsInLine;
    intermediate.y = A.y + ((B.y-A.y)*i)/NumWayPointsInLine;
    if (!wayPointIsFree(intermediate)) {
      return false;
    }
  }
  return true;
}

double wayPointDistance(WayPoint A, WayPoint B) {
  return sqrt(pow((A.x - B.x), 2.0) + pow((A.y - B.y), 2.0));
}

//Populating the map with waypoints
void createWayPoints() {
  int waypointId = 0;
  for (int i = 0; i < NumWayPoints; i++){
      WayPoint waypoint;
      waypoint.x = -0.9 + randomGenerate() * 9.8;
      waypoint.y = -4.9 + randomGenerate() * 9.8;
      waypoint.id = waypointId;
      if (wayPointIsFree(waypoint)) {
        waypoint.type = unoccupied;
        waypoints.push_back(waypoint);
        waypointId++;
        ROS_INFO("waypoint X:[%f] Y:[%f] id:[%d]", waypoint.x, waypoint.y, waypoint.id);
      } else {
        waypoint.type = occupied;
      }
      marker_pub.publish(build_waypoint_marker(waypoint, i));
    }
}

void addWayPointsToGraph() {
  WayPoint waypoint;
  waypoint.x = 0;
  waypoint.y = 0;
  waypoint.id = waypoints.size();
  waypoint.type = given;
  travelPoints.push_back(waypoint.id);
  waypoints.push_back(waypoint);
  marker_pub.publish(build_waypoint_marker(waypoint, waypoint.id));
  waypoint.x = 7;
  waypoint.y = 0;
  waypoint.id = waypoints.size();
  waypoint.type = given;
  travelPoints.push_back(waypoint.id);
  waypoints.push_back(waypoint);
  marker_pub.publish(build_waypoint_marker(waypoint, waypoint.id));
}

//Generating connected graph from waypoints
void generateGraphFromWayPoints() {
  for (int i = 0; i < waypoints.size(); i++) {
    ROS_WARN("working on waypoint [%i] at location x:[%f], y:[%f]", i, waypoints[i].x, waypoints[i].y);
    std::vector<double> distances;
    std::vector<int> waypointIds;
    for (int nodes = 0; nodes < NumConnectedNodes; nodes++) {
      distances.push_back(1000);
      waypointIds.push_back(1000);
    }

    for (int j = 0; j < waypoints.size(); j++) {
        
      if (i!=j) {
        double distance = wayPointDistance(waypoints[i], waypoints[ j]);
        if (distance < distances[NumConnectedNodes-1]) {
          int k = NumConnectedNodes-1;
          while (distance < distances[k-1] && k > 0) {
            k--;
          }
          if (pathIsFree(waypoints[i], waypoints[j])) {
            // ROS_INFO("new distance : [%f], old distances:[%f, %f, %f, %f, %f] at position %i", distance, distances[0],distances[1],distances[2],distances[3],distances[4], k);
            distances.insert(distances.begin()+k, distance);
            distances.pop_back();

            waypointIds.insert(waypointIds.begin()+k, waypoints[j].id);
            waypointIds.pop_back();
          }
        }
      }
    }
    for (int l = 0; l < waypointIds.size(); ++l)
    {
      waypoints[i].childNodes[l] = waypointIds[l];
      waypoints[i].distances[l] = distances[l];
      ROS_INFO("nearest point: X: [%f], Y: [%f], distance: [%f]", waypoints[waypointIds[l]].x , waypoints[waypointIds[l]].y, distances[l]);
    }
    marker_pub.publish(build_graph_lines(waypoints[i]));
  }
  ROS_INFO("done creating graph");
  graphCreated = true;
}

//Finding the best path between travelpoints
std::vector<WayPoint> findPathBetweenWayPoints(WayPoint A, WayPoint B) {
  ROS_ERROR("travelling from node:[%d] to node:[%d]", A.id, B.id);
  //Start from A and find path to B
  std::vector<WayPoint> openSet;
  std::vector<WayPoint> closedSet;
  std::vector<WayPoint> Path;
  A.previousNode = A.id;
  A.cost = 0;
  closedSet.push_back(A);
  // ROS_WARN("closed node id:[%d], cost:[%f]", A.id, A.cost);
  // ROS_INFO("has child nodes [%d, %d, %d, %d, %d]", A.childNodes[0], A.childNodes[1], A.childNodes[2], A.childNodes[3], A.childNodes[4]);
  WayPoint temp;
  //Adding the initial node
  for (int i = 0; i < NumConnectedNodes; ++i)
  {
    temp = waypoints[A.childNodes[i]];
    temp.previousNode = A.id;
    temp.cost = A.distances[i];
    openSet.push_back(temp);
  }
  //Continuing to add waypoints based on closed set
  while (openSet[0].id != B.id && openSet.size() > 0) {
    // ROS_WARN("closest open node id:[%d], cost:[%f]", openSet[0].id, openSet[0].cost);
    // ROS_INFO("has child nodes [%d, %d, %d, %d, %d]", openSet[0].childNodes[0], openSet[0].childNodes[1], openSet[0].childNodes[2], openSet[0].childNodes[3], openSet[0].childNodes[4]);
    for (int i = 0; i < NumConnectedNodes; ++i) {
      bool skipChildNode = false;
      bool addToOpenSet = true;
      temp = waypoints[openSet[0].childNodes[i]];
      for (int node = 0; node < closedSet.size(); node++) {
        // ROS_INFO("temp id: [%d], closedSet: [%d]", temp.id, closedSet[node].id);
        if (temp.id == closedSet[node].id) {
          skipChildNode = true; //do not check for costs if the child node already exists within the closedSet of nodes
          // ROS_INFO("skip child node: [%d]", temp.id);
          break;
        }
      }
      if (!skipChildNode) {
        temp.cost = wayPointDistance(openSet[0], temp) + openSet[0].cost;
        temp.previousNode = openSet[0].id;
        // ROS_INFO("child node id:[%d], previousNode:[%d], cost:[%f]", temp.id, temp.previousNode, temp.cost);
        for (int j = 0; j < openSet.size(); ++j) {
          // ROS_INFO("openSet contains id:[%d]", openSet[j].id);
          if (temp.id == openSet[j].id) {
            if (temp.cost < openSet[j].cost) {
              // ROS_INFO("update node: [%i]", temp.id);

              openSet.erase(openSet.begin()+j);
            } else {
              addToOpenSet = false; //if the node already exists in the openSet, but this path has a greater cost, do not add to the openSet.
            }
            break;
          }
        }
        if (addToOpenSet){
          bool hasNotBeenAdded = true;
          //check to see if cost is lower than any other value within the openSet
          for (int i = 0; i < openSet.size(); ++i) {
            if (temp.cost < openSet[i].cost) {
              openSet.insert(openSet.begin()+i, temp);
              // ROS_INFO("added node:[%i] to open set with previousNode:[%i]", temp.id, temp.previousNode);
              hasNotBeenAdded = false;
              break;
            }
          }
          if (hasNotBeenAdded) {
            openSet.push_back(temp);
            // ROS_INFO("added node:[%i] to open set with previousNode:[%i]", temp.id, temp.previousNode);
          }
        }
      }
    }
    // ROS_WARN("added node:[%d] to closed set with previousNode:[%i] and cost:[%f]", openSet[0].id, openSet[0].previousNode, openSet[0].cost);
    closedSet.push_back(openSet[0]);
    openSet.erase(openSet.begin());
  }
  if (openSet.size() > 0)
  {
      closedSet.push_back(openSet[0]);
  }
  // ROS_ERROR("travelling from node:[%d] to node:[%d]", A.id, B.id);
  ROS_INFO("DISTANCE BETWEEN POINTS: [%f]", wayPointDistance(A, B));
  for (int i = 0; i < closedSet.size(); ++i)
  {
    ROS_INFO("closedSet info: id:[%d], cost:[%f]", closedSet[i].id, closedSet[i].cost);
    if (B.id == closedSet[i].id)
    {
      temp = closedSet[i];
    }
  }
  if (temp.id != B.id) {
    ROS_INFO("unable to find the path");
    return Path;
  }
  ROS_INFO("path node id:[%d], previousNode:[%d], cost:[%f]", temp.id, temp.previousNode, temp.cost);
  Path.insert(Path.begin(), temp);
  while (temp.id != A.id) {
    for (int i = 0; i < closedSet.size(); ++i) {
      if (temp.previousNode == closedSet[i].id) {
        temp = closedSet[i];
        ROS_INFO("path node id:[%d], previousNode:[%d], cost:[%f]", temp.id, temp.previousNode, temp.cost);
        Path.insert(Path.begin(), temp);
        break;
      }
    }
  }
  ROS_INFO("DONE");
  marker_pub.publish(build_path_line(Path));
  return Path;
  }

void findPaths() {
  findPathBetweenWayPoints(waypoints[travelPoints[0]], waypoints[travelPoints[1]]);
  pathFound = true;
}

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg) {
  if (firstMapCallBack == true) {
    for (int i = 0; i<msg.data.size(); i++) {
      occupancyGrid.push_back(msg.data[i]);
    }
    mapSizeX = msg.info.width;
    mapSizeY = msg.info.height;
    mapResolution = msg.info.resolution;
    firstMapCallBack = false;
  }
}

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg) {
    int i;
    for(i = 0; i < msg.name.size(); i++) {
        if(msg.name[i] == "mobile_base") 
        break;
    }
    currentX = msg.pose[i].position.x;
    currentY = msg.pose[i].position.y;
    double Yaw = tf::getYaw(msg.pose[i].orientation);
    // ROS_INFO("Measurements: X: [%f], Y: [%f], Yaw: [%f]",X, Y, Yaw);
    marker_pub.publish(build_marker(currentX, currentY, Yaw));
}
//Callback function for the Position topic (LIVE)

// void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
// {
//  //This function is called when a new position message is received
//  double X = msg.pose.pose.position.x; // Robot X psotition
//  double Y = msg.pose.pose.position.y; // Robot Y psotition
//    double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

//  ROS_INFO("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
//   marker_pub.publish(build_marker(X, Y, Yaw));
// }

//generate interconnected graphs with waypoints

int main(int argc, char **argv)
{
  //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    // ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(5);    //20Hz update rate

    tf::TransformBroadcaster *br = new tf::TransformBroadcaster;
    tf::Transform *tform = new tf::Transform;

    tform->setOrigin(
        tf::Vector3(0, 0, 0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);

  tform->setRotation(q);

    while (ros::ok())
    {
      loop_rate.sleep(); //Maintain the loop rate
      ros::spinOnce();   //Check for new messages

      br->sendTransform(
          tf::StampedTransform(*tform, ros::Time::now(), "base_link", "map"));
      if (!graphCreated) {
        createWayPoints();
        addWayPointsToGraph();
        generateGraphFromWayPoints();
      } 
      if (!pathFound) {
        findPaths();
      }
      //Main loop code goes here:
      vel.linear.x = 0.0; // set linear speed
      vel.angular.z = 0.3; // set angular speed
      // ROS_INFO("publishing speed");
      // velocity_publisher.publish(vel); // Publish the command velocity
  }
    return 0;
}
