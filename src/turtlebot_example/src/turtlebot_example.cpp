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
#define numWayPoints 100


int pathId = 100000;

enum WayPointType { generated, given, unoccupied, occupied};

class WayPoint
{
  public:
    int id, previousNode;
    WayPointType type;
    double x, y;
    int childNodes [5];
    double cost;

};


std::vector<int> occupancyGrid;
std::vector<WayPoint> waypoints;
double mapSizeX;
double mapSizeY;
double mapResolution;
bool firstMapCallBack = true;
bool mapCreated = false;

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

visualization_msgs::Marker build_waypoint_marker(WayPoint waypoint) {
    visualization_msgs::Marker marker;
       marker.header.frame_id = "map";
       marker.header.stamp = ros::Time();
       marker.id = waypoint.id;
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
          ROS_INFO("free");
          break;
        case occupied: marker.color.r = 1.0;
          ROS_INFO("occupied"); 
          break;
       }
      //only if using a MESH_RESOURCE marker type:
       ROS_INFO("Measurements: X: [%f], Y: [%f], Id: [%d]",waypoint.x , waypoint.y, waypoint.id);
       marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
   return marker;
}

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg) 
{
    int i;
    for(i = 0; i < msg.name.size(); i++) {
        if(msg.name[i] == "mobile_base") 
        break;
    }
    double X = msg.pose[i].position.x;
    double Y = msg.pose[i].position.y;
    double Yaw = tf::getYaw(msg.pose[i].orientation);
    // ROS_INFO("Measurements: X: [%f], Y: [%f], Yaw: [%f]",X, Y, Yaw);
    marker_pub.publish(build_marker(X, Y, Yaw));
}

bool wayPointIsFree(WayPoint waypoint)
{
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

double wayPointDistance(WayPoint A, WayPoint B)
{
  return sqrt(pow((A.x - B.x), 2) + pow((A.x - B.x), 2));
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

//Populating the open and closed sample set
void createWayPoints()
{
  int waypointId = 0;
  for (int i = 0; i < numWayPoints; i++){
      WayPoint waypoint;
      waypoint.x = -0.9 + randomGenerate() * 9.8;
      waypoint.y = -4.9 + randomGenerate() * 9.8;
      waypoint.id = waypointId;
      // ROS_INFO("waypoint X:[%f] Y:[%f] id:[%d]", waypoint.x, waypoint.y, waypoint.id);
      waypointId++;
      waypoints.push_back(waypoint);
      if (wayPointIsFree(waypoint)) {
        waypoint.type = unoccupied;
      } else {
        waypoint.type = occupied;
      }
      
      marker_pub.publish(build_waypoint_marker(waypoint));
    }
  // ROS_INFO("done");
  mapCreated = true;
}

//Example of drawing a curve
void drawCurve(int k) 
{
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

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
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
    ros::Rate loop_rate(20);    //20Hz update rate

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
      if (!mapCreated) {
        createWayPoints();
      } else {
    
      //Main loop code goes here:
      vel.linear.x = 0.0; // set linear speed
      vel.angular.z = 0.3; // set angular speed
      // ROS_INFO("publishing speed");

      velocity_publisher.publish(vel); // Publish the command velocity
    }
  }
    return 0;
}
