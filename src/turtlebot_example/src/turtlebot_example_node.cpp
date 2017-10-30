//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <numeric>
#include <eigen/Eigen/Dense>

ros::Publisher pose_publisher;

using namespace Eigen;
using namespace std;

const int NUM_PARTICLES = 16;
const int UPDATE_RATE = 3;
#define PI 3.14159265

MatrixXf rotational_matrix(3,3);

struct Pose {
    float x;
    float y;
    float yaw;
};

MatrixXf estimates [NUM_PARTICLES];
MatrixXf predicted [NUM_PARTICLES];
MatrixXf variance(3,3);
MatrixXf measurement(3,1);
MatrixXf velocity(3,1);

double weights [NUM_PARTICLES];

short sgn(int x) { return x >= 0 ? 1 : -1; }

void generate_initial_samples(int num_particles) 
{
    int i;
    int j;
    int position = 0;
    ROS_INFO("generating initial samples");
    for (i = 0; i < NUM_PARTICLES; i++) {
            double random = (double)rand()/ RAND_MAX;
            MatrixXf estimate(3,1);
            estimate(0,0) = - 0.5 + random;
            estimate(1,0) = - 0.5 + random;
            estimate(2,0) = - PI + random * 2 * PI;
            //ROS_INFO("X: [%f], Y: [%f], Yaw: [%f]", estimate(0,0), estimate(1,0), estimate(2,0));
            estimates[position] = estimate;
            position++;
    }
    ROS_INFO("inital samples generated");
}


void set_rotational_matrix()
{
    rotational_matrix(0,0) = cos(measurement(2,0));
    rotational_matrix(0,1) = -sin(measurement(2,0));
    rotational_matrix(0,2) = 0;
    rotational_matrix(1,0) = sin(measurement(2,0));
    rotational_matrix(1,1) = cos(measurement(2,0));
    rotational_matrix(1,2) = 0;
    rotational_matrix(2,0) = 0;
    rotational_matrix(2,1) = 0;
    rotational_matrix(2,2) = 1;
}

double normpdf(Matrix<float, 3, 1> measurement, Matrix<float, 3, 1> expected, Matrix<float, 3, 3> variance) { 

    Matrix<float, 3,1> difference = measurement - expected;
    //double pdf = (1/ sqrt(pow(2.0 * PI, 3) * variance.determinant())) * exp(-0.5 * difference.transpose() * variance.inverse() * difference);
    //double pdf = exp(-0.5 *difference.transpose() * variance.inverse() * difference)/(sqrt(2 * PI * variance.determinant()));
    double pdf = exp(-0.5 * difference.transpose() * variance.inverse() * difference)/(pow(2 * PI, 1.5) * pow(variance.determinant(), 0.5));

    //ROS_INFO("difference: [%f, %f, %f]", difference(0,0), difference(1, 0), difference(2,0));
    //ROS_INFO("numerator: [%f]", difference.transpose() * variance.inverse() * difference);
    // ROS_INFO("denominator: [%f]", (pow(2 * PI, 1.5) * pow(variance.determinant(), 0.5)));
    //ROS_INFO("probability: [%f]", pdf);
    return pdf;
}

double rad_wrap(double angle)
{
    //ROS_INFO("angle %f", angle);
    while (angle < -PI || angle > PI)
    {
        if (angle < 0)
            {
                angle = angle + (2*PI);
            }
        if (angle > 0)
            {
                angle = angle -(2*PI);
            }
    } 
    return angle;
}

void update_predicted() 
{
    int particle;
    for(particle = 0;particle<NUM_PARTICLES;particle++) {
        MatrixXf updated_position(3,1);
        predicted[particle] = estimates[particle] + (1.0/UPDATE_RATE)*rotational_matrix*velocity;
        predicted[particle](2,0) = rad_wrap(predicted[particle](2,0));
        weights[particle] = normpdf(measurement, predicted[particle], variance);
        //ROS_INFO("Predictions: X: [%f], Y: [%f], Yaw: [%f]", predicted[particle](0,0), predicted[particle](1,0), predicted[particle](2,0));
        //ROS_INFO("probability: [%f]", weights[particle]);
    }
    // ROS_INFO("Estimate: X: [%f], Y: [%f], Yaw: [%f]", estimates[1](0,0), estimates[1](1,0), estimates[1](2,0));
    // ROS_INFO("Predictions: X: [%f], Y: [%f], Yaw: [%f]", predicted[1](0,0), predicted[1](1,0), predicted[1](2,0));
    // ROS_INFO("probability: [%f]", weights[1]);
}

void cum_sum_weights(){
    double max_weight;

    for(int i = 1; i < NUM_PARTICLES; i++) {
        weights[i] = weights[i]+weights[i-1];
    }

    max_weight = weights[(NUM_PARTICLES-1)];

    for(int i = 0; i<NUM_PARTICLES ; i++){
        weights[i] = weights[i] / max_weight;
    }
}

void resample() {
    int i;
    for(i = 0; i<NUM_PARTICLES ; i++) {
        double random = (double)rand()/ RAND_MAX;
        int j = 0;
        while (weights[j] < random)
        {
            j++;
        }
        //ROS_INFO("random: %i", j);
        estimates[i] = predicted[j];
        ROS_INFO("Predictions: X: [%f], Y: [%f], Yaw: [%f]", estimates[i](0,0), estimates[i](1,0), estimates[i](2,0));
        //ROS_INFO("Weight: [%f]", weights[j]);
    }

}

void odometry_callback(const nav_msgs::Odometry msg) {
    velocity(0,0) = msg.twist.twist.linear.x;
    velocity(1,0) = msg.twist.twist.linear.y;
    velocity(2,0) = msg.twist.twist.angular.z;
    ROS_WARN("Velocity: X: [%f], Y: [%f], Yaw: [%f]", velocity(0,0), velocity(1,0), velocity(2,0));
};

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg) 
{
    int i;
    for(i = 0; i < msg.name.size(); i++) {
        if(msg.name[i] == "mobile_base") 
        break;
    }
    measurement(0,0) = msg.pose[i].position.x;
    measurement(1,0) = msg.pose[i].position.y;
    measurement(2,0) = tf::getYaw(msg.pose[i].orientation);
    set_rotational_matrix();
    ROS_WARN("Measurements: X: [%f], Y: [%f], Yaw: [%f]", measurement(0,0), measurement(1,0), measurement(2,0));
}

visualization_msgs::MarkerArray build_rvizparticles() {
    visualization_msgs::MarkerArray markers;
    markers.markers = std::vector<visualization_msgs::Marker>(NUM_PARTICLES);
    for (int i = 0; i < NUM_PARTICLES; i++) {
       visualization_msgs::Marker marker;
        
       marker.header.frame_id = "base_link";
       marker.header.stamp = ros::Time();
       // marker.ns = "my_namespace";
       marker.id = i;
       marker.type = visualization_msgs::Marker::CUBE;
       marker.action = visualization_msgs::Marker::ADD;
       marker.pose.position.x = estimates[i](0,0);
       marker.pose.position.y = estimates[i](1,0);
       marker.pose.position.z = 0;
       // marker.pose.orientation.z = particle(2,0);
       marker.scale.x = 0.1;
       marker.scale.y = 0.1;
       marker.scale.z = 0.1;
       marker.color.a = 1.0; // Don't forget to set the alpha!
       marker.color.r = 1.0;
       marker.color.g = 0.0;
       marker.color.b = 0.0;
       marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

       markers.markers[i] = marker;
        //ROS_INFO("X: [%f], Y: [%f], Yaw: [%f]", particle(0,0), particle(1,0), particle(2,0));     
    }
    return markers;
}

visualization_msgs::Marker build_marker() {
    visualization_msgs::Marker marker;
       marker.header.frame_id = "base_link";
       marker.header.stamp = ros::Time();
       // marker.ns = "my_namespace";
       marker.id = 0;
       marker.type = visualization_msgs::Marker::CUBE;
       marker.action = visualization_msgs::Marker::ADD;
       marker.pose.position.x = measurement(0,0);
       marker.pose.position.y = measurement(1,0);
       marker.pose.position.z = 0;
       // marker.pose.orientation.z = measurement(2,0);
       marker.scale.x = 0.1;
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

//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

    ips_x X = msg.pose.pose.position.x; // Robot X psotition
    ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
    ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
    ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}*/

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    //This function is called when a new map is received
    
    //you probably want to save the map into a form which is easy to work with
}

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty 
//    vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;
    
    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}


int main(int argc, char **argv)
{
    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber odometry_sub = n.subscribe("/odom", 1, odometry_callback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    ros::Publisher particles_publisher = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );   

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(UPDATE_RATE);    //20Hz update rate
    
    variance(0,0) = 0.01;
    variance(0,1) = 0;
    variance(0,2) = 0;
    variance(1,0) = 0;
    variance(1,1) = 0.01;
    variance(1,2) = 0;
    variance(2,0) = 0;
    variance(2,1) = 0;
    variance(2,2) = 0.001;

    generate_initial_samples(NUM_PARTICLES);


    while (ros::ok())
    {
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages

        //Main loop code goes here:
        vel.linear.x = 0.1; // set linear speed
        vel.angular.z = 0.3; // set angular speed

        update_predicted();
        cum_sum_weights();
        resample();        

        velocity_publisher.publish(vel); // Publish the command velocity
        particles_publisher.publish(build_rvizparticles());
        vis_pub.publish(build_marker());

    }
    return 0;
}
