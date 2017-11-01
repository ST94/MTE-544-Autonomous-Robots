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

using namespace Eigen;
using namespace std;

const int NUM_PARTICLES = 144;
const int UPDATE_RATE = 2;
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
MatrixXf stdDev(3,3);
MatrixXf measurement(3,1);
MatrixXf velocity(3,1);

double weights [NUM_PARTICLES];

double pathId = 0;
double estimatesId = 10000;

short sgn(int x) { return x >= 0 ? 1 : -1; }

double randomGenerate()
{
	double random = (double)rand()/ RAND_MAX;
	return -1.0 + 2.0 * random;
}

void generate_initial_samples(int num_particles) 
{
    ROS_INFO("generating initial samples");
    int particle = 0;
    for (int i = 0; i < sqrt(NUM_PARTICLES); i++) {
    	for (int j = 0; j < sqrt(NUM_PARTICLES); j++){
            double random = (double)rand()/ RAND_MAX;
            MatrixXf estimate(3,1);
            estimate(0,0) = - 2.5 + (5.0/sqrt(NUM_PARTICLES)) * i;
            estimate(1,0) = - 2.5 + (5.0/sqrt(NUM_PARTICLES)) * j;
            estimate(2,0) = - PI + random*2*PI;
            estimates[particle] = estimate;
            particle++;
    	}
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
	//ROS_INFO("update_predicted begins");
    int particle;
    for(particle = 0;particle<NUM_PARTICLES;particle++) {
        MatrixXf updated_position(3,1);
        MatrixXf randomMatrix(3,1);
        randomMatrix(0,0) = randomGenerate();
        randomMatrix(1,0) = randomGenerate();
        randomMatrix(2,0) = randomGenerate();
        predicted[particle] = estimates[particle] + (1.0/UPDATE_RATE)*rotational_matrix*velocity + stdDev * randomMatrix;
        predicted[particle](2,0) = rad_wrap(predicted[particle](2,0));
        weights[particle] = normpdf(measurement, predicted[particle], variance);
        //ROS_INFO("Predictions: X: [%f], Y: [%f], Yaw: [%f]", predicted[particle](0,0), predicted[particle](1,0), predicted[particle](2,0));
        //ROS_INFO("probability: [%f]", weights[particle]);
    }
    // ROS_INFO("Estimate: X: [%f], Y: [%f], Yaw: [%f]", estimates[1](0,0), estimates[1](1,0), estimates[1](2,0));
    // ROS_INFO("Predictions: X: [%f], Y: [%f], Yaw: [%f]", predicted[1](0,0), predicted[1](1,0), predicted[1](2,0));
    // ROS_INFO("probability: [%f]", weights[1]);
    //ROS_INFO("update_predicted finish");
}

void cum_sum_weights(){
	//ROS_INFO("cum_sum begin");
    double max_weight;

    for(int i = 1; i < NUM_PARTICLES; i++) {
        weights[i] = weights[i]+weights[i-1];
    }

    max_weight = weights[(NUM_PARTICLES-1)];

    for(int i = 0; i<NUM_PARTICLES ; i++){
        weights[i] = weights[i] / max_weight;
    }
    //ROS_INFO("cum_sum finish");
}

void resample() {
    int i;
    for(i = 0; i<NUM_PARTICLES ; i++) {
        double random = randomGenerate();
        int j = 0;
        while (weights[j] < random && j < (NUM_PARTICLES-1))
        {
            j++;
        }
        //ROS_INFO("i:[%i], j:[%i]", i, j);
        //ROS_INFO("random: %i", j);
        estimates[i] = predicted[j];
        // ROS_INFO("Predictions: X: [%f], Y: [%f], Yaw: [%f]", estimates[i](0,0), estimates[i](1,0), estimates[i](2,0));
        //ROS_INFO("Weight: [%f]", weights[j]);
    }
    //ROS_INFO("resample");
}

void odometry_callback(const nav_msgs::Odometry msg) {
    velocity(0,0) = msg.twist.twist.linear.x + 0.1 * randomGenerate();
    velocity(1,0) = msg.twist.twist.linear.y + 0.1 * randomGenerate();
    velocity(2,0) = msg.twist.twist.angular.z + 0.1 * randomGenerate();
    // ROS_WARN("Velocity: X: [%f], Y: [%f], Yaw: [%f]", velocity(0,0), velocity(1,0), velocity(2,0));
};

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg) 
{
    int i;
    for(i = 0; i < msg.name.size(); i++) {
        if(msg.name[i] == "mobile_base") 
        break;
    }
    measurement(0,0) = msg.pose[i].position.x + 0.1 * randomGenerate();
    measurement(1,0) = msg.pose[i].position.y + 0.1 * randomGenerate();
    measurement(2,0) = tf::getYaw(msg.pose[i].orientation) + 0.1 *randomGenerate();
    set_rotational_matrix();
    // ROS_WARN("Measurements: X: [%f], Y: [%f], Yaw: [%f]", measurement(0,0), measurement(1,0), measurement(2,0));
}

visualization_msgs::Marker build_rvizparticles() {
    visualization_msgs::Marker marker;
     marker.header.frame_id = "base_link";
       marker.header.stamp = ros::Time();
       marker.id = estimatesId;
       estimatesId+=1;
       marker.type = visualization_msgs::Marker::POINTS;
       marker.action = visualization_msgs::Marker::ADD;
       // marker.pose.orientation.z = particle(2,0);
       marker.scale.x = 0.05;
       marker.scale.y = 0.05;
       marker.scale.z = 0.05;
       marker.color.a = 1.0; // Don't forget to set the alpha!
       marker.color.r = 1.0;
       marker.color.g = 0.0;
       marker.color.b = 0.0;
       marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

    for (int i = 0; i < NUM_PARTICLES; i++) {
       geometry_msgs::Point point;
       point.x = estimates[i](0,0);
       point.y = estimates[i](1,0);
       marker.points.push_back(point);
        //ROS_INFO("X: [%f], Y: [%f], Yaw: [%f]", particle(0,0), particle(1,0), particle(2,0));     
    }
    return marker;
}

visualization_msgs::Marker build_marker() {
    visualization_msgs::Marker marker;
       marker.header.frame_id = "base_link";
       marker.header.stamp = ros::Time();
       marker.id = pathId;
       //pathId+=1;
       marker.type = visualization_msgs::Marker::ARROW;
       marker.action = visualization_msgs::Marker::ADD;
       marker.pose.position.x = measurement(0,0);
       marker.pose.position.y = measurement(1,0);
       marker.pose.position.z = 0;
       marker.pose.orientation = tf::createQuaternionMsgFromYaw(measurement(2,0));
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
    ros::Publisher pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    ros::Publisher visual_publisher = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );   


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

    stdDev(0,0) = 0.1;
    stdDev(0,1) = 0;
    stdDev(0,2) = 0;
    stdDev(1,0) = 0;
    stdDev(1,1) = 0.1;
    stdDev(1,2) = 0;
    stdDev(2,0) = 0;
    stdDev(2,1) = 0;
    stdDev(2,2) = 0.1;

    generate_initial_samples(NUM_PARTICLES);


    while (ros::ok())
    {
        loop_rate.sleep(); //Maintain the loop rate

    	visual_publisher.publish(build_rvizparticles());
        visual_publisher.publish(build_marker());

        ros::spinOnce();   //Check for new messages
        update_predicted();
        cum_sum_weights();
        resample();     

        
    }
    return 0;
}
