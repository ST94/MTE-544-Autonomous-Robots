//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

class Pose
{
	geometry_msgs::Twist Velocity_Command;

	double Square_Size;

	enum Color { red, green, blue };

	enum Travel_Direction
	 {
	 	right,
	 	up,
	 	left,
	 	down
	 }; 

	 Travel_Direction travel_direction;  

	public:
	Pose()
	{
		Square_Size = 0.8;
		travel_direction = right;
	}

	void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
	{

		//This function is called when a new position message is received

		double X_Position = msg->pose.pose.position.x; // Robot X position
		double Y_Position = msg->pose.pose.position.y; // Robot Y position
	 	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw
		
		switch(travel_direction)
		{
			
			case right:
				if (X_Position >= Square_Size && X_Position >= 0)
					{
						make_turn();
						if (Yaw >= 1.37 && Yaw <= 1.77)
						{
							travel_direction = up;
							go_straight();
						}
						break;
					} else
					{
					 	go_straight();
					 	break;
					}

			case up:
				if (Y_Position >= Square_Size)
					{
						make_turn();
						if (Yaw >= 2.94 || (Yaw >=-3.14 && Yaw <= -2.94))
						{
							travel_direction = left;
							go_straight();
						}
						break;
					} else
					{
					 	go_straight();
					 	break;
					}

			case left:
				if (X_Position <= -Square_Size)
					{
						make_turn();
						if (Yaw >= -1.77 && Yaw <= -1.37)
						{
							travel_direction = down;
							go_straight();
						}
						break;
					} else
					{
					 	go_straight();
					 	break;
					}

			case down:
				if (Y_Position <= -Square_Size)
					{
						make_turn();
						if (Yaw >= -0.2)
						{
							travel_direction = right;
							go_straight();
						}
						break;
					} else
					{
					 	go_straight();
					 	break;
					}
		}
		ROS_INFO("X: [%f], Y: [%f], Yaw: [%f], travel_direction : %i", X_Position, Y_Position, Yaw, travel_direction);
	}

	geometry_msgs::Twist getVel()
	{
		return Velocity_Command;
	}

	private:
	void make_turn()
	{
		ROS_INFO("make turn");
		Velocity_Command.linear.x = 0;
		Velocity_Command.angular.z = 0.1;
	}

	void go_straight()
	{
		ROS_INFO("go straight");
		Velocity_Command.linear.x = 0.2;
		Velocity_Command.angular.z = 0.0;
	}


};

int main(int argc, char **argv)
{
//Initialize the ROS framework
    ros::init(argc,argv,"square_control");
    ros::NodeHandle n;
    Pose pose_class;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, &Pose::pose_callback, &pose_class);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate    
    ros::Rate loop_rate(20);    //20Hz update rate

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
    
    	//Main loop code goes here:
    	vel = pose_class.getVel();
    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
