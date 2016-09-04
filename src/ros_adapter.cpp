#include "ros_adapter.h"

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

static const char * ROS_NODE_NAME = "navigator";
static const char * ROS_TOPIC_NAME = "cmd_vel_mux/input/navi";

static ros::Publisher g_publisher;

void ros_adapter::init(int argc, char * argv[])
{
	ros::init(argc, argv, ROS_NODE_NAME);
	ros::start();
	
	ros::NodeHandle g_node_handle;
	g_publisher = g_node_handle.advertise<geometry_msgs::Twist>(ROS_TOPIC_NAME, 1000);
}

void ros_adapter::update(float linear, float angular) 
{
	geometry_msgs::Twist cmd_vel_msg;
	cmd_vel_msg.linear.x = linear;
	cmd_vel_msg.angular.z = angular;
	g_publisher.publish(cmd_vel_msg);
}

void ros_adapter::move_back(float t, float linear, float angular)
{
	if (!t) return;
	ros::Rate back_rate(1.0 / t);
	update(linear, angular);
	back_rate.sleep();
}

void ros_adapter::shutdown()
{
	ros::shutdown();
}
