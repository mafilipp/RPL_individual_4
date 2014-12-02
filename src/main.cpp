/*
 * main.pp
 *
 *  Created on: Oct 22, 2014
 *      Author: Filippo Martinoni
 *      Note: The core of the node path_planning
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

#include <math.h>

#include "map.h"


using namespace std;



// Callback function for map messages
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("I read a map!");
}



geometry_msgs::PoseArray publishParticleArray()
{
	geometry_msgs::PoseArray poseArray;
	geometry_msgs::Pose pose;
	geometry_msgs::Quaternion quaternion;

	poseArray.header.frame_id = "map";
	poseArray.header.stamp = ros::Time();
//	poseArray.header.seq = 1;  --> A cosa serve?

	// For all particle, plot the pose
	for (int i = 0; i < 10; i ++)
	{
		quaternion = tf::createQuaternionMsgFromYaw(M_PI/2);

		pose.position.x = 0.2;
		pose.position.y = 0.0;
		pose.position.z = 0.0;
		pose.orientation = quaternion;

		poseArray.poses.push_back(pose);
	}


//	poseArray.poses.push_back(pose);

	return poseArray;
}

// Per una sola particle
//geometry_msgs::PoseStamped publishParticleArray()
//{
////	geometry_msgs::PoseArray poseArray;
//	geometry_msgs::PoseStamped pose;
//
//	pose.header.frame_id = "map";
//	pose.header.stamp = ros::Time();
//
//	pose.header.seq = 1;
//	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(M_PI/2);
//
//	pose.pose.position.x = 0.2;
//	pose.pose.position.y = 0.1;
//	pose.pose.position.z = 0.0;
//	pose.pose.orientation = odom_quat;
//
//
////	poseArray.poses.push_back(pose);
//
//	return pose;
//}


// Main
int main(int argc, char **argv)
{
  // Initialize the node
  ros::init(argc, argv, "particle_filter");

  // Define variables
  double robotRadius = 0.08;
  Map gridMap(robotRadius);
  bool PathComputed = false;

  ros::NodeHandle n;

  // Set publisher and subscriber
  ros::Subscriber mapSubscriber = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, mapCallback);
  ros::Subscriber mapSubscriberCallback = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, &Map::mapCallback, &gridMap);

  ros::Publisher inflatedMapPublisher = n.advertise<nav_msgs::OccupancyGrid>("/inflatedMap",10);
  ros::Publisher particle_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  ros::Publisher particle_array_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
//  ros::Publisher particle_pose = n.advertise<geometry_msgs::PoseStamped>( "/particle_pose", 0 );
  ros::Publisher particle_pose = n.advertise<geometry_msgs::PoseArray>( "/particle_pose", 0 );


  ros::Rate loop_rate(1);

  // First wait until we get the map and is inflated
  while (!gridMap.isUpToDate() && !gridMap.isAlreadyInflated())
  {
    ros::spinOnce();

	// Inflate the map
	gridMap.inflate();
	// Send the inflated map through /inflatedMap
	inflatedMapPublisher.publish(gridMap.getMap());

//    // Publish the path
    loop_rate.sleep();
  }


  while (ros::ok())
  {

	  ROS_INFO("Noi siamo qua");
	  ROS_INFO("resolution %f", gridMap.getResolution());
	  loop_rate.sleep();



	  particle_pose.publish(publishParticleArray());
//	  particle_pose.publish( particle_pose() );

  }


  return 0;
}
