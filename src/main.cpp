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
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <random>
#include <iostream>
#include <chrono>


#include "map.h"
#include "Model.h"


using namespace std;



//// Callback function for map messages
//void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
//{
//  ROS_INFO("I read a map!");
//}


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

	return poseArray;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	ROS_INFO("Laser");
}




//geometry_msgs::PoseStamped publishSinglePose(geometry_msgs::Pose posess)
//{
//	geometry_msgs::PoseStamped poseStamped;
//
//	poseStamped.header.frame_id = "map";
//	poseStamped.header.stamp = ros::Time();
//
//
//	geometry_msgs::Pose pose;
//
//	geometry_msgs::Quaternion quaternion;
//
//	quaternion = tf::createQuaternionMsgFromYaw(M_PI/2);
//
//	pose.position.x = 0.7;
//	pose.position.y = 0.7;
//	pose.position.z = 0.0;
//	pose.orientation = quaternion;
//
//
//	poseStamped.pose = pose;
//
//	return poseStamped;
//}

// ==================================================





//=====================================

// Main
int main(int argc, char **argv)
{
  // Initialize the node
  ros::init(argc, argv, "particle_filter");

  // Define variables
  double robotRadius = 0.08;
  Map gridMap(robotRadius);
  bool PathComputed = false;
  Model model;

  struct timeval tv;

  ros::NodeHandle n;

  // Set publisher and subscriber
//  ros::Subscriber mapSubscriber = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, mapCallback);
  ros::Subscriber mapSubscriberCallback = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, &Map::mapCallback, &gridMap);
  ros::Subscriber laserSubscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, laserCallback);
  ros::Subscriber odomSubscriber = n.subscribe<nav_msgs::Odometry>("/thymio_driver/odometry", 10, &Model::odomCallback, &model);

  ros::Publisher inflatedMapPublisher = n.advertise<nav_msgs::OccupancyGrid>("/inflatedMap",10);
  ros::Publisher particle_pose = n.advertise<geometry_msgs::PoseArray>( "/particle_pose", 0 );
  ros::Publisher single_particle_pose = n.advertise<geometry_msgs::PoseStamped>( "/single_particle_pose", 0 );


  ros::Rate loop_rate(1);

  // First wait until we get the map and is inflated
  while (!gridMap.isUpToDate() && !gridMap.isAlreadyInflated())
  {
    ros::spinOnce();

	// Inflate the map
	gridMap.inflate();
	// Send the inflated map through /inflatedMap
	inflatedMapPublisher.publish(gridMap.getMap());

    // Publish the path
    loop_rate.sleep();
  }




  while (ros::ok())
  {

	  ros::spinOnce();

	  // Publish
	  particle_pose.publish(publishParticleArray());

	  single_particle_pose.publish(model.publishSinglePose());

	  ROS_INFO("Up");



//	  loop_rate.sleep();


  }


  return 0;
}

// Get c++ version
//	  if( __cplusplus == 201103L ) std::cout << "C++11\n" ;
//	  else if( __cplusplus == 19971L ) std::cout << "C++98\n" ;
//	  else std::cout << "pre-standard C++\n" ;
