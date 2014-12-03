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

	return poseArray;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	ROS_INFO("Laser");
}

double sample(double variance)
{
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0,sqrt(variance));
	return distribution(generator);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO("odom");

//	calculateX_m(msg);

	double dRot1, dTrans, dRot2;
	double dRot1_hat, dTrans_hat, dRot2_hat;
	double x_odom, x_odom_old, y_odom, y_odom_old, theta_odom, theta_odom_old;
	double x, x_old, y, y_old, theta, theta_old;
	double alpha1, alpha2, alpha3, alpha4;

	geometry_msgs::Pose pose;

	dRot1 = atan2(y_odom - y_odom_old, x_odom - x_odom_old) - theta_odom_old;
	dTrans = sqrt( pow((x_odom_old - x_odom), 2) + pow((y_odom_old - y_odom), 2) );
	dRot2 = theta - theta_odom_old - dRot1;
	dRot1_hat = dRot1 - sample(alpha1 * pow(dRot1, 2) + alpha2 * pow(dTrans, 2));
	dTrans_hat = dTrans - sample(alpha3 * pow(dTrans, 2) + alpha4 * pow(dRot1, 2) + alpha4 * pow(dRot2, 2) );
	dRot2_hat = dRot2 - sample(alpha1 * pow(dRot2, 2) + alpha2 * pow(dTrans, 2) );

	x = x_old + dTrans_hat*cos( theta + dRot1_hat);
	y = y_old + dTrans_hat * sin( theta + dRot1_hat);
	theta = theta_old + dRot1_hat + dRot2_hat;

	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = 0;
	pose.orientation = tf::createQuaternionMsgFromYaw(theta);

}

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
  ros::Subscriber laserSubscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, laserCallback);
  ros::Subscriber odomSubscriber = n.subscribe<nav_msgs::Odometry>("/thymio_driver/odometry", 10, odomCallback);
//  Type:
//
//  Publishers:
//   * /play_1417598832845931919 (http://mafilipp-MacBook:43680/)
//
//  Subscribers: None
//
//
//  mafilipp@mafilipp-MacBook:~$ rostopic info /thymio_driver/odometry
//  Type:


  ros::Publisher inflatedMapPublisher = n.advertise<nav_msgs::OccupancyGrid>("/inflatedMap",10);
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

    // Publish the path
    loop_rate.sleep();
  }


  while (ros::ok())
  {

	  ros::spinOnce();

	  ROS_INFO("loop");


	  particle_pose.publish(publishParticleArray());
	  loop_rate.sleep();

  }


  return 0;
}
