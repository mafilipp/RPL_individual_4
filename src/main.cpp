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


#include <sys/time.h>



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

class Model
{
public:
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	double sample(double variance);
	void modelPrediction();
	void setModelUpdatedPose();
	Model();
	geometry_msgs::PoseStamped publishSinglePose();



private:
	geometry_msgs::Pose pose;

	// Used by callback
	double x_odom, x_odom_old, y_odom, y_odom_old, theta_odom, theta_odom_old;
	double dRot1, dTrans, dRot2;
	double dRot1_hat, dTrans_hat, dRot2_hat;
	double x, x_old, y, y_old, theta, theta_old;
	double alpha1, alpha2, alpha3, alpha4;

	bool overwritingOdometry;
	bool usingOdomData;
	bool isUpToDate;

	//
};

Model::Model()
{
	x_odom = 0;
	x_odom_old = 0;
	y_odom = 0;
	y_odom_old = 0;
	theta_odom = 0;
	theta_odom_old = 0;

	dRot1  = 0;
	dTrans = 0;
	dRot2 = 0;

	dRot1_hat = 0;
	dTrans_hat = 0;
	dRot2_hat = 0;

	x = 0;
	x_old = 0;
	y = 0;
	y_old = 0;
	theta = 0;
	theta_old = 0;

	alpha1 = 0;
	alpha2 = 0;
	alpha3 = 0;
	alpha4 = 0;

	overwritingOdometry = false;
	usingOdomData = false;
	isUpToDate = false;
}

geometry_msgs::PoseStamped Model::publishSinglePose()
{
	geometry_msgs::PoseStamped poseStamped;

	poseStamped.header.frame_id = "odometry_link";
	poseStamped.header.stamp = ros::Time();

	poseStamped.pose = pose;

	return poseStamped;
}

void Model::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO("odom");

	// Introduce flag to avoid dreadlock
//	if( ! usingOdomData)
//	{
		overwritingOdometry = true;

		x_odom_old = x_odom;
		y_odom_old = y_odom;
		theta_odom_old = theta_odom;

		x_odom = msg->pose.pose.position.x;
		y_odom = msg->pose.pose.position.y;
		theta_odom = tf::getYaw(msg->pose.pose.orientation);

		isUpToDate = true;
		overwritingOdometry = false;

//	}


		  // Model update
		  modelPrediction();

		  // Sensor Update

		  // General Update
		  setModelUpdatedPose();

}

void Model::modelPrediction()
{
//	if(!overwritingOdometry && isUpToDate)
//	{
//		usingOdomData = true;

		dRot1 = atan2(y_odom - y_odom_old, x_odom - x_odom_old) - theta_odom_old;
		dTrans = sqrt( pow((x_odom_old - x_odom), 2) + pow((y_odom_old - y_odom), 2) );
		dRot2 = theta_odom - theta_odom_old - dRot1;

		dRot1_hat = dRot1;// - sample(alpha1 * pow(dRot1, 2) + alpha2 * pow(dTrans, 2));
		dTrans_hat = dTrans;// - sample(alpha3 * pow(dTrans, 2) + alpha4 * pow(dRot1, 2) + alpha4 * pow(dRot2, 2) );
		dRot2_hat = dRot2;// - sample(alpha1 * pow(dRot2, 2) + alpha2 * pow(dTrans, 2) );

		x = x_old + dTrans_hat * cos( theta_old + dRot1_hat);
		y = y_old + dTrans_hat * sin( theta_old + dRot1_hat);
		theta = theta_old + dRot1_hat + dRot2_hat;



		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = 0;
		pose.orientation = tf::createQuaternionMsgFromYaw(theta);


		usingOdomData = false;
		isUpToDate = false;
//		return pose;

//	}

}

void Model::setModelUpdatedPose()
{
	x_old = pose.position.x;
	y_old = pose.position.y;
	theta_old = tf::getYaw(pose.orientation);
}

double Model::sample(double variance)
{
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator (seed);
	std::normal_distribution<double> distribution(0, sqrt(variance));

	return distribution(generator);
}


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
  ros::Subscriber mapSubscriber = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, mapCallback);
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
