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
#include "Particle.h"
#include "Sensor.h"
#include "Resampler.h"


using namespace std;


double deg2Rad(double grad)
{
	return grad/180*M_PI;
}

geometry_msgs::PoseArray publishParticleArray(Particle * particleCloud, int numberOfParticle)
{
	geometry_msgs::PoseArray poseArray;
	geometry_msgs::Pose pose;

	poseArray.header.frame_id = "odometry_link";
	poseArray.header.stamp = ros::Time();
//	poseArray.header.seq = 1;  --> A cosa serve?

	// For all particle, draw in Rviz the pose
	for (int i = 0; i < numberOfParticle; i ++)
	{
		pose.position.x = particleCloud[i].getX();
		pose.position.y = particleCloud[i].getY();
		pose.position.z = 0.0;
		pose.orientation = tf::createQuaternionMsgFromYaw(particleCloud[i].getTheta());

		poseArray.poses.push_back(pose);
	}

	return poseArray;
}

// Main
int main(int argc, char **argv)
{
  //** Initialize the node
  ros::init(argc, argv, "particle_filter");

  //** Define variables
  // Map
  double robotRadius = 0.08;
  Map gridMap(robotRadius);

  // Particles
  int numberParticle = 4;
  Particle * particleCloud = new Particle[numberParticle];

  // Resampler --> Perché qui il resampler non funziona???
  // Correlation
  double * correlation = new double[numberParticle];

  // Model
  Model model(particleCloud, numberParticle);

  // Sensor
  Sensor sensor(particleCloud, numberParticle, &gridMap, correlation);

  // Resampler
  Resampler resample(particleCloud, numberParticle, &gridMap, correlation);

  //** Initialize node
  ros::NodeHandle n;

  // Set publisher and subscriber
  ros::Subscriber mapSubscriberCallback = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, &Map::mapCallback, &gridMap);
  ros::Subscriber laserSubscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &Sensor::laserCallback, &sensor);
  ros::Subscriber odomSubscriber = n.subscribe<nav_msgs::Odometry>("/thymio_driver/odometry", 10, &Model::odomCallback, &model);

  ros::Publisher particle_pose = n.advertise<geometry_msgs::PoseArray>( "/particle_pose", 0 );
//  ros::Publisher single_particle_pose = n.advertise<geometry_msgs::PoseStamped>( "/single_particle_pose", 0);

  //** Initialize Element

  // First wait until we get the map

  while ( !(gridMap.isUpToDate() && sensor.isUpToDate()) )
  {
    ros::spinOnce();
  }

  // Initialize the particle with ramdom posend
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator (seed);
  std::uniform_real_distribution<double> distribution(0.0,1.0);

  double number;

//std::cout << gridMap.getRow() * gridMap.getResolution() << std::endl;

  double smallx = 0;
  double smally = 0;

  for (int i = 0; i < numberParticle; i++)
  {
	  // TODO: controll that column correspond to x..
	  number = distribution(generator);
	  particleCloud[i].setX(gridMap.getColumn() * gridMap.getResolution() * number);

	  number = distribution(generator);
	  particleCloud[i].setY(gridMap.getRow() * gridMap.getResolution() * number);

	  number = distribution(generator);
	  particleCloud[i].setTheta( deg2Rad(number*360) );

//	  particleCloud[i].setX(smallx+ 0.005);
//	  particleCloud[i].setY(smally+ 0.005);
//	  particleCloud[i].setTheta(0);
//	  smallx = smallx + 0.01;
//	  if(smallx > 2)
//	  {
//		  smallx = 0;
//		  smally = smally + 0.01;
//	  }
  }

  particleCloud[0].setX(0);
  particleCloud[0].setY(0);
  particleCloud[0].setTheta(0);

  particleCloud[1].setX(0.6);
  particleCloud[1].setY(0.7);
  particleCloud[1].setTheta(deg2Rad(-60));

  particleCloud[2].setX(1.9);
  particleCloud[2].setY(0.3);
  particleCloud[2].setTheta(deg2Rad(90));

  particleCloud[3].setX(1.9);
  particleCloud[3].setY(0.1);
  particleCloud[3].setTheta(deg2Rad(90));
  //** Start the algorithm

  // The first time, since we don't want to consider particle that are inside the wall
//  std::cout << "bef resample" << std::endl;
//  resample.resampleMap();
//  std::cout << "aft resample" << std::endl;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

	  // --> Controlla sensor update: è l'unica cosa che non funziona....
	  // --> Controlla anche bene il senso di rotazione del sensore!!
	  // ora cambiato nel for loop
	  ros::spinOnce();

//	  particle_pose.publish(publishParticleArray(particleCloud, numberParticle));

	  //** Make the calculation
//	  model.modelPrediction();

	  particle_pose.publish(publishParticleArray(particleCloud, numberParticle));


//	  ROS_INFO("After model prediction");
//	  for(int i = 0; i < numberParticle; i++)
//	  {
//		  ROS_INFO("particle Main %d: x = %f, y = %f", i, particleCloud[i].getX(), particleCloud[i].getY());
//	  }

ROS_INFO(" Before prediction 2");
	  sensor.sensorPrediction2();
////
		for(int i = 0; i < numberParticle; i++)
		{
			ROS_INFO("main correlation %d = %f", i, correlation[i]);
		}
////
////
//	  resample.resampleUniversal();
//	  resample.resampleMap();
//
//	  particle_pose.publish(publishParticleArray(particleCloud, numberParticle));


	  //** Publish

//		std::cout << "All particle" << std::cout;
//		for(int i = 0; i < numberParticle; i++)
//		{
//			std::cout << "x = " << particleCloud[i].getX() << "  y = " << particleCloud[i].getY() << std::endl;
//
//		}



	  loop_rate.sleep();

  }


  return 0;
}

//	  single_particle_pose.publish(model.publishSinglePose());

//	  particle_pose.publish(publishParticleArray());


// Get c++ version
//	  if( __cplusplus == 201103L ) std::cout << "C++11\n" ;
//	  else if( __cplusplus == 19971L ) std::cout << "C++98\n" ;
//	  else std::cout << "pre-standard C++\n" ;


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


//	  int cc = gridMap.getIndex (110,185);
//
//	  std::cout << "beginn" << std::endl;
//
//	  std::cout << "cooupied	" << gridMap.isOccupied(5,96) << std::endl;
//	  std::cout << "cooupied	" << gridMap.isOccupied(35,96) << std::endl;
//
//	  std::cout << "cooupied	" << gridMap.isOccupied(110,175) << std::endl;
//	  std::cout << "cooupied	" << gridMap.isOccupied(110,185) << std::endl;
//	  std::cout << "cooupied	" << gridMap.isOccupied(97,85) << std::endl;
//	  std::cout << "cooupied	" << gridMap.isOccupied(97,35) << std::endl;
//	  std::cout << "cooupied	" << gridMap.isOccupied(97,120) << std::endl; //



// ==================================================

//
//int xGrid = 200;
//int yGrid = 200;
//int thetaGrid = 90;
//
//int totalNumberElement = xGrid * yGrid * thetaGrid;
//
//// x = column, y = row, theta = third dimension
//int getIndex(int x, int y, int theta)
//{
//	return x + y * xGrid + theta * (xGrid * yGrid);
//}
//



//=====================================
