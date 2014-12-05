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


using namespace std;




class Sensor
{
private:
//	Map localMap;
//	nav_msgs::OccupancyGrid localMapMsg;

	sensor_msgs::LaserScan::ConstPtr scanPtr;
	Particle * particleCloud;
	Map * globalMap;
	int numberOfParticle;

	float rangeMin;
	float rangeMax;
	float angleMin;
	float angleMax;
	float angleIncrement;


public:
	Sensor(Particle * pc, int nOp, Map Globalmap);
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void occupancyGridMapping();

	Map getMap();
	nav_msgs::OccupancyGrid getMapMsg();

	double * findCorrespondence();
	void resample(double * correlation);

};

Sensor::Sensor(Particle * pc, int nOp, Map Globalmap)
{
	particleCloud = pc;
	numberOfParticle = nOp;
	globalMap = &Globalmap;
}

void Sensor::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	ROS_INFO("Laser");
	scanPtr = msg;
	rangeMin = scanPtr->range_min;
	rangeMax = scanPtr->range_max;
	angleMin = scanPtr->angle_min;
	angleMax = scanPtr->angle_max;
	angleIncrement = scanPtr->angle_increment;
//	occupancyGridMapping();

}

double * Sensor::findCorrespondence()
{
	double correlation[numberOfParticle];

	for (int i = 0; i < numberOfParticle; i++)
	{
		int idx = 0;
		int count = 0;
		double total = 0;

		// Find possible Cell
		for (double angle = angleMin; angle < angleMax; angle = angle + angleIncrement)
		{
			// Find the distance to the wall
			double x, y, theta, upX, upY;

			x = particleCloud[i].getX();
			y = particleCloud[i].getY();
			theta = particleCloud[i].getTheta();

			upX = cos(angle + theta)*globalMap->getResolution();
			upY = sin(angle + theta)*globalMap->getResolution();


			for(double dist = 0; dist < 1; dist = dist + 0.01)
			{
				x = x + upX;
				y = y + upY;

				if(globalMap->isOccupied(x,y))
				{
					count = count + 1;
					if(scanPtr->ranges[idx] < rangeMax)
					{
						total = total + std::abs(dist - scanPtr->ranges[idx]);

					}
					continue;
				}
			}

			idx = idx + 1;
		}

		// Calculate Correlation

		// Store it in the correlation array
		correlation[i] = total;
	}
}

void Sensor::resample(double * correlation)
{
	// Find the total
	double sum = 0;
	double random;

	// Create a new particle vector
	Particle * resampledParticle = new Particle[numberOfParticle];


	for(int i = 0; i < numberOfParticle; i++)
	{
		sum = sum + correlation[i];
	}

	// Normalize the correlation vector
	for(int i = 0; i < numberOfParticle; i++)
	{
		correlation[i] = correlation[i] / sum;
	}

	double beta[numberOfParticle];
	sum = 0;
	// Calculate the vector for resampling
	for(int i = 0; i < numberOfParticle; i++)
	{
		sum = sum + correlation[i];
		beta[i] = sum;
	}

	// Choose the weighted particle
	for(int i = 0; i < numberOfParticle; i++)
	{
		random = rand()/RAND_MAX;
		for(int j = 0; j < numberOfParticle; j++)
		{
			if(random < correlation[j])
				resampledParticle[i] = particleCloud[j];
		}
	}

	particleCloud =  resampledParticle;

}

//
//Map Sensor::getMap()
//{
//	return localMap;
//}
//
//nav_msgs::OccupancyGrid Sensor::getMapMsg()
//{
//	return localMapMsg;
//}
//
//bool isInPerceptualField(int cell)
//{
//
//}
//
//void findPossibleCell
//{
//	for(int i = 0; i < NumberOfParticle; i++)
//	{
//
//	}
//}
//
//void Sensor::occupancyGridMapping()
//{
////	localMap
//
////	localMap.setColumn(200);
////	localMap.setRow(200);
//
//	localMapMsg.info.height = 200;
//	localMapMsg.info.width = 200;
////	localMapMsg.info.origin = 50;
//	localMapMsg.info.resolution = 0.01;
//
//
//	for(int i = 0; i < 200*200; i++)
//	{
//		localMapMsg.data.push_back(0);
////		if isInPerceptualField(localMap.getData[i])
////		{
////			//
////		}
//	}
//
//	int idx = 0;
//
//
//	float angle, range;
//
//	for(angle = angleMin; angle < angleMax; angle = angle + angleIncrement)
//	{
//		range = scanPtr->ranges[idx];
//
//		// Check if we are inside the thresholds
//
//		if(range > rangeMin && range < rangeMax)
//		{
////			std::cout << range << std::endl;
//		}
//
//		// Increase index
//		idx = idx + 1;
//	}
//}


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


int xGrid = 200;
int yGrid = 200;
int thetaGrid = 90;

int totalNumberElement = xGrid * yGrid * thetaGrid;

// x = column, y = row, theta = third dimension
int getIndex(int x, int y, int theta)
{
	return x + y * xGrid + theta * (xGrid * yGrid);
}

double deg2Rad(double grad)
{
	return grad/180*M_PI;
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



  // Particles
  int numberParticle = 100;

  Particle * particleCloud = new Particle[numberParticle];


  Sensor sensor(particleCloud, numberParticle, gridMap);

  // Initialize Particle Cloud
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator (seed);
  std::uniform_real_distribution<double> distribution(0.0,1.0);
  double number;

  for (int i = 0; i < numberParticle; i++)
  {

	  number = distribution(generator);
	  particleCloud[i].setX(number*2);

	  number = distribution(generator);
	  particleCloud[i].setY(number*2);

	  number = distribution(generator);
	  particleCloud[i].setTheta( deg2Rad(number*90) );
  }

  // Model
  Model model(particleCloud, numberParticle);

  ros::NodeHandle n;

  // Set publisher and subscriber
//  ros::Subscriber mapSubscriber = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, mapCallback);
  ros::Subscriber mapSubscriberCallback = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, &Map::mapCallback, &gridMap);
  ros::Subscriber laserSubscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &Sensor::laserCallback, &sensor);
  ros::Subscriber odomSubscriber = n.subscribe<nav_msgs::Odometry>("/thymio_driver/odometry", 10, &Model::odomCallback, &model);

  ros::Publisher inflatedMapPublisher = n.advertise<nav_msgs::OccupancyGrid>("/inflatedMap",10);
  ros::Publisher perceptualFieldPublisher = n.advertise<nav_msgs::OccupancyGrid>("/perceptual_field",10);

  ros::Publisher particle_pose = n.advertise<geometry_msgs::PoseArray>( "/particle_pose", 0 );
  ros::Publisher single_particle_pose = n.advertise<geometry_msgs::PoseStamped>( "/single_particle_pose", 0 );



  // First wait until we get the map and is inflated
  while (!gridMap.isUpToDate() && !gridMap.isAlreadyInflated())
  {
    ros::spinOnce();

	// Inflate the map
	//gridMap.inflate();
	// Send the inflated map through /inflatedMap
	//inflatedMapPublisher.publish(gridMap.getMap());

    // Publish the path
//    loop_rate.sleep();
  }



  ros::Rate loop_rate(10);


  while (ros::ok())
  {

	  ros::spinOnce();


	  // Publish

	  model.modelPrediction();


	  single_particle_pose.publish(model.publishSinglePose());

	  particle_pose.publish(model.publishParticleArray());


//	  perceptualFieldPublisher.publish(sensor.getMap().getMap());

//	  perceptualFieldPublisher.publish(sensor.getMapMsg());

//	  loop_rate.sleep();


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
//	  std::cout << "cooupied	" << gridMap.isOccupied(97,120) << std::endl;

//	  gridMap.isOccupied(96, 35)
  }


  return 0;
}

//	  particle_pose.publish(publishParticleArray());


// Get c++ version
//	  if( __cplusplus == 201103L ) std::cout << "C++11\n" ;
//	  else if( __cplusplus == 19971L ) std::cout << "C++98\n" ;
//	  else std::cout << "pre-standard C++\n" ;
