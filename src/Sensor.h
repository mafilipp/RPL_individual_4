/*
 * Sensor.h
 *
 *  Created on: Dec 5, 2014
 *      Author: mafilipp
 */

#ifndef MAFILIPP_PARTICLE_FILTER_SRC_SENSOR_H_
#define MAFILIPP_PARTICLE_FILTER_SRC_SENSOR_H_


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


class Sensor
{
private:
//	Map localMap;
//	nav_msgs::OccupancyGrid localMapMsg;

	sensor_msgs::LaserScan::ConstPtr scanPtr;
	Particle * particleCloud;
	Map * globalMap;
	int numberOfParticle;
	double * correlation;

	float rangeMin;
	float rangeMax;
	float angleMin;
	float angleMax;
	float angleIncrement;


public:
	Sensor(Particle * pc, int nOp, Map Globalmap);
	virtual ~Sensor();


	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void occupancyGridMapping();

	Map getMap();
	nav_msgs::OccupancyGrid getMapMsg();

	double * findCorrespondence();

};

#endif /* MAFILIPP_PARTICLE_FILTER_SRC_SENSOR_H_ */
