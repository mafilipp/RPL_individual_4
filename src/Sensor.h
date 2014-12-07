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

	sensor_msgs::LaserScan::ConstPtr scanPtr;
	Particle * particleCloud;
	int numberOfParticle;
	Map * mapPtr;
	double * correlation;

	float rangeMin;
	float rangeMax;
	float angleMin;
	float angleMax;
	float angleIncrement;

	bool upToDate;


public:
	Sensor(Particle * pc, int numPart, Map *map, double * cor);
	virtual ~Sensor();

	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void sensorPrediction();
	bool isUpToDate() const;
	void setUpToDate(bool upToDate);
};

#endif /* MAFILIPP_PARTICLE_FILTER_SRC_SENSOR_H_ */
