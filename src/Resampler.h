/*
 * Resampler.h
 *
 *  Created on: Dec 5, 2014
 *      Author: mafilipp
 */

#ifndef MAFILIPP_PARTICLE_FILTER_SRC_RESAMPLER_H_
#define MAFILIPP_PARTICLE_FILTER_SRC_RESAMPLER_H_

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

class Resampler {
public:
	Resampler(Particle * pc, int numPart, Map *map, double * cor);
	virtual ~Resampler();
	void resampleMap();
	void resampleUniversal();

	void debug();

private:
	Particle * particleCloud;
	int numberOfParticle;
	Map * mapPtr;
	double * correlation;
};

#endif /* MAFILIPP_PARTICLE_FILTER_SRC_RESAMPLER_H_ */
