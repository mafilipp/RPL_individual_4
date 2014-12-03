/*
 * Model.h
 *
 *  Created on: Dec 3, 2014
 *      Author: mafilipp
 */

#ifndef MAFILIPP_PARTICLE_FILTER_SRC_MODEL_H_
#define MAFILIPP_PARTICLE_FILTER_SRC_MODEL_H_

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


class Model
{
public:
	Model();
	virtual ~Model();
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	double sample(double variance);
	void modelPrediction();
	void setModelUpdatedPose();
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
#endif /* MAFILIPP_PARTICLE_FILTER_SRC_MODEL_H_ */
