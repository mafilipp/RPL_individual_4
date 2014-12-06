/*
 * Model.cpp
 *
 *  Created on: Dec 3, 2014
 *      Author: mafilipp
 */

#include "Model.h"


Model::Model(Particle * pc, int nOp)
{
	// Initialize all the parameter
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

	// Connect the model with the particle cloud
	particleCloud = pc;
	numberOfParticle = nOp;

//	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
//	generator.seed(seed);
}

Model::~Model() {
	// TODO Auto-generated destructor stub
}

//geometry_msgs::PoseStamped Model::publishSinglePose()
//{
//	geometry_msgs::PoseStamped poseStamped;
//
//	poseStamped.header.frame_id = "odometry_link";
//	poseStamped.header.stamp = ros::Time();
//
//	poseStamped.pose = pose;
//
//	return poseStamped;
//}

void Model::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
//	ROS_INFO("odom");

	// Introduce flag to avoid dreadlock
//	if( ! usingOdomData)
//	{
	overwritingOdometry = true;


	x_odom = msg->pose.pose.position.x;
	y_odom = msg->pose.pose.position.y;
	theta_odom = tf::getYaw(msg->pose.pose.orientation);

	isUpToDate = true;
	overwritingOdometry = false;

//	modelPrediction();
//	}


	// Model update

	// Sensor Update

	// General Update
//	setModelUpdatedPose();

}

void Model::modelPrediction()
{
//	if(!overwritingOdometry && isUpToDate)
//	{
//		usingOdomData = true;

	alpha1 = 0.0;
	alpha2 = 0.0;
	alpha3 = 0.0;
	alpha4 = 0.0;

//	ROS_INFO("%f	%f		%f", sample(alpha1 * pow(dRot1, 2) + alpha2 * pow(dTrans, 2)), sample(alpha3 * pow(dTrans, 2) + alpha4 * pow(dRot1, 2) + alpha4 * pow(dRot2, 2) ), sample(alpha1 * pow(dRot2, 2) + alpha2 * pow(dTrans, 2) ));
	dRot1 = atan2(y_odom - y_odom_old, x_odom - x_odom_old) - theta_odom_old;
	dTrans = sqrt( pow((x_odom_old - x_odom), 2) + pow((y_odom_old - y_odom), 2) );
	dRot2 = theta_odom - theta_odom_old - dRot1;

	dRot1_hat = dRot1   - sample(alpha1 * pow(dRot1, 2) + alpha2 * pow(dTrans, 2));
	dTrans_hat = dTrans - sample(alpha3 * pow(dTrans, 2) + alpha4 * pow(dRot1, 2) + alpha4 * pow(dRot2, 2) );
	dRot2_hat = dRot2 - sample(alpha1 * pow(dRot2, 2) + alpha2 * pow(dTrans, 2) );

	for(int i = 0; i < numberOfParticle; i++)
	{

		x_old = particleCloud[i].getX();
		y_old = particleCloud[i].getY();
		theta_old = particleCloud[i].getTheta();

		x = x_old + dTrans_hat * cos( theta_old + dRot1_hat);
		y = y_old + dTrans_hat * sin( theta_old + dRot1_hat);
		theta = theta_old + dRot1_hat + dRot2_hat;

		particleCloud[i].setX(x);
		particleCloud[i].setY(y);
		particleCloud[i].setTheta(theta);

//		pose.position.x = x;
//		pose.position.y = y;
//		pose.position.z = 0;
//		pose.orientation = tf::createQuaternionMsgFromYaw(theta);

//	}
		usingOdomData = false;
		isUpToDate = false;
//		return pose;


		x_odom_old = x_odom;
		y_odom_old = y_odom;
		theta_odom_old = theta_odom;
	}

}

// For one particle
//void Model::modelPrediction()
//{
////	if(!overwritingOdometry && isUpToDate)
////	{
////		usingOdomData = true;
//
////	for(int i = 0; i < numberOfParticle; i++)
////	{
//
//		// TODO: scrivi le cose in maniera che l'update avvenga per ogni particella
//
//	alpha1 = 0.01;
//	alpha2 = 0.02;
//	alpha3 = 0.01;
//	alpha4 = 0.01;
//
//	ROS_INFO("%f	%f		%f", sample(alpha1 * pow(dRot1, 2) + alpha2 * pow(dTrans, 2)), sample(alpha3 * pow(dTrans, 2) + alpha4 * pow(dRot1, 2) + alpha4 * pow(dRot2, 2) ), sample(alpha1 * pow(dRot2, 2) + alpha2 * pow(dTrans, 2) ));
//		dRot1 = atan2(y_odom - y_odom_old, x_odom - x_odom_old) - theta_odom_old;
//		dTrans = sqrt( pow((x_odom_old - x_odom), 2) + pow((y_odom_old - y_odom), 2) );
//		dRot2 = theta_odom - theta_odom_old - dRot1;
//
//		dRot1_hat = dRot1   - sample(alpha1 * pow(dRot1, 2) + alpha2 * pow(dTrans, 2));
//		dTrans_hat = dTrans - sample(alpha3 * pow(dTrans, 2) + alpha4 * pow(dRot1, 2) + alpha4 * pow(dRot2, 2) );
//		dRot2_hat = dRot2   - sample(alpha1 * pow(dRot2, 2) + alpha2 * pow(dTrans, 2) );
//
//		x = x_old + dTrans_hat * cos( theta_old + dRot1_hat);
//		y = y_old + dTrans_hat * sin( theta_old + dRot1_hat);
//		theta = theta_old + dRot1_hat + dRot2_hat;
//
//
//
//		pose.position.x = x;
//		pose.position.y = y;
//		pose.position.z = 0;
//		pose.orientation = tf::createQuaternionMsgFromYaw(theta);
//
////	}
//		usingOdomData = false;
//		isUpToDate = false;
////		return pose;
//
////	}
//
//}

//void Model::setModelUpdatedPose()
//{
//	x_old = pose.position.x;
//	y_old = pose.position.y;
//	theta_old = tf::getYaw(pose.orientation);
//}

double Model::sample(double variance)
{
//	std::normal_distribution<double> distribution(0, sqrt(variance));
//
//	return distribution(generator);
//	return ( rand()/RAND_MAX * sqrt(variance) *sqrt(6));
//	double sum;
//	double sDeviation = sqrt(variance);
//	for (int i = 0; i < 12; i ++)
//	{
//		sum = sum + (2 * sDeviation * rand()/RAND_MAX - sDeviation);
//	}
//	return sum;

	return sqrt(variance) * (2* rand()/RAND_MAX -1) * (2* rand()/RAND_MAX -1);
}


//geometry_msgs::PoseArray Model::publishParticleArray()
//{
//	geometry_msgs::PoseArray poseArray;
//	geometry_msgs::Pose pose;
//
//	poseArray.header.frame_id = "odometry_link";
//	poseArray.header.stamp = ros::Time();
////	poseArray.header.seq = 1;  --> A cosa serve?
//
//	// For all particle, plot the pose
//	for (int i = 0; i < numberOfParticle; i ++)
//	{
//		pose.position.x = particleCloud[i].getX();
//		pose.position.y = particleCloud[i].getY();
//		pose.position.z = 0.0;
//		pose.orientation = tf::createQuaternionMsgFromYaw(particleCloud[i].getTheta());
//
//		poseArray.poses.push_back(pose);
//	}
//
//	return poseArray;
//}
