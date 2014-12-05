/*
 * Sensor.cpp
 *
 *  Created on: Dec 5, 2014
 *      Author: mafilipp
 */

#include "Sensor.h"



Sensor::Sensor(Particle * pc, int nOp, Map Globalmap)
{
	particleCloud = pc;
	numberOfParticle = nOp;
	globalMap = &Globalmap;
}

Sensor::~Sensor() {
	// TODO Auto-generated destructor stub
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


