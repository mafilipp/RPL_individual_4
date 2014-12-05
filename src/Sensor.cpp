/*
 * Sensor.cpp
 *
 *  Created on: Dec 5, 2014
 *      Author: mafilipp
 */

#include "Sensor.h"



Sensor::Sensor(Particle * pc, int numPart, Map *map, double * cor)
{
	particleCloud = pc;
	numberOfParticle = numPart;
	mapPtr = map;
	correlation = cor;

	rangeMin = 0;
	rangeMax = 0;
	angleMin = 0;
	angleMax = 0;
	angleIncrement = 0;
}

Sensor::~Sensor() {
	// TODO Auto-generated destructor stub
}

void Sensor::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
//	ROS_INFO("Laser");
	scanPtr = msg;
	rangeMin = scanPtr->range_min;
	rangeMax = scanPtr->range_max;
	angleMin = scanPtr->angle_min;
	angleMax = scanPtr->angle_max;
	angleIncrement = scanPtr->angle_increment;
//	occupancyGridMapping();

}

void Sensor::sensorPrediction()
{
//	ROS_INFO("mapPtr->getResolution() = %f", mapPtr->getResolution()); -- > Corretta!!
	std::cout << "Start sensorPrediction" << std::endl;

	bool calculated;

	for (int i = 0; i < numberOfParticle; i++)
	{

//		std::cout << "Particle " << i << std::endl;
		int idx = 0;
		int count = 0;
		double total = 0;

		// Find the distance to the wall
		double x, y, theta, upX, upY;

		x = particleCloud[i].getX();
		y = particleCloud[i].getY();
//		ROS_INFO("particle %d: x = %f, y = %f", i, x, y);
		// Sembra ok
		theta = particleCloud[i].getTheta();

		ROS_INFO("HERE1");

//		if(    x > mapPtr->getColumn() * mapPtr->getResolution() ||
//			   x < 0 ||
//			   y > mapPtr->getRow() * mapPtr->getResolution() ||
//			   y < 0 ||
//			   mapPtr-> isOccupied(y,x))
		if( (x > mapPtr->getColumn() * mapPtr->getResolution()) || (x < 0) || (y > mapPtr->getRow() * mapPtr->getResolution() ) || (y < 0) || (mapPtr-> isOccupied(y,x) ) )
		{
			ROS_INFO("HERE");

			correlation[i] = 0;
		}
		else
		{
			ROS_INFO("HERE2");
			// Find possible Cell -- > Per un quache motivo non mi entra ne loop....
			for (double angle = angleMin; angle < angleMax; angle = angle + angleIncrement)
			{
				ROS_INFO("HERE3");
				ROS_INFO("angle -> %f", angle);
				ROS_INFO("index -> %d", idx);
				ROS_INFO("range -> %f", scanPtr->ranges[idx]);

				calculated = false;

				// Forse al contrario??
				upX = cos(angle + theta) * mapPtr->getResolution();
				upY = sin(angle + theta) * mapPtr->getResolution();

				ROS_INFO("x = %f, y = %f", x, y);
				for(double dist = 0; dist < 1; dist = dist + 0.01)
				{
					x = x + upX;
					y = y + upY;

					std::cout << "x = " << x << " y = " << y << std::endl;
					if(mapPtr->isOccupied(y,x))
					{
						ROS_INFO("3");

						if(scanPtr->ranges[idx] > rangeMin && scanPtr->ranges[idx] < rangeMax)
						{
							total = total + std::abs(dist - scanPtr->ranges[idx]);
							count = count + 1;
							calculated = true;
						}
						else
						{
							calculated = false;
						}
					}
				}
				if(!calculated)
				{
					total = total + std::abs(1 - scanPtr->ranges[idx]);
				}
				idx = idx + 1;
			}

			// Calculate Correlation

			// Store it in the correlation array
			correlation[i] = total;
		}
	}

//	for(int i = 0; i < numberOfParticle; i++)
//	{
//		if(i < 10)
//			correlation[i] = 10;
//		else
//			correlation[i] = 0;
//	}
}


