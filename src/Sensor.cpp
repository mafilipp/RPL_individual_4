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

	upToDate = false;
}

Sensor::~Sensor() {
	// TODO Auto-generated destructor stub
}

void Sensor::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
//	ROS_INFO("Laser");
	scanPtr = msg;

	if(!upToDate)
	{
		rangeMin = scanPtr->range_min;
		rangeMax = scanPtr->range_max;
		angleMin = scanPtr->angle_min;
		angleMax = scanPtr->angle_max;
		angleIncrement = scanPtr->angle_increment;
		upToDate = true;
	}

	//	occupancyGridMapping();

}

bool Sensor::isUpToDate() const {
	return upToDate;
}

void Sensor::setUpToDate(bool upToDate) {
	this->upToDate = upToDate;
}

//		if(    x > mapPtr->getColumn() * mapPtr->getResolution() ||
//			   x < 0 ||
//			   y > mapPtr->getRow() * mapPtr->getResolution() ||
//			   y < 0 ||
//			   mapPtr-> isOccupied(y,x))
//		if( (x > mapPtr->getColumn() * mapPtr->getResolution()) || (x < 0) || (y > mapPtr->getRow() * mapPtr->getResolution() ) || (y < 0) || (mapPtr-> isOccupied(y,x) ) )
//		{
//			ROS_INFO("HERE");
//
//			correlation[i] = 0;
//		}
//		else
//		{

//bool isnan (double f) { return (f != f); }

void Sensor::sensorPrediction()
{
//	for (int i = 0; i < numberOfParticle; i++)
//	{
//		ROS_INFO("sensorParticle x = %f, y = %f, theta = %f", particleCloud[i].getX(), particleCloud[i].getY(), particleCloud[i].getTheta());
//	}


		//	ROS_INFO("mapPtr->getResolution() = %f", mapPtr->getResolution()); -- > Corretta!!

	// Le particelle a cui accediamo sono quelle correttamente updated dal model
//	std::cout << "Start sensorPrediction" << std::endl;


	// First check if we get some data from the scan (somethimes can happen that all the measurement are NaN)
	bool allNaN = true;
	int idx = 0;

	for (double check = angleMin; check < angleMax; check = check + angleIncrement)
	{
		if(isnan(scanPtr->ranges[idx]))
		{
			idx = idx + 1;
		}
		else
		{
			allNaN = false;
			break;
		}
	}

	if(!allNaN)
	{
		bool calculated;

		double resolution = mapPtr->getResolution();
		int nColumn = mapPtr->getColumn();
		int nRow = mapPtr->getRow();

		for (int i = 0; i < numberOfParticle; i++)
		{
			idx = 0;
			int count = 0;
			double total = 0;

			// Find the distance to the wall
			double x, y, x_original, y_original, theta, upX, upY;

			x_original = particleCloud[i].getX();
			y_original = particleCloud[i].getY();
			theta = particleCloud[i].getTheta();

			x = x_original;
			y = y_original;

//			ROS_INFO("particle Sensor %d: x = %f, y = %f, theta = %f", i, x, y, theta);
	//		ROS_INFO("noi");
			// Sembra ok

			// Check if they are inside the map
			if(    x > nColumn * resolution ||
				   x < 0 ||
				   y > nRow * resolution ||
				   y < 0)
			{
				correlation[i] = 0;
			}
			else
			{
				// Check if they are in a free space or "in the wall"
				if(mapPtr->isOccupied(y, x)) //-> I don't know why but so it make what we want to see in Rviz
				{
					correlation[i] = 0;
				}
				else
				{
	//				ROS_INFO("Particle inside boundary and not in wall (before laser loop)");

					// Qui siamo sicuri che la particella è nel boundary e non in un muro
					// Find possible Cell -- > Per un quache motivo non mi entra ne loop....


					// Non mi entra nel loop, nessuna idea del perché..
					// Metti flag come per map per l'update del laser!!
					for (double angle = angleMin; angle < angleMax; angle = angle + angleIncrement)
					{
						x = x_original;
						y = y_original;
	//					ROS_INFO("x -> %f, y -> %f", x, y);
	//					ROS_INFO("angle -> %f", angle);
	//					ROS_INFO("index -> %d", idx);
	//					ROS_INFO("range -> %f", scanPtr->ranges[idx]);

						// Check if the scan measurement is NaN
						if(isnan(scanPtr->ranges[idx]))
						{
							idx = idx + 1;
	//						ROS_INFO("nan");
							continue;
						}

						calculated = false;

						// Forse al contrario??

						upX = cos(angle + theta) * resolution;
						upY = sin(angle + theta) * resolution;

	//					ROS_INFO("upX = %f, upY = %f", upX, upY);

						for(double dist = rangeMin; dist < rangeMax; dist = dist + resolution)
						{
							x = x + upX;
							y = y + upY;
	//						ROS_INFO(" In the dist for loop: x = %f, y = %f", x, y);


							if(mapPtr->isOccupied(y,x))
							{

	//							if(scanPtr->ranges[idx] > rangeMin && scanPtr->ranges[idx] < rangeMax)
	//							{
									total = total + std::abs(dist - scanPtr->ranges[idx]);
									count = count + 1;
									calculated = true;
									break;
	//							}
	//							else
	//							{
	//								calculated = false;
	//							}
							}
						}
						if(!calculated)
						{
							total = total + std::abs(rangeMax - scanPtr->ranges[idx]);
							count = count + 1;
						}
						idx = idx + 1;
						calculated = false;

//						std::cout << "total = " << total << std::endl;
					}

				}
//				std::cout << "forse? = " << total << std::endl;

			}

			// Calculate Correlation
			// Store it in the correlation array
			if (count == 0)
			{
				correlation[i] = 0;
	//			std::cout << "total = 0" << std::endl;

			}
			else
			{
				correlation[i] = total/count;
	//			std::cout << "total / count " << total/count << std::endl;

			}
		}


		double total = 0;
		for (int i = 0; i < numberOfParticle; i++)
		{
			total = total + correlation[i];
		}

		for (int i = 0; i < numberOfParticle; i++)
		{
			if(correlation[i] != 0)
			{
				correlation[i] = (total - correlation[i])/total;
			}
	//		std::cout << "corr " << i << " = " << correlation[i] << std::endl;

		}

	}
//
//	for(int i = 0; i < numberOfParticle; i++)
//	{
//		ROS_INFO("sensor correlation %d = %f", i, correlation[i]);
//	}


//	for(int i = 0; i < numberOfParticle; i++)
//	{
//		if(i < 10)
//			correlation[i] = 10;
//		else
//			correlation[i] = 0;
//	}
}












//void Sensor::sensorPrediction()
//{
////	for (int i = 0; i < numberOfParticle; i++)
////	{
////		ROS_INFO("sensorParticle x = %f, y = %f, theta = %f", particleCloud[i].getX(), particleCloud[i].getY(), particleCloud[i].getTheta());
////	}
//
//
//		//	ROS_INFO("mapPtr->getResolution() = %f", mapPtr->getResolution()); -- > Corretta!!
//
//	// Le particelle a cui accediamo sono quelle correttamente updated dal model
////	std::cout << "Start sensorPrediction" << std::endl;
//
//	bool calculated;
//
//	double resolution = mapPtr->getResolution();
//	int nColumn = mapPtr->getColumn();
//	int nRow = mapPtr->getRow();
//
//	for (int i = 0; i < numberOfParticle; i++)
//	{
//		int idx = 0;
//		int count = 0;
//		double total = 0;
//
//		// Find the distance to the wall
//		double x, y, x_original, y_original, theta, upX, upY;
//
//		x_original = particleCloud[i].getX();
//		y_original = particleCloud[i].getY();
//		theta = particleCloud[i].getTheta();
//
//		x = x_original;
//		y = y_original;
//
//		ROS_INFO("particle Sensor %d: x = %f, y = %f, theta = %f", i, x, y, theta);
////		ROS_INFO("noi");
//		// Sembra ok
//
//		// Check if they are inside the map
//		if(    x > nColumn * resolution ||
//			   x < 0 ||
//			   y > nRow * resolution ||
//			   y < 0)
//		{
//			correlation[i] = 0;
//		}
//		else
//		{
//			// Check if they are in a free space or "in the wall"
//			if(mapPtr->isOccupied(y, x)) //-> I don't know why but so it make what we want to see in Rviz
//			{
//				correlation[i] = 0;
//			}
//			else
//			{
////				ROS_INFO("Particle inside boundary and not in wall (before laser loop)");
//
//				// Qui siamo sicuri che la particella è nel boundary e non in un muro
//				// Find possible Cell -- > Per un quache motivo non mi entra ne loop....
//
//
//				// Non mi entra nel loop, nessuna idea del perché..
//				// Metti flag come per map per l'update del laser!!
//				for (double angle = angleMin; angle < angleMax; angle = angle + angleIncrement)
//				{
//					x = x_original;
//					y = y_original;
////					ROS_INFO("x -> %f, y -> %f", x, y);
////					ROS_INFO("angle -> %f", angle);
////					ROS_INFO("index -> %d", idx);
////					ROS_INFO("range -> %f", scanPtr->ranges[idx]);
//
//					// Check if the scan measurement is NaN
//					if(isnan(scanPtr->ranges[idx]))
//					{
//						idx = idx + 1;
////						ROS_INFO("nan");
//						continue;
//					}
//
//					calculated = false;
//
//					// Forse al contrario??
//
//					upX = cos(angle + theta) * resolution;
//					upY = sin(angle + theta) * resolution;
//
////					ROS_INFO("upX = %f, upY = %f", upX, upY);
//
//					for(double dist = rangeMin; dist < rangeMax; dist = dist + resolution)
//					{
//						x = x + upX;
//						y = y + upY;
////						ROS_INFO(" In the dist for loop: x = %f, y = %f", x, y);
//
//
//						if(mapPtr->isOccupied(y,x))
//						{
//
////							if(scanPtr->ranges[idx] > rangeMin && scanPtr->ranges[idx] < rangeMax)
////							{
//								total = total + std::abs(dist - scanPtr->ranges[idx]);
//								count = count + 1;
//								calculated = true;
//								break;
////							}
////							else
////							{
////								calculated = false;
////							}
//						}
//					}
//					if(!calculated)
//					{
//						total = total + std::abs(rangeMax - scanPtr->ranges[idx]);
//						count = count + 1;
//					}
//					idx = idx + 1;
//					calculated = false;
//
//					std::cout << "total = " << total << std::endl;
//				}
//
//			}
//			std::cout << "forse? = " << total << std::endl;
//
//		}
//
//		// Calculate Correlation
//		// Store it in the correlation array
//		if (count == 0)
//		{
//			correlation[i] = 0;
////			std::cout << "total = 0" << std::endl;
//
//		}
//		else
//		{
//			correlation[i] = total/count;
////			std::cout << "total / count " << total/count << std::endl;
//
//		}
//	}
//
//	double total = 0;
//	for (int i = 0; i < numberOfParticle; i++)
//	{
//		total = total + correlation[i];
//	}
//
//	for (int i = 0; i < numberOfParticle; i++)
//	{
//		if(correlation[i] != 0)
//		{
//			correlation[i] = (total - correlation[i])/total;
//		}
////		std::cout << "corr " << i << " = " << correlation[i] << std::endl;
//
//	}
////
////	for(int i = 0; i < numberOfParticle; i++)
////	{
////		ROS_INFO("sensor correlation %d = %f", i, correlation[i]);
////	}
//
//
////	for(int i = 0; i < numberOfParticle; i++)
////	{
////		if(i < 10)
////			correlation[i] = 10;
////		else
////			correlation[i] = 0;
////	}
//}
