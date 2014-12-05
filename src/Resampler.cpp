/*
 * Resampler.cpp
 *
 *  Created on: Dec 5, 2014
 *      Author: mafilipp
 */

#include "Resampler.h"

Resampler::Resampler(Particle * pc, int numPart, Map *map, double * cor) {
	// TODO Auto-generated constructor stub
	particleCloud = pc;
	numberOfParticle = numPart;
	mapPtr = map;
	correlation = cor;

//	ROS_INFO("Resampler resolution before %f", mapPtr->getResolution());
//	mapPtr->setResolution(13.2);
//	ROS_INFO("Resampler resolution  %f", mapPtr->getResolution());
//
//	ROS_INFO("particle ptr -> 0.x = %f", particleCloud[0].getX());


}

Resampler::~Resampler() {
	// TODO Auto-generated destructor stub
}
void Resampler::debug()
{
	std::cout << "deb" << std::endl;
	  std::cout << particleCloud[55].getX() << std::endl;
	  std::cout << particleCloud[55].getTheta() << std::endl;
}
void Resampler::resampleMap()
{

	// Find the total
	double sum = 0;
	double random;
	float x,y;

	// Create a new particle vector

//	ROS_INFO("sum prima = %f", sum);
//	ROS_INFO("nOp = %d", numberOfParticle);

//	ROS_INFO("2 = %f", mapPtr->getColumn() * mapPtr->getResolution()); -> giusto!!

//	std::cout << "All particle" << std::cout;
//	for(int i = 0; i < numberOfParticle; i++)
//	{
//		std::cout << "x = " << particleCloud[i].getX() << "  y = " << particleCloud[i].getY() << std::endl;
//
//	}
	for(int i = 0; i < numberOfParticle; i++)
	{

		x = particleCloud[i].getX();
		y = particleCloud[i].getY();
//		ROS_INFO("Particle %d  x = %f,  y = %f", i, x, y);

		if(    x > mapPtr->getColumn() * mapPtr->getResolution() ||
			   x < 0 ||
			   y > mapPtr->getRow() * mapPtr->getResolution() ||
			   y < 0)
		{
			correlation[i] = 0;
		}
		else
		{
			if(mapPtr->isOccupied(y, x)) //-> I don't know why but so it make what we want to see in Rviz
			{
				correlation[i] = 0;
			}
			else
			{
				correlation[i] = 1;
				sum = sum + 1;
			}
		}
	}

//
	std::cout << sum << std::endl;

	// Normalize the correlation vector
	for(int i = 0; i < numberOfParticle; i++)
	{
		correlation[i] = correlation[i] / sum;
	}

	double beta[numberOfParticle + 1];
	sum = 0;

	// Calculate the vector for resampling
	for(int i = 0; i < numberOfParticle + 1; i++)
	{
		if (i == 0)
		{
			beta[i] = 0;
		}
		else
		{
			sum = sum + correlation[i-1];
			beta[i] = sum;
		}

	}

//	ROS_INFO("start");
//for(int a = 0; a < numberOfParticle; a++)
//{
//	std::cout << beta[a] << std::endl;
//}
//
//ROS_INFO("end");
//

	Particle * resampledParticle = new Particle[numberOfParticle];


	// Choose the weighted particle
	// Loop through the particle vector

	ROS_INFO("Here");
	for(int i = 0; i < numberOfParticle; i++)
	{
		random = (double)rand()/RAND_MAX;

//		std::cout << "rand " << random << std::endl;

		// Loop through beta
		for(int j = 1; j < numberOfParticle + 1; j++)
		{

				if(beta[j-1] <= random && random < beta[j])
				{
					resampledParticle[i] = particleCloud[j-1];
					std::cout << "choose" << j << std::endl;
				}

		}
	}


	for(int i = 0; i < numberOfParticle; i++)
	{
		particleCloud[i] = resampledParticle[i];
	}

//	delete resampledParticle; --> chiedi Andrey

}

void Resampler::resampleUniversal()
{
	//TODO
}

//void Sensor::resample(double * correlation)
//{
//	// Find the total
//	double sum = 0;
//	double random;
//
//	// Create a new particle vector
//	Particle * resampledParticle = new Particle[numberOfParticle];
//
//
//	for(int i = 0; i < numberOfParticle; i++)
//	{
//		sum = sum + correlation[i];
//	}
//
//	// Normalize the correlation vector
//	for(int i = 0; i < numberOfParticle; i++)
//	{
//		correlation[i] = correlation[i] / sum;
//	}
//
//	double beta[numberOfParticle];
//	sum = 0;
//	// Calculate the vector for resampling
//	for(int i = 0; i < numberOfParticle; i++)
//	{
//		sum = sum + correlation[i];
//		beta[i] = sum;
//	}
//
//	// Choose the weighted particle
//	for(int i = 0; i < numberOfParticle; i++)
//	{
//		random = rand()/RAND_MAX;
//		for(int j = 0; j < numberOfParticle; j++)
//		{
//			if(random < correlation[j])
//				resampledParticle[i] = particleCloud[j];
//		}
//	}
//
//	particleCloud =  resampledParticle;
//
//}
