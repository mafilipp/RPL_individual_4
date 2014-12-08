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

	for(int i = 0; i < numberOfParticle; i++)
	{
		// Get x and y
		x = particleCloud[i].getX();
		y = particleCloud[i].getY();

		// Check if they are inside the map
		if(    x > mapPtr->getColumn() * mapPtr->getResolution() ||
			   x < 0 ||
			   y > mapPtr->getRow() * mapPtr->getResolution() ||
			   y < 0)
		{
			correlation[i] = 0;
		}
		else
		{
			// Check if they are in a free space or "in the wall"
			if(mapPtr->isOccupied(x, y)) //-> I don't know why but so it make what we want to see in Rviz
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

	// Normalize the correlation vector
	for(int i = 0; i < numberOfParticle; i++)
	{
		correlation[i] = correlation[i] / sum;
	}

	// Vector that we need for the resampling
	double beta[numberOfParticle + 1];

	sum = 0;

	// Full the vector for resampling
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

	// Create a temporary vector for the store of the new particle
	Particle * resampledParticle = new Particle[numberOfParticle];


	// Choose the weighted particle
	// Loop through the particle vector
	for(int i = 0; i < numberOfParticle; i++)
	{
		random = (double)rand()/RAND_MAX;

		// Loop through beta
		for(int j = 1; j < numberOfParticle + 1; j++)
		{
				if(beta[j-1] <= random && random < beta[j])
				{
					resampledParticle[i] = particleCloud[j-1];
//					std::cout << "choosen particle number " << j << std::endl;
				}
		}
	}

	// Fill the vector with the new particle
	for(int i = 0; i < numberOfParticle; i++)
	{
		particleCloud[i] = resampledParticle[i];
	}

//	delete resampledParticle; --> chiedi Andrey

}

void Resampler::resampleUniversal()
{
	// In this function we don't need to find the correlation vector since it is already given by the sensor update
	double sum = 0;

//	for(int i = 0; i < numberOfParticle; i++)
//	{
//		ROS_INFO("resample correlation %d = %f", i, correlation[i]);
//	}

	for(int i = 0; i < numberOfParticle; i++)
	{
		sum = sum + correlation[i];
	}

	// Normalize the correlation vector
	for(int i = 0; i < numberOfParticle; i++)
	{
		correlation[i] = correlation[i] / sum;
	}

	// Vector that we need for the resampling
	double beta[numberOfParticle + 1];

	sum = 0;

	// Full the vector for resampling
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

	// Create a temporary vector for the store of the new particle
	Particle * resampledParticle = new Particle[numberOfParticle];

	double random = 0;

	// Choose the weighted particle
	// Loop through the particle vector
	for(int i = 0; i < numberOfParticle; i++)
	{
		random = (double)rand()/RAND_MAX;

		// Loop through beta
		for(int j = 1; j < numberOfParticle + 1; j++)
		{
				if(beta[j-1] <= random && random < beta[j])
				{
					resampledParticle[i] = particleCloud[j-1];
//					std::cout << "choosen particle number " << j << std::endl;
				}
		}
	}

	// Fill the vector with the new particle
	for(int i = 0; i < numberOfParticle; i++)
	{
		particleCloud[i] = resampledParticle[i];
	}
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
