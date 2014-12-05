/*
 * Resampler.cpp
 *
 *  Created on: Dec 5, 2014
 *      Author: mafilipp
 */

#include "Resampler.h"

Resampler::Resampler(Particle * pc, int numPart, Map map, double * cor) {
	// TODO Auto-generated constructor stub
	particleCloud = pc;
	numberOfParticle = numPart;
	mapPtr = &map;
	correlation = cor;

}

Resampler::~Resampler() {
	// TODO Auto-generated destructor stub
}

void Resampler::resampleMap()
{
	// TODO
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
