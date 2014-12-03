/*
 * Particle.cpp
 *
 *  Created on: Dec 3, 2014
 *      Author: mafilipp
 */

#include "Particle.h"

Particle::Particle() {
	// TODO Auto-generated constructor stub

}

Particle::~Particle() {
	// TODO Auto-generated destructor stub
}

double Particle::getTheta() const {
	return theta;
}

void Particle::setTheta(double theta) {
	this->theta = theta;
}

double Particle::getX() const {
	return x;
}

void Particle::setX(double x) {
	this->x = x;
}

double Particle::getY() const {
	return y;
}

void Particle::setY(double y) {
	this->y = y;
}
