#include "Particle.h"
#include <iostream>

Particle::Particle(float x, float y, float mass)
{
	this->radius = 4.0;
	this->position = Vec2{ x, y };
	this->velocity = Vec2{ 0.0f, 0.0f };
	this->acceleration = Vec2{ 0.0f, 0.0f };
	this->mass = mass;
	std::cout << "Particle constructor called" << std::endl;
}

Particle::~Particle()
{
	std::cout << "Particle destructor called" << std::endl;
}