#include "potential.h"
#include <math.h>
#include <yarp/math/Math.h>
using namespace yarp::math;
#include <iCub/ctrl/math.h>
using namespace iCub::ctrl;

#include "../common/common.h"

#include <iostream>
using namespace std;

Potential::Potential(void)
{
}

Potential::Potential(double x, double y, double myRadius, double myPotential, int myAge) : radius(myRadius), potential(myPotential), age(myAge)//, reached(false)
{
	position = Vector(2);
	position[0] = x;
	position[1] = y;
}

Potential::Potential(const Potential &myPotential):
position(myPotential.position),
radius(myPotential.radius),
potential(myPotential.potential),
age(myPotential.age)
{
}


Potential::~Potential(void)
{
}

const Vector &Potential::GetPosition(void) const
{
	return position;
}

Vector Potential::GetPotentialVector(void) const
{
    Vector potentialVector(2);

    //if(reached)
    //{
    //    potentialVector.x = 0;
    //    potentialVector.y = 0;
    //    return potentialVector;
    //}

	Vector normalizedPotentialVector;
	double positionNorm = norm(position);
    normalizedPotentialVector = Normalize(-potential * position);

    double distance = positionNorm - radius;

    if(potential < 0)
    {
        potentialVector = normalizedPotentialVector * NEGATIVE_GRADIENT_EXPRESSION;
    }
    else
    {
        if(distance > (double)d0)
        {
            potentialVector.zero();
        }
        else
        {
            potentialVector = normalizedPotentialVector * POSITIVE_GRADIENT_EXPRESSION;
        }
    }

	return potentialVector;
}

void Potential::Translate(const Vector &t)
{
    position = position + t;
}

void Potential::Rotate(double angle)
{
	double teta = radians(angle);
	double xPrime, yPrime;
	xPrime = position[0] * cos(teta) - position[1] * sin(teta);
	yPrime = position[1] * cos(teta) + position[0] * sin(teta);

	position[0] = xPrime;
	position[1] = yPrime;
}

void Potential::SetReached(void)
{
    /*reached = true;*/
    potential = 0;
}

double Potential::GetRadius(void) const
{
    return radius;
}


double Potential::GetPotential(void) const
{
    return potential;
}

void Potential::MoveTo(double x, double y)
{
	position[0] = x;
	position[1] = y;
}


int Potential::GetAge()
{
	return age;
}

void Potential::GetOlder()
{
	age++;
}
