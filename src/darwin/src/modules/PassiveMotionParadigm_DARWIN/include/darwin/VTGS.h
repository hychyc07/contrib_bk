#ifndef VTGS_H
#define VTGS_H

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include "darwin/tbg.h"

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/os/Property.h>
#include <yarp/os/Semaphore.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

namespace Darwin
{
namespace pmp
{

class VirtualTrajectoryGenerator
{
public:
	// class variables

	//typedef enum {bump, ...} shape;

	TimeBaseGenerator tbg1;
	TimeBaseGenerator tbg2;
	Matrix weights;
	unsigned int time;
	bool hasTarget;
	unsigned int numTargetPoints;

	// class methods 
	VirtualTrajectoryGenerator(Property *options);
	~VirtualTrajectoryGenerator();

	bool setTarget(Matrix tg);
	bool setTarget(Vector tg);
	bool setStartingPoint(Vector x0);
	bool setWeights(Matrix w);
	bool setWeights(Vector w);
	bool setTbg(TimeBaseGenerator &tbg, double T_init, double T_dur, double SlopeRamp, double alpha);

	Matrix getTarget();
	//Vector getTarget();
	Matrix getWeights();
	void run(unsigned int iter, Vector &x_target);

private:
	// class variables
	Vector x_virtual;
	Matrix x_tg;     // stores 3x3D points of 3 subsequent critical points
	Semaphore runSem;
	Property *options;

	// class methods
	void getNextVirtualPosition(Vector &x);
	bool initialize();
	//bool getListFromProperty();
	bool Bottle2Vector(Bottle *bot, Vector &v);
	Bottle PropertyGroup2Bottle(Bottle group);
	bool setFromProperty(string key, Vector &v);
	bool setFromProperty(string key, Matrix &m);

	//ofstream traj;

};

}// end pmp namespace
}// end Darwin namespace

#endif
