#ifndef VTGS_H
#define VTGS_H

#pragma warning( disable : 4244 )

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <pmp_lib/core/tbg.h>

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

namespace iCub
{
namespace pmplib
{
namespace core
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
	bool hasPose;
	bool hasInitialPose;
	unsigned int numTargetPoints;

	// class methods 
	VirtualTrajectoryGenerator(const Property *options);
	VirtualTrajectoryGenerator(const VirtualTrajectoryGenerator &options);
	~VirtualTrajectoryGenerator();

	bool setTarget(const Matrix &tg);
	bool setTarget(const Vector &tg, bool back);
	bool setTargetPose(const Matrix &tgPose);
	bool setStartingPoint(const Vector &x0);
	bool setStartingPose(const Matrix &initPose);
	bool setWeights(const Matrix &w);
	bool setWeights(const Vector &w);
	bool setTbg(TimeBaseGenerator &tbg, const double &T_init, const double &T_dur, const double &SlopeRamp, const double &alpha);

	Matrix getTarget();
	//Vector getTarget();
	Matrix getWeights();
	void run(unsigned int iter, Vector &x_target);
	void getNextVirtualPosition(Vector &x);
	Vector getNextVirtualPosition(const int &iter);
	Matrix getNextVirtualPose(const int &iter);
	Vector getAxisAngle(const Matrix &initPose, const Matrix &tgPose);
	int maxIter();
	bool update(const Property &options);
	
private:
	// class variables
	Vector x_virtual;
	Matrix x_tg;     // stores 3x3D points of 3 subsequent critical points
	Matrix tgPose;
	Matrix virtualPose;
	Matrix initPose;
	Matrix dRot;
	Vector axisAngle;
	double theta;
	Semaphore runSem;
	Property *options;

	// class methods
//	void getNextVirtualPosition(Vector &x);
	bool initialize();
	//bool getListFromProperty();
	bool Bottle2Vector(Bottle *bot, Vector &v);
	Bottle PropertyGroup2Bottle(Bottle group);
	bool setFromProperty(string key, Vector &v);
	bool setFromProperty(string key, Matrix &m);
	
	//ofstream traj;

};

class vtgsBlock
{
	Vector _tg;
	Matrix _tgPose;
	int _iter;
	bool initialized;
public:
	int maxIter;
	VirtualTrajectoryGenerator *vtgs;

	vtgsBlock():vtgs(0),maxIter(0),_iter(0), initialized(0)
	{
		_tg.resize(3,0.0);
		_tgPose.resize(3,3);
		_tgPose.eye();
	};
	vtgsBlock(const vtgsBlock& b)
	{
		this->_iter = b._iter;
		this->maxIter = b.maxIter;
		this->initialized = b.initialized;
		if (initialized)	vtgs = new VirtualTrajectoryGenerator(*b.vtgs);
		else				vtgs = NULL;

		this->_tg.resize(3);
		this->_tg = b._tg;

		this->_tgPose.resize(3,3);
		this->_tgPose = b._tgPose;
	}
	vtgsBlock &operator =(const vtgsBlock& b)
	{
		if (this != &b)
		{
			if (this->initialized) delete this->vtgs;
			if (!b.initialized)	this->vtgs = NULL;
			else				this->vtgs = new VirtualTrajectoryGenerator(*b.vtgs);
			this->initialized = b.initialized;
			this->_iter = b._iter;
			this->maxIter = b.maxIter;
			this->_tg.resize(3);
			this->_tg = b._tg;
			this->_tgPose.resize(3,3);
			this->_tgPose = b._tgPose;
		}
		return *this;
	};
	int iter(){return _iter;};
	void reset(){_iter = 0;};
	/*
	void init(Property *options)
	{
		if (initialized) return update(*options);
		vtgs = new VirtualTrajectoryGenerator(options);
		maxIter = (int)(vtgs->tbg1.getT_dur() + vtgs->tbg1.getT_init())/vtgs->tbg1.getSlopeRamp()+1;
		initialized = true;
	};
	*/
	void init(const Property *options)
	{
		if (initialized) return update(*(const_cast<Property*>(options)));
		vtgs = new VirtualTrajectoryGenerator(options);
		maxIter = (int)(vtgs->tbg1.getT_dur() + vtgs->tbg1.getT_init())/vtgs->tbg1.getSlopeRamp()+1;
		initialized = true;
	};
	void update(const Property &options)
	{
		if (!initialized) return init(&options);
		vtgs->update(options);
		maxIter = (int)(vtgs->tbg1.getT_dur() + vtgs->tbg1.getT_init())/vtgs->tbg1.getSlopeRamp()+1;
		_iter = 0;
	};
	void setTarget(const Vector &tg, Matrix * tgPose = NULL)
	{
		if (!initialized) {cout << "not initialized" << endl;return;}
		vtgs->setTarget(tg,false);
		_tg = tg;

		if(tgPose!=NULL)
		{
			vtgs->setTargetPose(*tgPose);
			_tgPose = *tgPose;
		}
		_iter = 0;
	};
	void setInitialState( const Vector &x0, Matrix * initPose = NULL)
	{
		if (!initialized) {cout << "not initialized" << endl;return;}
		vtgs->setStartingPoint(x0);
		
		if(initPose!=NULL)
			vtgs->setStartingPose(*initPose);
	};
	void setMode( const string &mode){};
	Vector step()
	{
		if (!initialized) {cout << "not initialized" << endl; return _tg;}
		_iter++;
		if (_iter > maxIter) return _tg;
		else
			return vtgs->getNextVirtualPosition(_iter);
	};

	void stepAll(Vector &position, Matrix &pose)
	{
		if (!initialized) {cout << "not initialized" << endl; return;}
		_iter++;
		if (_iter > maxIter)
		{
			position = _tg;
			pose = _tgPose;
			return;
		}
		else
		{
			position = vtgs->getNextVirtualPosition(_iter);
			pose = vtgs->getNextVirtualPose(_iter);
			return;
		}
	};

	VirtualTrajectoryGenerator * generator()
	{
		return vtgs;
	};
	bool hasFinished(const int &iter)
	{
		return iter > maxIter;
	};
	inline bool hasFinished()
	{
		bool ok= _iter > maxIter;
		return ok;
	};
	double TimeStep(int tbg=1)
	{
		if(tbg==1)	return vtgs->tbg1.getSlopeRamp();
		else		return vtgs->tbg1.getSlopeRamp();
	};
};
}// end core namespace
}// end pmplib namespace
}// end iCub namespace

#endif
