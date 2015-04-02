// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2011 Dalia De Santis, Jacopo Zenzeri
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/

// $Id: Matrix.h,v 1.16 2008-10-27 19:00:32 natta Exp $ 

#ifndef GRASP_H
#define GRASP_H

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <yarp/os/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
//#include <yarp/os/Semaphore.h>

/**
* \file grasper.h contains the definition of a grasper type 
*/

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace yarp::sig::draw;

namespace Darwin{
namespace grasper{

	typedef enum{null, top, side} graspType;

	Vector Bottle2Vector(Bottle Bot);	
	Bottle Vector2Bottle(Vector v);

class Grasper
{
	private:
	string portName;
	graspType string2enum(string type);
	string enum2string(graspType type);
	bool sendACK();
	Bottle speeds;
	Bottle offset;

	public:
		Grasper(string portName);
		~Grasper();

		//Semaphore mutex;
		//VectorOf<RGBenum> ReadTargetColor();
		Port cmdOut;

		bool StartPorts();
		bool InterruptPorts();
		bool ClosePorts();
		
		Bottle Grasp(Bottle primitives, string type, string side, bool sensitive = true);
		Bottle Tighten(string side);
		Bottle Release(Bottle homeAngles, string side);
		void setGraspSpeeds(const Bottle & speeds);
		Bottle getGraspSpeeds();
		void setOffset(const Bottle & offset);
		Bottle getOffset();
		bool graspSucceeded();
		bool hasFinished();

	private:
		Bottle getTightenAngles(string side);
		Bottle getEncoders(string side);
};
}// end namespace grasper
}// end namespace Darwin
#endif // grasper_H
