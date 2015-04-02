#ifndef GAZECONTROLLER_HPP_
#define GAZECONTROLLER_HPP_



/*
 *  gazeController.h
 *  HandSome
 *
 *  Created by Jiuguang Wang on 7/27/10.
 *  Copyright 2010 Jiuguang Wang. All rights reserved.
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>

#include <gsl/gsl_math.h>

#include <stdio.h>
#include <deque>

#define CTRL_THREAD_PER     0.02        // [s]
#define PRINT_STATUS_PER    1.0         // [s]
#define STORE_POI_PER       3.0         // [s]
#define SWITCH_STATE_PER    10.0        // [s]
#define STILL_STATE_TIME    5.0         // [s]

#define STATE_TRACK         0
#define STATE_RECALL        1
#define STATE_WAIT          2
#define STATE_STILL         3

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

#include "controller.hpp"

#define DAMP	0.01	/*	N/w		*/
#define STIFF	0.16	/*	N/deg	*/

using namespace yarp;
using namespace yarp::dev;

namespace HandSome {
	class GazeControlThd: public ControllerThd {
		
	protected:
		PolyDriver       *client;
		PolyDriver       *clientTorso;
		IGazeControl     *igaze;
		IEncoders        *ienc;
		IPositionControl *ipos;
		Vector compJ; //Compliant joints

		int state;
		
		Vector fp;
		
		deque<Vector> poiList;
		
		double t;
		double t0;
		double t1;
		double t2;
		double t3;
		double t4;

	private:

		
	public:
		GazeControlThd(PolyDriver *p, Vector &compJoints, double period);

		//Called after thread->start()
		virtual bool threadInit();
		
		//Core of the thread
		virtual void run();
		
		//Called after thread->stop()
		virtual void threadRelease();
		
		void generateTarget();
		void storeInterestingPoint();
		double norm(const Vector &v);
		void printStatus();
		void setFixationPoint(Vector point);
	};
	
}

#endif
