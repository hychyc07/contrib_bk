/*
 *  gazeController.cpp
 *  HandSome
 *
 *  Created by Jiuguang Wang on 7/27/10.
 *  Copyright 2010 Jiuguang Wang. All rights reserved.
 *
 */

#include "gazeController.hpp"
#include <yarp/os/Time.h>

using namespace HandSome;

/*
 Input Parameters:
 Polydriver p 	- device to control
 compJoints		- the j-th element is 1 if the corresponding joint has to be controlled in impedance
 the j-th element is 0 if the corresponding joint has to be controlled in position
 period			- period of the thread
 */

GazeControlThd::GazeControlThd(PolyDriver* p, Vector& compJoints, double period) : ControllerThd(double(period)) {
	this->client = p;
	this->compJ = compJoints;
}


bool GazeControlThd::threadInit() {
	if (!client)
		return false;

	// open the view
	client->view(igaze);
	
	// set trajectory time:
	// we'll go like hell since we're using the simulator :)
	igaze->setNeckTrajTime(0.4);
	igaze->setEyesTrajTime(0.1);
	
	// put the gaze in tracking mode, so that
	// when the torso moves, the gaze controller 
	// will compensate for it
	igaze->setTrackingMode(true);
	
	
	
	
//	Property optTorso("(device remote_controlboard)");
//	optTorso.put("remote","/icubSim/torso");
//	optTorso.put("local","/torso_client");
//	
//	clientTorso=new PolyDriver;
//	if (!clientTorso->open(optTorso))
//	{
//		delete clientTorso;    
//		return false;
//	}
//	
//	// open the view
//	clientTorso->view(ienc);
//	clientTorso->view(ipos);
//	ipos->setRefSpeed(0,10.0);
//	
	fp.resize(3);
	
	state=STATE_TRACK;
	
	t=t0=t1=t2=t3=t4=Time::now();
	
	return true;
}

/*
 The setpoint vector cannot be read and written at the same time.
 This is controlled by a mutex mechanism.
 */
void GazeControlThd::run() {
	t=Time::now();
	
	//generateTarget();  
	
	igaze->lookAtFixationPoint(fp);
	printStatus();
}

void GazeControlThd::threadRelease() {
	// we require an immediate stop
	// before closing the client for safety reason
	// (anyway it's already done internally in the
	// destructor)
	igaze->stopControl();
	
	// it's a good rule to reinstate the tracking mode
	// as it was
	igaze->setTrackingMode(false);
	
	delete client;
	
	cout << "Gaze control thread released!" << endl;
}

void GazeControlThd::generateTarget() {   
	// translational target part: a circular trajectory
	// in the yz plane centered in [-0.5,0.0,0.3] with radius=0.1 m
	// and frequency 0.1 Hz
	fp[0]=-0.5;
	fp[1]=+0.0+0.1*cos(2.0*M_PI*0.1*(t-t0));
	fp[2]=+0.3+0.1*sin(2.0*M_PI*0.1*(t-t0));            
}

void GazeControlThd::storeInterestingPoint() {
	if (t-t3>=STORE_POI_PER) {
		Vector ang;
		
		// we store the current azimuth, elevation
		// and vergence wrt the absolute reference frame
		// The absolute reference frame for the azimuth/elevation couple
		// is head-centered with the robot in rest configuration
		// (i.e. torso and head angles zeroed). 
		igaze->getAngles(ang);            
		
		fprintf(stdout,"Storing POI #%d ... %s [deg]\n",
				poiList.size(),ang.toString().c_str());
		
		poiList.push_back(ang);
		
		t3=t;
	}
}

double GazeControlThd::norm(const Vector &v) {
	return sqrt(dot(v,v));
}

void GazeControlThd::printStatus() {        
	if (t-t1>=PRINT_STATUS_PER) {
		Vector x;
		
		// we get the current fixation point in the
		// operational space
		igaze->getFixationPoint(x);
		
		fprintf(stdout,"+++++++++\n");
		fprintf(stdout,"fp         [m] = %s\n",fp.toString().c_str());
		fprintf(stdout,"x          [m] = %s\n",x.toString().c_str());
		fprintf(stdout,"norm(fp-x) [m] = %g\n",norm(fp-x));
		fprintf(stdout,"---------\n\n");
		
		t1=t;
	}
}

void GazeControlThd::setFixationPoint(Vector point) {
	fp = point;
}
