
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Andrea Del Prete, Alexander Schmitz
 * email:   andrea.delprete@iit.it, alexander.schmitz@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/Time.h>
#include <algorithm>
#include <math.h>

#include "iCub/tactileGrasp/GraspThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::tactileGrasp;


const int GraspThread::MOTOR_MIN[] = { 0,  0,  0,  0,   0};
const int GraspThread::MOTOR_MAX[] = {50, 50, 50, 50, 150};

GraspThread::GraspThread(ResourceFinder* rf, string robotName, string moduleName, bool rightHand)
{
	this->rf			= rf;
	this->robotName		= robotName;
	this->moduleName	= moduleName;
	this->rightHand		= rightHand;
}

bool GraspThread::threadInit() 
{
	// open the compensated tactile data port
	string skinPortName;
	if(rightHand)
	    skinPortName = "/icub/skin/right_hand_comp";
    else
        skinPortName = "/icub/skin/left_hand_comp";
	if (!compensatedTactileDataPort.open( ("/"+moduleName+"/skin:i").c_str() )) {
        fprintf(stderr, "Unable to open the tactile data input port ");
		return false;  // unable to open; let RFModule know so that it won't run
	}
    if( !Network::connect(skinPortName.c_str(), compensatedTactileDataPort.getName().c_str()) ){
        fprintf(stderr, "Unable to connect to tactile data port: %s\n", skinPortName.c_str());
		return false;  // unable to connect; let RFModule know so that it won't run
    }

    // open the streaming output monitoring port
    string monitorPortName = "/"+moduleName+"/monitor:o";
    if(!monitorPort.open(monitorPortName.c_str())){
        fprintf(stderr, "Unable to open the monitor output port!");
		return false;  // unable to open; let RFModule know so that it won't run
    }


	/* initialize variables and create data-structures if needed */
	Property params;
	params.put("robot", robotName.c_str());	
	params.put("device", "remote_controlboard");	
	if(rightHand){
		params.put("part", "right_arm");
		params.put("local", ("/"+moduleName+"/right_arm").c_str());   //local port names
		params.put("remote", ("/"+robotName+"/right_arm").c_str());		
	}
	else{
		params.put("part", "left_arm");
		params.put("local", ("/"+moduleName+"/left_arm").c_str());   //local port names
		params.put("remote", ("/"+robotName+"/left_arm").c_str());
	}

	// create a device
	robotDevice = new PolyDriver(params);
	if (!robotDevice->isValid()) {
		printf("Device not available.  Here are the known devices:\n");
		printf("%s", Drivers::factory().toString().c_str());
		return false;
	}	

	bool ok;
	ok = robotDevice->view(pos);
    ok = ok && robotDevice->view(vel);
	ok = ok && robotDevice->view(encs);
	if (!ok) {
		printf("Problems acquiring interfaces\n");
		return false;
	}

	// set reference speed and acceleration
	pos->getAxes(&NUM_JOINTS); //the number of axes
	for (int i = 0; i < NUM_JOINTS; i++){
		if(i<7){
			pos->setRefSpeed(i, REF_SPEED_ARM);
			pos->setRefAcceleration(i, REF_ACC_ARM);
		}
		else{
			pos->setRefSpeed(i, REF_SPEED_FINGER);
			pos->setRefAcceleration(i, REF_ACC_FINGER);
		}		
	}

	speedNoTouch	= VELOCITY_TOUGH;
	graspComm		= doNothing;			// when the thread starts, it does nothing until it gets a command 
	touch_threshold	= 1;					// default touch threshold
	touch_desired = 5;						// default touch desired

	percentile.resize(SKIN_DIM, 0);
	touchError.resize(NUM_FINGERS, (float)touch_desired);	// so that the error derivative is zero the first step (assuming a starting point with no touch)
	touchErrorOld.resize(NUM_FINGERS, 0);
	touchErrorSum.resize(NUM_FINGERS, 0);
	touchPerFinger.resize(NUM_FINGERS, 0);	
	touchBandPerFinger.resize(NUM_FINGERS);
	K_P.resize(NUM_FINGERS, 1.0);
	K_I.resize(NUM_FINGERS, 0.0);
	K_D.resize(NUM_FINGERS, 0.0);
    velocities.resize(NUM_FINGERS, 0);
	encoders.resize(NUM_JOINTS);
    doControl = false;
	
	return true;
}

void GraspThread::threadRelease() 
{
	setArmJointsPos();

	// close the poly driver
	if(robotDevice)
		robotDevice->close();
}
void GraspThread::run(){
    fprintf(stderr, "RUNNING\n\n");	
	
	range zeroRange;
	zeroRange.min=0;
	zeroRange.max=0;

	//Then it starts cycling.
	while (!isStopping()) { // the thread continues to run until isStopping() returns true
		graspCommSem.wait();

		if(graspComm==tough){
			speedTouch		= VELOCITY_TOUGH;
			doControl		= false;
		}
		else if(graspComm==soft){
			speedTouch		= VELOCITY_SOFT;
			doControl		= false;
		}
		else if(graspComm==compliant){
			speedTouch		= VELOCITY_COMPLIANT;
			doControl		= false;
		}
		else if(graspComm==doNothing){
			graspCommSem.post();
			Time::delay(1.0);			// this method of "doing nothing" causes a "busy waiting", but keeps everything simple
			continue;
		}
		else if(graspComm==openHand){
			setArmJointsPos();
			touchErrorSum.assign(touchErrorSum.size(), 0);	// reset the touch error integral used by the controller
			graspComm = doNothing;
			graspCommSem.post();
			continue;
		}
		else if(graspComm==control){
			doControl = true;
		}
		else{
			graspCommSem.post();
			fprintf(stderr, "\n[ERROR] Unknown grasping command in GraspThread\n");
			return;
		}

		graspCommSem.post();

		// read the motor encoders and the tactile sensors		
		encs->getEncoders(encoders.data());
		Vector compensatedData = *(compensatedTactileDataPort.read());
		
		// FIND THE MAX TOUCH FOR EACH FINGER
		touchPerFinger.assign(touchPerFinger.size(), 0);
		touchBandPerFinger.assign(touchBandPerFinger.size(), zeroRange);
		percentileSem.wait();		// make sure the percentile is not modified during this phase
		for (int i=0; i<48; i++){
			if( compensatedData[i]-percentile[i] > touchPerFinger[i/12] ){
				touchPerFinger[i/12] = (float)(compensatedData[i]-percentile[i]);
				touchBandPerFinger[i/12].min = (float)(compensatedData[i]-percentile[i]);
			}
			if( compensatedData[i]+percentile[i] > touchBandPerFinger[i/12].max )
				touchBandPerFinger[i/12].max = (float)(compensatedData[i]+percentile[i]);
		}
		percentileSem.post();		// now the percentile can be modified again

		// compute the touch errors (even if the controller is not going to be executed,
		// so that the touchError and touchErrorOld values are always sensible!)
		controllerSem.wait();	// make sure the touch_desired is not modified
		for(int i=0; i<NUM_FINGERS; i++){
			touchErrorOld[i] = touchError[i];
			if(touchBandPerFinger[i].max < touch_desired)
				touchError[i] = touch_desired - touchBandPerFinger[i].max;	// touch desired is higher than the band
			else if(touchBandPerFinger[i].min > touch_desired)
				touchError[i] = touch_desired - touchBandPerFinger[i].min;	// touch desired is lower than the band
			else
				touchError[i] = 0;											// touch desired is within the band
		}
		controllerSem.post();	// now the touch_threshold can be modified again
		
		if(doControl)
			controller();
		else
			grasp();

        sendMonitoringData();
	}
}

void GraspThread::sendMonitoringData(){
    Vector& monitorData = monitorPort.prepare();
    monitorData.clear();
    monitorData.resize(NUM_FINGERS*2+1);

    // add the touch desired
    controllerSem.wait();	// make sure the touch_desired is not modified
    monitorData.push_back(touch_desired);
    controllerSem.post();

    // add the touch band of each finger
    for(int i=0; i<NUM_FINGERS; i++){
        monitorData.push_back(touchBandPerFinger[i].min);
        monitorData.push_back(touchBandPerFinger[i].max);
    }
    
    if(doControl){
        // add the output finger velocities
        for(int i=0; i<NUM_FINGERS; i++){
            monitorData.push_back(velocities[i]);
        }
    }

    monitorPort.write();
}

void GraspThread::controller(){
	// compute the touch error derivatives and the velocities
	controllerSem.wait();	// make sure K_P, K_I and K_D are not modified
	for(int i=0; i<NUM_FINGERS; i++){
		touchErrorSum[i] += touchError[i];
		velocities[i] = K_P[i] * touchError[i]	+	
						K_I[i] * touchErrorSum[i]	+
						K_D[i] * (touchError[i]-touchErrorOld[i]);
	}
	controllerSem.post();	// now K_P, K_I and K_D can be modified again

	// to command the ring and pinky velocities use the smaller between the two, 
	// so as to prevent strong contacts that could damage the tactile sensors
	velocities[2] = min(velocities[2], velocities[3]);
	
	safeVelocityMove(11, velocities[0]);	// index
	safeVelocityMove(12, velocities[0]);	// index
	safeVelocityMove(13, velocities[1]);	// middle
	safeVelocityMove(14, velocities[1]);	// middle
	if(robotName=="icubSim")
		safeVelocityMove(15, velocities[2]);	// ring-pinky
	else
		safeVelocityMove(15, velocities[2]*2);	// ring-pinky (double velocity because otherwise it is slower than the others)
}

void GraspThread::grasp(){
	vector<bool> detect_touch_finger(NUM_FINGERS-1, false);
	
    //fprintf(stderr, "Encoder data: %s\n", encoders.toString().c_str());
	
	touchThresholdSem.wait();	// make sure the touch threshold is not modified during this phase
	
	if(touchPerFinger[0]>touch_threshold)
			detect_touch_finger[0] = true;
	if(touchPerFinger[1]>touch_threshold)
			detect_touch_finger[1] = true;
	if(touchPerFinger[2]>touch_threshold || touchPerFinger[3]>touch_threshold)
			detect_touch_finger[2] = true;
		
	touchThresholdSem.post();	// now the percentile can be modified again

	//index and middle fingers
	for(unsigned int i=0; i<detect_touch_finger.size()-1; i++){
		if (detect_touch_finger[i]){
			safeVelocityMove(11+2*i, speedTouch);
			safeVelocityMove(12+2*i, speedTouch);
		}
		else {
			safeVelocityMove(11+2*i, speedNoTouch);
			safeVelocityMove(12+2*i, speedNoTouch);
		}
	}

	//ring+pinky
	if (detect_touch_finger[detect_touch_finger.size()-1]){
		safeVelocityMove(15, speedTouch*3);
    }
    else {
	    safeVelocityMove(15, speedNoTouch*3);
    }  
}

/**
  * Set the velocity of the specified joint taking into account the min and max value specified for the encoder
  * and the max and min values specified for the velocities.
  */
void GraspThread::safeVelocityMove(int encIndex, double speed){
	if(encoders[encIndex] <= MOTOR_MIN[encIndex-11]){
		speed = (speed<0) ? 0 : speed;
	}
	else if(encoders[encIndex] >= MOTOR_MAX[encIndex-11]){
		speed = (speed>0) ? 0 : speed;
	}

	speed = (speed>VELOCITY_MAX) ? VELOCITY_MAX : speed;
	speed = (speed<VELOCITY_MIN) ? VELOCITY_MIN : speed;

	vel->velocityMove(encIndex, speed);
}

void GraspThread::setArmJointsPos(){
	// stop the current motion (if any)
	vel->stop();

	// set the arm in the starting position
	pos->positionMove(0 ,-20);
	pos->positionMove(1 , 30);
	pos->positionMove(2 ,  0);
	pos->positionMove(3 , 65);
	pos->positionMove(4 ,-30);
	pos->positionMove(5 ,  0);
	pos->positionMove(6 ,-20);
	pos->positionMove(7 , 15);
	pos->positionMove(8 , 90);
	pos->positionMove(9 , 20);
	pos->positionMove(10,  0);

	// set the fingers to the original position
	pos->positionMove(11, 5);
	pos->positionMove(12, 5);
	pos->positionMove(13, 5);
	pos->positionMove(14, 5);
	pos->positionMove(15, 5);

	Time::delay(3.0);
    fprintf(stderr, "Hand has been opened!\n");
}

bool GraspThread::setGraspCommand(GraspCommand command){
	graspCommSem.wait();	
	graspComm = command;
	graspCommSem.post();
	return true;
}

bool GraspThread::setTouchThreshold(int newTouchThreshold){
	if(newTouchThreshold>=0 && newTouchThreshold<MAX_SKIN){
		touchThresholdSem.wait();
		touch_threshold = newTouchThreshold;
		touchThresholdSem.post();
		return true;
	}
	return false;
}

bool GraspThread::setPercentile(vector<float> newPercentile){
	// check the vector size
	if(newPercentile.size() < SKIN_DIM)
		return false;

	//check the percentile values (not negative AND not greater than or equal to 255)
	for(int i=0; i<SKIN_DIM; i++){
		if(newPercentile[i]<0 || newPercentile[i]>=MAX_SKIN)
			return false;
	}

	// update the percentile
	percentileSem.wait();
	for(int i=0; i<SKIN_DIM; i++){
		percentile[i] = newPercentile[i];
	}
	percentileSem.post();

	return true;
}

bool GraspThread::setKP(float newKP){
	if(newKP<0 || newKP>MAX_KP)
		return false;

	controllerSem.wait();
	K_P.assign(K_P.size(), newKP);
	controllerSem.post();
	return true;
}

bool GraspThread::setKD(float newKD){
	if(newKD<0 || newKD>MAX_KD)
		return false;

	controllerSem.wait();
	K_D.assign(K_D.size(), newKD);
	controllerSem.post();
	return true;
}

bool GraspThread::setKI(float newKI){
	if(newKI<0 || newKI>MAX_KI)
		return false;

	controllerSem.wait();
	K_I.assign(K_I.size(), newKI);
	controllerSem.post();
	return true;
}

bool GraspThread::setKP(unsigned int index, float newKP){
	if(newKP<0 || newKP>MAX_KP || index>=K_P.size())
		return false;

	controllerSem.wait();
	K_P[index] = newKP;
	controllerSem.post();
	return true;
}

bool GraspThread::setKD(unsigned int index, float newKD){
	if(newKD<0 || newKD>MAX_KD || index>=K_D.size())
		return false;

	controllerSem.wait();
	K_D[index] = newKD;
	controllerSem.post();
	return true;
}

bool GraspThread::setKI(unsigned int index, float newKI){
	if(newKI<0 || newKI>MAX_KI || index>=K_I.size())
		return false;

	controllerSem.wait();
	K_I[index] = newKI;
	controllerSem.post();
	return true;
}

bool GraspThread::setTouchDesired(float newTouchDesired){
	if(newTouchDesired<0 || newTouchDesired>MAX_SKIN)
		return false;

	controllerSem.wait();
	touch_desired = newTouchDesired;
	controllerSem.post();
	return true;
}
