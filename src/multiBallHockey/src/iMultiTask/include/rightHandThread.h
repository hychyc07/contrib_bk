#ifndef RHANDTHREAD_H
#define RHANDTHREAD_H

#include "taskThread.h"
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>


#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <yarp/os/Network.h>

#include <yarp/sig/Image.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>


#include <yarp/dev/ControlBoardInterfaces.h>

//#include <yarp/dev/GazeControl.h>

class RightHandThread : public TaskThread
{
public:
  // IGazeControl  *igaze;
    RightHandThread(int period);
   /** main() function of the thread. */
   void run();
   /** Signal the thread to exit the next time it is woken up. */
   void stop();
   bool threadInit();
   void threadRelease();

// left arm
	PolyDriver         leftclient;
    ICartesianControl *leftarm;
    Vector leftxd;
    Vector leftod;
  int startup_context_left_init;
    int startup_context_left_dof10;
    int startup_context_left_dof9;
	Vector homeLeft;
	Matrix oLeftHome;
	Matrix oLeft;
	IPositionControl *posLeft; // pos
    	IEncoders *encsLeft; // encs
	PolyDriver leftArmPosDevice; // robotDevice
	 Vector encodersLeft;
    	Vector commandLeft;
	int njLeft;

	void  initLeftArmPosControl();

// right arm
    PolyDriver         client;
    ICartesianControl *arm;
    Vector xd;
    Vector od;
    int startup_context_right_init;
    int startup_context_right_dof10;
    int startup_context_right_dof9;
 void limitTorsoPitch();
	Matrix oRight;
  Matrix oRightHome;
	IPositionControl *posRight; 
    	IEncoders *encsRight; 
	PolyDriver rightArmPosDevice; 
	 Vector encodersRight;
    	Vector commandRight;
	int njRight;

	void  initRightArmPosControl();

	// torso position control
	IPositionControl *posTorso; 
    	IEncoders *encsTorso; 
	PolyDriver torsoPosDevice; 
	 Vector encodersTorso;
    	Vector commandTorso;
	int njTorso;

	void  initTorsoPosControl();

	bool resetTorso();
	// 
	bool movedRight;
	bool movedLeft;
	bool resettedTorso;
	Vector prevHitPos;

	bool firstPosSet;
	int resetCounter;

 // right arm home position
 Vector homeRight;

    yarp::sig::Vector homeworld;

	void setNextTaskPosition(Vector pos);
	bool runNextTask( );
	bool runNextTaskRight();
	bool runNextTaskLeft();

	void setLeftHighWeight();
	void setRightHighWeight();
	bool reset();
	bool resetRightJoints();
	bool resetLeftJoints();
	bool resetRightCartesian();
	bool resetLeftCartesian();
	bool terminateCurrentTask( );	

};

#endif
