// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Vishwanathan Mohan
  * email: Vishwanathan Mohan@iit.it
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

#ifndef _OBSERVER_THREAD_H_
#define _OBSERVER_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <math.h>

#define COMMAND_VOCAB_REQ          VOCAB3('R','E','Q') //with episodic mem module
#define COMMAND_VOCAB_ACK          VOCAB3('A','C','K')
#define COMMAND_VOCAB_REACH        VOCAB3('R','E','A') //with body schema module
#define COMMAND_VOCAB_FIND         VOCAB4('F','I','N','D')  //with OPC module
#define COMMAND_VOCAB_GRAZP        VOCAB5('G','R','A','Z') //with KCL grasper

//shared vocabulary with user interface module
#define COMMAND_VOCAB_XPLOR          VOCAB4('X','P','L','R')
#define COMMAND_VOCAB_LEARNACT       VOCAB8('L','E','A','R','N','A','C','T')
#define COMMAND_VOCAB_LEARNOBJ       VOCAB8('L','E','A','R','N','O','B','J')
#define COMMAND_VOCAB_STACK          VOCAB5('S','T','A','C','K')
#define COMMAND_VOCAB_PUSH           VOCAB4('P','U','S','H') 
#define COMMAND_VOCAB_PICK           VOCAB4('P','I','C','K') 
#define COMMAND_VOCAB_WHERE          VOCAB5('W','H','E','R','E')
#define COMMAND_VOCAB_HELP           VOCAB4('H','E','L','P') 


class ObserverThread : public yarp::os::Thread {
private:
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
        
   	yarp::os::BufferedPort<yarp::os::Bottle > PlanEx;     // output port to command the robot
	yarp::os::BufferedPort<yarp::os::Bottle > QueryEPIM;      // input port to receive 3d information
	yarp::os::BufferedPort<yarp::os::Bottle > AknowEpim;     // output port to command the robot
    
	yarp::os::Port Planxx;
	yarp::os::RpcClient EpimCtrlPort; //connection to the server : Episodic memory and learning
	yarp::os::RpcClient OPCCtrlPort; //connection to the server Object properties collector
	yarp::os::RpcClient BodySchemaCtrlPort; //connection to the server PMP.BS
	yarp::os::RpcClient GraspPort; //connection to the server PMP.BS
	yarp::os::RpcServer UserServices;  //Connects to external clients: User

	
	std::string name;           // rootname of all the ports opened by this thread

	int KeySend;
	int state;                  //state that represents the action to perform
	int Strata[1000];
	int GiD; //this can be transformed into a shared vocabulary between user and observer
	double NumberofObs, NumberofObsE;
	int SeqAc[6], SeqAcP[10],numcu,numcy,nummu,PastPlan[10], Replan,numberpast;
	int SeqAcXl[2];
	int PickMicro, PlaceMicro, NPiCs,findsuccess,findsuccess2,PPiCo;
	double PlaceMap[10][18]; //col-shap-x-y-z-constraint
	double ObjIDEE[10], ObjIDEEEpim[10], XPosition[3];
	double NumObjectsinScene;
	int GetObjIDs[2], largeness,cannotfindXLoc,CannotFind,Cumulate;
	double PMPresp[12],cannotfindX,NumCubID[3],NumCylID[2],NumMushID[2];
	double XlatTrackP[10],XlatTrackPl[10],StaticLoc[3];
	int PtOfReplan;


public:
    /**
    * constructor default
    */
    ObserverThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    ObserverThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~ObserverThread();

    /**
    *  initialises the thread
    */
    bool threadInit();

    /**
    *  correctly releases the thread
    */
    void threadRelease();

    /**
    *  active part of the thread
    */
    void run(); 

    /**
    *  on stopping of the thread
    */
    void onStop();

    /*
    * function that sets the rootname of all the ports that are going to be created by the thread
    * @param str rootnma
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /*
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);

	void Interpret();  //plan interpreter

	double RefreshPlacemap(); // speak with vision

	double PrimBodySchema(int PMPGoalCode,int OIDinPM,int PIdentifier, int ObjectIDPMP);

	int PrimGrasp(int GraspReq );

	int PickandPlace(int pick, int place, int seqNumber);

	double PrimSearch(int obj1, int obj2, int goalidentity);

	void Xlator();

	void Mergence();

	void InvXlator(int pi, int pl);

	/*
    * function that sets the inputPort name
    */
//    void setColorPath(std::string inp) { colorMapPath = inp; };

       /*
    * function that sets the model path
    */
  //  void setModelPath(std::string inp) { modelPath = inp; };

    	
};

#endif  //_OBSERVER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

