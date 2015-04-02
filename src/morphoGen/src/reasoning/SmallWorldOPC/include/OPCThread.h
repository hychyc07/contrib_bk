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

#ifndef _OPC_THREAD_H_
#define _OPC_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <math.h>

#define COMMAND_VOCAB_FIND         VOCAB4('F','I','N','D')  //with OPC module

//shared vocabulary with Observer module


class OPCThread : public yarp::os::Thread {
private:
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
        
   	yarp::os::RpcServer OPCServer;  //Connects to external clients: User
	yarp::os::BufferedPort<yarp::os::Bottle > WorldSnap;   
	std::string name;           // rootname of all the ports opened by this thread

	double Eighteens[10][18]; //face coordinates
	int NumObject;
	int IOD[10];
	
public:
    /**
    * constructor default
    */
    OPCThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    OPCThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~OPCThread();

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
	void setInputPortName(std::string inpPrtName);

    std::string getName(const char* p);

    /*
    * function that sets the inputPort name
    */
    /*
    * function that sets the inputPort name
    */
//    void setColorPath(std::string inp) { colorMapPath = inp; };

       /*
    * function that sets the model path
    */
  //  void setModelPath(std::string inp) { modelPath = inp; };

    	
};

#endif  //_OPC_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

