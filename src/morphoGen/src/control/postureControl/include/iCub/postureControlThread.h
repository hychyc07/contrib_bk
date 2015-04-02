// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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

/**
 * @file postureControlThread.h
 * @brief Definition of a thread that receives an RGN image from input port and sends it to the output port.
 */


#ifndef _POSTURE_CONTROL_THREAD_H_
#define _POSTURE_CONTROL_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <iostream>
#include <fstream>
#include <time.h>




class postureControlThread : public yarp::os::Thread {
private:
    int jntsRightArm;              // number of jointRight leg
    
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    
    yarp::sig::Vector encodersRightArm;      // encoders position

    yarp::dev::PolyDriver        *robotDeviceRightArm;  // device right arm
    yarp::dev::PolyDriver        *robotDeviceLeftArm;   // device left arm
    
    yarp::dev::IPositionControl  *posRightArm;          // position control of the robot RightArm
    yarp::dev::IEncoders         *encsRightArm;         // encoders readings from the robot RightArm
    yarp::dev::IControlMode      *ictrlRightArm;        // sets the modality of the control RightArm
    yarp::dev::IImpedanceControl *iimpRightArm;         // impedence control of the robot RightArm
    yarp::dev::ITorqueControl    *itrqRightArm;         // torque control of the robot RightArm

    yarp::dev::IPositionControl  *posLeftArm;          // position control of the robot RightArm
    yarp::dev::IEncoders         *encsLeftArm;         // encoders readings from the robot RightArm
    yarp::dev::IControlMode      *ictrlLeftArm;        // sets the modality of the control RightArm
    yarp::dev::IImpedanceControl *iimpLeftArm;         // impedence control of the robot RightArm
    yarp::dev::ITorqueControl    *itrqLeftArm;         // torque control of the robot RightArm

    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputCallbackPort;
    yarp::os::BufferedPort<yarp::os::Bottle > inputRightArm;                             //input port for right arm
    yarp::os::BufferedPort<yarp::os::Bottle > inputLeftArm;                              //input port for right arm
    
    std::string name;                                                                    // rootname of all the ports opened by this thread
    
    static const double armMin[];
    static const double armMax[];
    
public:
    /**
    * constructor default
    */
    postureControlThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    postureControlThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~postureControlThread();

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

    /*
    * function that sets the robotName
    */
    void setRobot(std::string robotName){robot = robotName;};

    /*
    * function that initialises the controller of all the bodyparts
    */
    bool initController();

    /*
    * function that checks the validity of the content of the arm bottle
    * @param b bottle that has arm position
    */
    bool checkArm(yarp::os::Bottle* b);

    /**
     * function that parses the bottle received and generate the vector of the control pos
     * @param b received bottle
     * @param dim dimension of the control vector
     * @result vector of the control of the body part
     */
    yarp::sig::Vector parseBottle(yarp::os::Bottle* b, int dim);


};



#endif  //_POSTURE_CONTROL_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

