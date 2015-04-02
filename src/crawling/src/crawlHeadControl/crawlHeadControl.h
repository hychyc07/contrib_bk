/** @file ReachManager.h Header file the ReachManager class.
*
* Version information : 1.0
*
* Date 04/05/2009
*
*/
/*
* Copyright (C) 2009 Gay Sebastien, EPFL
* RobotCub Consortium, European Commission FP6 Project IST-004370
* email: sebastien.gay@epfl.ch
* website: www.robotcub.org
*
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2
* or any later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*/

#ifndef CRAWLHEADCONTROL__H
#define CRAWLHEADCONTROL__H

#include <yarp/os/all.h>
using namespace yarp::os;

#include <yarp/sig/Vector.h>
using namespace yarp::sig;

#include <yarp/dev/PolyDriver.h>
using namespace yarp::dev;

#include <math.h>
#include <map>
#include <string>
using namespace std;



#define L 0.36
#define MODULE_PERIOD 0.1

/**
* A simple class handling the control of the head when approaching a "good" object when crawling.
* The module gets the position of one or more objects and decides which one to focus on (the nearest good one).
* I makes sure that the object stays in the field of view of the robot.
* It outputs the joint configuration of the head to achieve this.
*/


/* CT(15/3/2011): Changed from Module to RFModule*/
class crawlHeadControl :
    public yarp::os::RFModule
{
    std::string moduleName;
    std::string robotName; 
    std::string robotPortName;  
    std::string inputPortName;
    std::string outputPortName; 
	int headControlCommandCode;
	double headControlModeDist;
	int objectID;

	
/*class crawlHeadControl :
    public Module
{*/

public:

	/* configure all the module parameters and return true if successful */
	
	/**
    * Opens the module.
	* Reads the parameters from the config file.
	* Opens the input and output ports
	* Initializes the polydriver to communicate with the head of the robot.
    */
	bool configure(yarp::os::ResourceFinder &rf); 
	/**
    * Constructor of the MultiMarkerTracker class.
    * Does nothing.
    */
    crawlHeadControl(void);

	/**
    * Destructor of the MultiMarkerTracker class.
    * Does nothing.
    */
    virtual ~crawlHeadControl(void);

	/**
    * Main module loop,
    * Gets position of object on the input port.
	* Computes the nearest "good" one.
	* Computes the head angles to keep the object in the field of view of the robot.
    */
    virtual bool updateModule();

	/**
    * Returns the period of the module.
    */
	virtual double getPeriod();

	/**
    * Closes the module.
	* Closes the input and output ports.
	* Closes the polydriver of the head.
    */
	virtual bool close();

    virtual bool interruptModule();



protected:
    BufferedPort<Bottle> inPort;
    BufferedPort<Bottle> outPort;
	// CT(15/3/2011): I guess is not needed anymore:
	//map<string, Value *> parameters;
	PolyDriver *polydriver;
	int nbJoints;

private:
	// CT(15/3/2011): I guess is not needed anymore:
	//Value GetValueFromConfig(Searchable& config, string valueName);
	void InitPolydriver();
	map<string, double> GetHeadAngles(void);



};

#endif //CRAWLHEADCONTROL__H