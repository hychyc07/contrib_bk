/** @file crawlGeneratorModule.h Header file the crawlGeneratorModule class.
*
* Version information : 1.0
*
* Date 04/05/2009
*
*/
/*
* Copyright (C) 2009 Sarah Degallier, EPFL
* RobotCub Consortium, European Commission FP6 Project IST-004370
* email: sarah.degallier@epfl.ch
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

#ifndef crawlGeneratorThread__H
#define crawlGeneratorThread__H

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RFModule.h> 

#include <stdio.h>
#include <iostream>
#include <string>
#include "cpgs.h"
#include "crawlInvKin.h"

#define LIMIT_TOL 2.0 //conservative limit tolerance in degrees
#define MAX_FREQUENCY 1.5 //in Hz
#define MAX_TURN_ANGLE 0.52 //in radians 

#define VELOCITY_INDEX_OFFSET 1000

#define LEFT 1
#define RIGHT 2

#define ARM 1
#define LEG 2

/**
* The main crawling generator module. 
* This class is responsible for recovering the necessary parameters to properly 
* setup and run the periodic thread theThread, where the actual movement generation is done.
* @see GeneratorThread
*/
class GeneratorThread : public yarp::os::RateThread
{
 public:

	/**
    * Constructor of the GeneratorThread class.
    * Sets the value of the period member.
    */
    GeneratorThread(int period); 

    /**
    * Destructor of the GeneratorThread class.
    * Does nothing ?.
    */
    ~GeneratorThread(); //destructor

    /**
    * Runs the main loop.
    * Gets the encoders, integrates the system and sends commands to the robot.
    * @see Cpgs::get_dt()
    * @see Cpgs::integrate_step()
    * @see connectToOtherLimbs()
    * @see getEncoders()
    * @see getOtherLimbStatus()
    * @see getParameters()
    * @see sendFastJointCommand()
    */
    void run();

    /**
    * Does nothing.
    * @return true.
    */
    virtual bool threadInit();

    /**
    * Closes the thread.
    * Disconnects the ports, destroy the CPG and its parameters.
    * @see disconnectPorts()
    */
    virtual void threadRelease();

    /**
    * Inits the generator thread.
    * Sets up the polydriver of the controlled part.
    * Connects to the velocity control port to send commands
    * Connects to the manager port to receive parameters
    * Create and sets up the CPG.
    * @param s A Searchable structure containing the necessary parameters to setup the connections and the CPG.
    * @return true if the initialisation succeded with no error, false otherwise.
    * @see getEncoders()
    * @see Cpgs::Cpgs()
    * @see Cpgs::printInternalVariables()
    */
    //CTmodified: bool init(Searchable &s);
	bool init(yarp::os::ResourceFinder &rf); 

    /**
    * Reads the parameters coming from the manager and update the cpg.
    * @see run()
    */
    void getParameters();
    
 private:

	double period; /**< Period in seconds */
	bool go_to_init_position;

    std::string partName; /**< Name of the part */

    int nbDOFs; /**< Number of degrees of freedom of this part */
    int nbLIMBs; /**< Number of limbs coupled with this part */
    int *jointMapping; /**< Mapping of the joints and the states */
    double *y_cpgs; /**< Missing Description */
    double *states; /**< Missing Description */
	double *dstates; /**< Missing Description */
    double *previous_states; /**< Missing Description */
    double *encoders; /**< Missing Description */
    double *joint_limit_up; /**< Upper joint limit for each join of this part */
    double *joint_limit_down; /**< Lower joint limit for each join of this part */
    double *initPos; /**< Missing Description */
    double original_time; /**< Missing Description */
    double theoretical_time; /**< Missing Description */
    double lastBeat_time; /**< Missing Description */
	double amplit;/**< Missing Description */
    int previous_quadrant; /**< Uncomplete Description : used to calculate the new beat */
    int beat; /**< Missing Description */
	int side, limb;/**< Missing Description */

    Cpgs *myCpg; /**< The CPG of the controlled part */
    PolyDriver *ddPart; /**< The Polydriver of the controlled part */
    IEncoders *PartEncoders; /**< Missing Description */
    IKManager *myIK;/**< Missing Description */
    ITorqueControl *PartTorques;
	IPositionControl *PartPosition;
	IAmplifierControl *PartAmplifier;

    BufferedPort<Bottle> vcControl_port; /**< Port of the velocity controller */
    BufferedPort<Bottle> vcFastCommand_port; /**< The velocity controller FastCommand port. MISSING */
    BufferedPort<Bottle> parameters_port; /**< Port to receive parameters from the manager */
    BufferedPort<Bottle> check_motion_port; /**< Port to send the status of the motion */
    BufferedPort<Bottle> other_part_port[3]; /**< Port to connect to the other limbs */
    BufferedPort<Bottle> current_state_port; /**< Port to send the current status of the CPG */
    BufferedPort<Bottle> contact_port;/**< Missing Description */
	BufferedPort<Bottle> check_status_port; /**< Missing Description */

    bool other_part_connected[3]; /**< Specifies if the coupled part is connected */
    ConstString other_part_name[3]; /**< Names of the other limbs */
   
    bool current_action; /**< Missing Description */

    FILE *target_file; /**< Missing Description */
    FILE *parameters_file; /**< Missing Description */
    FILE *encoder_file; /**< Missing Description */
    FILE *feedback_file; /**< Missing Description */
    FILE *velocity_file; /**< Missing Description */
    FILE* torque_file;

	ConstString robot;/**< Name of the robot (ex : iCub, iCubSim...) */


	private:

	 /**
    * Sends commands to the velocity controller.
    * Sends the joint mappings and the states of the system.
    * @return true.
    * @see run()
    */
    bool sendFastJointCommand();

    /**
    * Checks that the joint limits have not been exceeded.
    * If joint limits have not been exceeded, puts the states back inside the joint limits.
    * @see sendFastJointCommand()
    */
    bool checkJointLimits();

    /**
    * Missing Description.
    * Detailed Description of the function, what it does.
    * @return desctiption of the return arguments, their types, and what they mean.
    * @see GeneratorThread()
    */
    bool getEncoders();
    
    bool getTorque();

    /**
    * Gets the status of the other limbs.
    * @return true.
    * @see run()
    */
    bool getOtherLimbStatus();

    /**
    * Connects the controlled part to the other coupled parts.
    * Opens the ports and sets up connection between this part and the coupled ones. Sets up the CPG external coupling.
    * @see run()
    */
    void connectToOtherLimbs();

    /**
    * Disconnects the ports used by the thread.
    * Disconnects the velocity controller ports and the manager ports.
    * @see threadRelease()
    */
    void disconnectPorts();

	/**
    * MISSING INFORMATION
    * @see truc()
    */
    void getContactInformation();

	/**
    * MISSING INFORMATION
    * @see truc()
    */
    void sendStatusForManager();
};

#endif //crawlGeneratorThread__H


