/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
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
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/all.h>

#include <gsl/gsl_math.h>

#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdarg.h>

#include "iCubDblTchSlv.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::skinDynLib;

using namespace std;

/****************************************************************************
* This thread detects a touched taxel on the skin (through readings from the
* skinContactList port), and it moves the "controlateral" limb toward
* the affected taxel.
*****************************************************************************/

class doubleTouchThread: public RateThread {
protected:

  /***************************************************************************/
  // EXTERNAL VARIABLES: change them from command line or through .ini file
  // Flag that manages verbosity (v=1 -> more text printed out):
  int verbosity;
  // Name of the module (to change port names accordingly):
  string name;
  // Name of the robot (to address the module toward icub or icubSim):
  string robot;
  
  /***************************************************************************/
  // CONTACT-RELATED VARIABLES:
  Vector cntctPosLink;     // Position in i-th link RF
  Vector cntctPosWRF;      // Position in WRF
  Vector cntctPosEE;       // Position in end-eff RF
  Vector cntctNormDir;     // Normal Direction
  Matrix cntctH0;          // RT matrix located in the contact with the 
                           // x axis normal to the cover
  int cntctLinkNum;        // Link number
  double cntctPressure;    // Pressure
  string cntctArm;         // Affected Arm
  skinContact cntctSkin;   // SkinContact
  
  /***************************************************************************/
  // INTERNAL VARIABLES:
  // Flag to know in which step the thread is
  int step;

  // Upper arm and forearm indexes for the SkinPart enum
  // (eventually, there will be easy to change the skinparts to monitor):
  int ual, fal, uar, far;

  // Ports:
  BufferedPort<iCub::skinDynLib::skinContactList> *cntctRdr;
  
  // Driver for "classical" interfaces
  PolyDriver       ddR; // right arm device driver
  PolyDriver       ddL; // left arm  device driver
  PolyDriver       ddG; // gaze controller  driver

  // "Classical" interfaces - RIGHT ARM
  IEncoders         *iencsR;
  IPositionControl  *iposR;
  IControlMode      *ictrlR;
  IControlLimits    *ilimR;
  Vector            *encsR;
  int jntsR;
  // "Classical" interfaces - LEFT ARM
  IEncoders         *iencsL;
  IPositionControl  *iposL;
  IControlMode      *ictrlL;
  IControlLimits    *ilimL;
  Vector            *encsL;
  int jntsL;
  // Gaze controller interface
  IGazeControl       *igaze;
  int contextGaze;

  // IPOPT STUFF
  iCubDoubleTouch_Problem   *probl;  // problem
  iCubDoubleTouch_Variables *g;      // guess
  iCubDoubleTouch_Variables *s0;     // solution - waypoint
  iCubDoubleTouch_Variables *s1;     // solution
  iCubDoubleTouch_Solver    *slv;    // solver

  // CUSTOM LIMB (for testing the achievement of the task)
  iCubCustomLimb testLimb;

  void closePort(Contactable *_port);

  int printMessage(const int level, const char *format, ...) const;

  /**
  * Checks if the motion has finished. To be implemented in future releases
  * (the old version is not available any more because of the changes)
  */
  bool checkMotionDone() { return true; };

  /**
  * Moves arms to starting position:
  */
  void goToRest();

  /**
  * Uses the contralateral hand to touch the affected taxel
  * @param s is the type of movement (either "standard" or "waypoint")
  */
  void goToTaxel()    { goToTaxel("standard"); };

  void goToTaxel(string s);

  /**
  * Reads the contact from either /skinManager/skin_events:o or
  * /wholeBodyDynamics/contacts:o , and handles the skinContacts
  */
  void detectContact(skinContactList *_sCL);

  /**
  * Locates the contact in World Reference Frame's coordinates
  * @param sc is the skinContact for which the location is computed
  */
  Vector locateContact(skinContact &sc);

  /**
  * Aligns joint bounds according to the actual limits of the robot
  */
  bool alignJointsBounds();

  /**
  * Finds the proper H0 for the limb
  * @param sc is the skinContact for which the H0 has to be computed
  */
  Matrix findH0(skinContact &sc);

  /**
  * Tests if the task is achieved, by reading joints encoders
  */
  void testAchievement();

  /**
  * Handles the gaze controller for each step of the module
  */
  void handleGaze();

  void delay(int sec);

public:
  // CONSTRUCTOR
  doubleTouchThread(int _dtt_rate, const string &_name,
                    const string &_robot, int _v, const string _type);
  // INIT
  virtual bool threadInit();
  // RUN
  virtual void run();
  // RELEASE
  virtual void threadRelease();
};

// PERSONAL NOTES (remember to remove them sooner or later!)
// iCub/main/src/tools/skinManagerGui
// iCub/main/src/modules/skinManager
// iCub/main/src/libraries/skinDynLib
//
// A skinContactList is represented as a list of lists
// where each list is a skinContact
// basically, it's a vector of skinContact
//
// A skinContact is a list of 8 elements that are:
// - a list of 4 int, i.e. contactId, bodyPart, linkNumber, skinPart
// - a list of 3 double, i.e. the CoP
// - a list of 3 double, i.e. the force
// - a list of 3 double, i.e. the moment
// - a list of 3 double, i.e. the geometric center
// - a list of 3 double, i.e. the normal direction
// - a list of N int, i.e. the active taxel ids
// - a double, i.e. the pressure
// 
// ((48725 4 4 5) (-0.017 0.062 -0.036) (0.476424 0.109944 0.611614)
// (0.0 0.0 0.0) (-0.017 0.062 -0.036) (-0.585 -0.135 -0.751) (134) 16.288001)

