/* DOUBLE TOUCH v. 0.2
 * Copyright (C) 2012 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Alessandro Roncone
 * email:   alessandro.roncone@iit.it
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

#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iostream>
#include <string.h> 

#include "iCubDblTchThrd.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

/*
yarp connect /skinManager/skin_events:o /doubleTouch/contactReader/contacts:i
*/
/*
yarp connect /wholeBodyDynamics/contacts:o /doubleTouch/contactReader/contacts:i
*/

/*****************************************************************************/

class doubleTouch: public RFModule {

private:
  doubleTouchThread *dblTchThrd;

public:
  doubleTouch() {
    dblTchThrd=0;
  }

  bool configure(ResourceFinder &rf) {
    string robot="icubSim";
    string name="doubleTouch";
    string type="customLimb"; // type is here for future expansions
    
    int verbosity = 0;      // verbosity (check doubleTouchThread.h)
    int dtt_rate = 100;     // rate of the doubleTouchThread

    //******************************************************
    //********************** CONFIGS ***********************

    //******************* NAME ******************
    if (rf.check("name")) {
      name = rf.find("name").asString();
      cout << "Module name set to "<<name<<endl;  
    }
    else cout << "Module name set to default, i.e. " << name << endl;
    setName(name.c_str());

    //******************* ROBOT ******************
    if (rf.check("robot")) {
      robot = rf.find("robot").asString();
      cout << "Robot is: " << robot << endl;
    }
    else cout << "Could not find robot option in the config file; using " << robot << " as default\n";

    //******************* VERBOSE ******************
    if (rf.check("verbosity")) {
      verbosity = rf.find("verbosity").asInt();
      cout << "doubleTouchThread verbosity set to " << verbosity << endl;
    }
    else cout << "Could not find verbosity option in the config file; using "<< verbosity <<" as default\n";
    
    //****************** dtt_rate ******************
    if (rf.check("dtt_rate")) {
      dtt_rate = rf.find("dtt_rate").asInt();
      cout << "doubleTouchThread rateThread working at " << dtt_rate << " ms\n";
    }
    else cout << "Could not find dtt_rate in the config file; using "<< dtt_rate <<" ms as default\n";

    //******************************************************
    //*********************** THREADS **********************
    
    if( dtt_rate != 0 ) {  

      dblTchThrd = new doubleTouchThread(dtt_rate, name, robot, verbosity, type);

      dblTchThrd -> start();
      bool strt = 1;
      if (!strt) {
            delete dblTchThrd;
            dblTchThrd = 0;
            cout << "\nERROR!!! doubleTouchThread wasn't instantiated!!\n";
            return false;
        }
      cout << "DOUBLE TOUCH: dblTchThrd thread istantiated...\n";
    }
    else {
      dblTchThrd = 0;
      cout << "\nERROR!!! doubleTouchThread wasn't instantiated!!\n";
      return false;
    }

    return true;
  }

  bool close() {
    cout << "DOUBLE TOUCH: Stopping threads.."<<endl;
    if (dblTchThrd) { dblTchThrd->stop(); delete dblTchThrd; dblTchThrd=0; }

    return true;
  }

  double getPeriod()  { return 1.0; }
  bool updateModule() { return true; }
};

//******************************************************
//******************************************************

int main(int argc, char * argv[]) {
  
  YARP_REGISTER_DEVICES(icubmod)

  ResourceFinder rf;
  rf.setVerbose(true);
  // rf.setDefaultContext("$ICUB_ROOT/contrib/src/doubleTouch");
  rf.setDefaultConfigFile("doubleTouch.ini");
  rf.configure("ICUB_ROOT",argc,argv);

  if (rf.check("help")) {    
    cout << "Options:" << endl << endl;
    // cout << "   --context \tcontext: where to find the called resource (referred to $ICUB_ROOT/app):\n";
    // cout << "             \t         default $ICUB_ROOT/contrib/src/doubleTouch." << endl;
    cout << "   --from     \tfrom: the name of the file.ini to be used for setting." << endl;
    cout << "   --robot    \trobot: the name of the robot. Default icub." << endl;
    cout << "   --dtt_rate \trate: the period used by the doubleTouchThread thread. Default 100ms." << endl;
    cout << "   --verbosity\tint: verbosity level (default 0)." << endl;
    return 0;
  }

  Network yarp;
  if (!yarp.checkNetwork()) return -1;

  doubleTouch dblTch;
  return dblTch.runModule(rf);
}
// empty line to make gcc happy