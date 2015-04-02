// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Eric Sauser
 * email:   eric.sauser@a3.epfl.ch
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

/**
 *
\defgroup icub_ResetSkinPart ResetSkinPart
@ingroup icub_lasaBox_tools

Utility for reseting the skins. (Remove drift by just zeroing them)

\section intro_sec Description

This is just an executable that runs, connects to skin drivers, reset the specified skin parts and exits immediately (count 1 sec per part)

\endverbatim

\section dependencies_sec Dependencies

- YARP

\section parameters_sec Parameters

\verbatim
part1 [partN]* :    A list of skin parts to be reset
\endverbatim

\section portsc_sec Ports Created

Input ports:


Output ports:
  

\section in_files_sec Input Data Files

None

\section out_data_sec Output Data Files

None

\section conf_file_sec Configuration Files

None

\section tested_os_sec Tested OS

Linux

\section example_sec Example Instantiation of the Module

\verbatim
ResetSkinPart /icub/skin/right_arm /icub/skin/left_arm
\endverbatim

\author Eric Sauser

Copyright (C) 2011 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at $ICUB_ROOT/contrib/src/lasaBox/src/tools/ResetSkinPart
**/


#include <iostream>
#include <string>
#include <unistd.h>

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
using namespace yarp::os;
using namespace yarp::dev;

#include <iostream>
using namespace std;
#include <math.h>

int main(int argc, char *argv[])
{
    Network yarp;
    if(!yarp.checkNetwork())
        return 0;
      
    if(argc<=1){
        cerr << "Usage: ResetSkinPart portname [portname]*"<<endl;
        cerr << "  Example: ResetSkinPart /icub/skin/right_arm"<<endl;
        cerr <<endl;
        return 0;
    }
      
    IAnalogSensor *tactileSensor;	
    PolyDriver* tactileSensorDevice;

    

    for(int i=0;i<argc-1;i++){
        cerr << "Reseting part:"<< argv[i+1] <<endl;
    
	    Property options;
	    options.put("robot",  "icub");
	    options.put("part",   "skin_part");          //skin part that you want to control
	    options.put("local",  "/resetSkinPart");
	    options.put("remote",  argv[i+1]);
	    options.put("device", "analogsensorclient");	//important! Its different from remote_controlboard that you use to control motors!
	 
	    // create a new device driver
	    tactileSensorDevice = new PolyDriver(options);
        if(tactileSensorDevice!=NULL){
	        if (!tactileSensorDevice->isValid()){
		        printf("Device not available.  Here are the known devices:\n");
		        printf("%s", Drivers::factory().toString().c_str());
		        continue;
	        }
	        // open the sensor interface	
	        bool ok = tactileSensorDevice->view(tactileSensor);
	        if (!ok) {
		        printf("Problems acquiring interfaces\n");
	        }
	        usleep(200*1000);
	        int dim = tactileSensor->getChannels();
	        fprintf(stderr,"This part has %d sensors\n",dim);
	        if(dim==0){
		        fprintf(stderr, "Error while reading the number of channels of the tactile sensor device\n");
	        }
	        tactileSensor->calibrateSensor();	
            tactileSensorDevice->close();
            delete tactileSensorDevice;
        }
    }
		
    return 0;
}



