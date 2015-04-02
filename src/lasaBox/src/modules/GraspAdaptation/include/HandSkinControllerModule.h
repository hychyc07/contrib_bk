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
\defgroup icub_GraspAdaptation GraspAdaptation
@ingroup icub_lasaBox_modules



Demo module for the Grasp Adaptation learning technique developed at LASA-EPFL.

\section section_abstract Abstract

The reference and abstract of the research paper describing this work can be found 
\ref icub_lasaBox_GraspAdaptation_Abstract "here".




\section intro_sec Description

Documentation wise, this page will strictly describe the basics of the module.
For documentation as to how to run a demo and about the data format of generated files,<br/>
see the \ref icub_lasaBox_GraspAdaptation_MainPage pages.


This software comes in two parts: the GraspAdaptation module and some octave/matlab scripts. The module
controls the robot, prepares data, save them, and execute models. The octave scripts are here
to learn the models from the data gathered by the module.

The octave/matlab scripts can be found 
in: $ICUB_ROOT/contrib/src/lasaBox/src/modules/GraspAdaptation/octave






For sake of completeness, here's the full command list available to the module:
\verbatim
*** State and behavior control commands ***********************************
idle         : Idle mode (No commands are sent to the motors at all)
rest         : Go to a predefined rest position mode 
rec          : Record mode
               - keep the last position fron Idle mode
               - use low gains            
run          : Run a loaded  model
recr         : Run a model and and set the recording mode 
               - use the model to help manipulating and refining
               - use low gains            
rep          : Replay data
reps         : Replay data and simulate skin input
*** Data control commands *************************************************
lrep <name>  : Load replay data
lgmm <name>  : Load GMM data
<RETURN>     : Switch on/off data output on port
dnam <name>  : Use <name> for auto saving data when output streaming is on
asav         : Toggle auto save during output streaming
test         : toggle test data output mode
*** Model behavior control  commands **************************************
mism         : toggle reliability measure check
gon          : turn on gradient ascent mode
goff         : turn off gradient ascent mode
gain <double>: set a global gain to the controllers: useful to compensate 
               for the untensioning of the hand cables
*** Miscelanous commands **************************************************
scal         : Reset skin calibration (nothing shall be in the robot's hand)
don          : Debug message On
doff         : Debug message Off
\endverbatim



\section dependencies_sec Dependencies

- YARP
- iCubInterface shall be running

\section parameters_sec Parameters

\verbatim
--name <string>:    the module base name for port creation (GraspAdaptation by default)
--period <double>:  control loop period (0.02 sec by default)
--robot <string>:   the robot name to connect to
--part <string>:    the icub part to be controlled (left or right) (mandatory parameter)
\endverbatim


\section Input ports:

- /moduleName/partName/rpc
- /moduleName/partName/fingertip:i : port to which skin data shoudl be streamed

\section Output ports:

- /moduleName/partName/output:o : streams output data (see \ref icub_lasaBox_GraspAdaptation_FileFormat)

\section in_files_sec Input Data Files

See below

\section out_data_sec Output Data Files

See below

\section conf_file_sec Configuration Files

None

\section tested_os_sec Tested OS

Linux


\section example_sec Example Instantiation of the Module

\verbatim
GraspAdaptation --robot icub --part right --path $ICUB_ROOT/contrib/src/lasaBox/data/GraspAdaptation
\endverbatim

\author Eric Sauser

Copyright (C) 2011 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at $ICUB_ROOT/contrib/src/lasaBox/src/modules/GraspAdaptation
**/



/**
* \page icub_lasaBox_GraspAdaptation_MainPage LASA's Grasp Adaptation extended documentation
*

<i>Note:</i> This page is related to the \ref icub_GraspAdaptation "Grasp Adaptation" module.

\subpage icub_lasaBox_GraspAdaptation_Abstract

\subpage icub_lasaBox_GraspAdaptation_RunningDemo

\subpage icub_lasaBox_GraspAdaptation_FileFormat


*/


/**
* \page icub_lasaBox_GraspAdaptation_Abstract Abstract
*

A paper describing this work has been  submitted to Robotics and Autonomous Systems.

"Iterative Learning of Grasp Adaptation through Human Corrections", by 
Eric L. Sauser, Brenna D. Argall, Giorgio Metta and Aude G. Billard


<b>Abstract:</b>

In the context of object interaction and manipulation, one hallmark of a
robust grasp is its ability to comply with external perturbations applied to the
grasped object while still maintaining the grasp. In this work we introduce
an approach for grasp adaptation which learns a statistical model to adapt
hand posture solely based on the perceived contact between the object and
fingers. Using a multi-step learning procedure, the model dataset is built by
first demonstrating an initial hand posture, which is then physically corrected
by a human teacher pressing on the fingertips, exploiting compliance in the
robot hand. The learner then replays the resulting sequence of hand postures,
to generate a dataset of posture-contact pairs that are not influenced by the
touch of the teacher. A key feature of this work is that the learned model may
be further refined by repeating the correction-replay steps. Alternatively, the
model may be reused in the development of new models, characterized by the
contact signatures of a dfferent object. Our approach is empirically validated
on the iCub robot. We demonstrate grasp adaptation in response to changes
in contact, and show successful model reuse and improved adaptation with
additional rounds of model refinement.

<i>Note:</i> This page refers to the \ref icub_GraspAdaptation module.

*/

/**
* \page icub_lasaBox_GraspAdaptation_FileFormat Port output and file format 
*
<i>Note:</i> This page refers to the \ref icub_GraspAdaptation module.

<b> Ourput port and file format: </b>

When a model is running, the output port of the module sends a vector of number.
If the autosave-mode is on, this file is also written to the disk.

In non testing mode, there is 87 values:
    - The 9 current joint angles from the iCub hands (9 values (we actually only use the 2nd to the 8th joint))
    - The 3 averaged sensor pressure for each finger (3 values, thumb index, middle)
    - 6 values that are still here for historical reasons (normal angle of each finger when projected on perpendicular plane)
    - The 3 preceived 3D normal directions (9 values, thumb index, middle) 
    - All the 5x12 individual touch sensor values (60 values, index, middle ring, pinky, thumb)

In testing mode, you add 11 values for a total of 98 values:
    - The position error for each of the 7 joints (actual-desired) (7 values)
    - The 3 pressure error (actual-desired) (3 values)
    - The model's position confidence value (1 value)

*/




/**
* \page icub_lasaBox_GraspAdaptation_RunningDemo Running a demo
*

<i>Note:</i> This page refers to the \ref icub_GraspAdaptation module.

Here's an example of how to run the module and the learning procedure. 

First, let's assume that the lasaBox's binaries are in your path.
Then, run the module and set the data path containing the examples.
\verbatim
GraspAdaptation --robot icub --part right --path $ICUB_ROOT/contrib/src/lasaBox/data/GraspAdaptation
\endverbatim

Initial setup:
\verbatim
scal                // Reset the fingertip skin sensors (nothing should be in the robot's hand)
rest                // Prepare the robot's hand nicely
asav                // Set recording to be saved into files
lgmm demo_init_rep  // Load a dummy adaptation model (size of a gray coke-light can)
                    // 'demo_init_rep' should be located in your folder
                    // $ICUB_ROOT/contrib/src/lasaBox/data/GraspAdaptation 
run                 // Run the model and the robot's execution
\endverbatim

Recording a refinement:
\verbatim
dnam demo0        // Recording's filename
recr              // Run with low gain mode
<return>          // Start recording 
...               // You shall manipulate the robot's hand now for some time
<return>          // Stop recording (and save a data file in the path provided at startup) 
\endverbatim

Preparing the replay: In an "octave or matlab" terminal: (You may also go into the the 
octave folder to get direct access to these scripts or set your octave path)
\verbatim
// Do this two lines once
global GRASP_ADAPTATION_PATH    // Set the octave scripts path variable
GRASP_ADAPTATION_PATH = [getenv('ICUB_ROOT') '/contrib/src/lasaBox/data/GraspAdaptation'];
                                // Set the default path for the demo          
                                
ProcessRawData('demo0',0);      // produces a file demo0_p.txt (preprocess the data) 
                                // the 0 is for the mode to process demonstration data 
\endverbatim

Back in the module: replay
\verbatim
lrep demo0         // load the generated replay file 
dnam demo0_rep     // set a new filename for final replay data (please use the _rep suffix for 
                                               avoiding bugs with the octave scripts and others)
rep                // starts replaying  
<return>           // start recording 
...                // Durig this time, you may check that the can is not falling or even hold it
                   //  to help the iCub (actually a good habit :) )
<return>           // stop recording sna save the replay data file
\endverbatim


Back in octave/matlab: learning
\verbatim
ProcessRawData('demo0_rep',1);   // preprocess replayed data 
                                 // the 1 is for processing replay data 
LearnModel('demo0_rep');         // learn a model from processed replay data 
TestGMM('demo0_rep');            // (optional) look at some nice graphs 
\endverbatim

Back in the module: run the new model
\verbatim
lgmm demo0_rep                   // load the new gmm 
run                              // run it 
\endverbatim

Et voilà!


Finally, there's also some demo models in the repository.
In the same model path, you have
\verbatim
lgmm bcan10_rep                   // load a model for a big can (diet-coke type of can)
run                               // run it 
\endverbatim
\verbatim
lgmm can10_rep                    // load a model for a thin can (coke-zero type of can)
                                  // (Sorry for the unecessary advertisement...)
run                               // run it 
\endverbatim

*/

#ifndef HandSkinControllerMODULE_H_
#define HandSkinControllerMODULE_H_

#include <yarp/os/Module.h>
#include <yarp/dev/PolyDriver.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

#include "HandSkinControllerThread.h"

class HandSkinControllerModule: public Module
{
private:
    Property                    mParams;
    double                      mPeriod;
    bool                        bIsReady;

    BufferedPort<Bottle>        mControlPort;

    PolyDriver*                 mDriver;

    HandSkinControllerThread   *mThread;

    char                        mRobotName[256];
    
public:
            HandSkinControllerModule();
    virtual ~HandSkinControllerModule();


    virtual bool    open(Searchable &s);
    virtual bool    close();
    virtual bool    respond(const Bottle& command, Bottle& reply);
    virtual double  getPeriod();
    virtual bool    updateModule();
    virtual int     runModule(int argc, char *argv[], bool skipFirst = true);
};

#endif

