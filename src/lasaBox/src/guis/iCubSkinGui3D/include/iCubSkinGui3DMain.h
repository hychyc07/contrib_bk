// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2011 RobotSkin Consortium
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
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
\defgroup icub_iCubSkinGUI3D iCubSkinGUI3D
@ingroup icub_lasaBox_guis

This gui primarily loads a 3D mesh of a skin and displays live the activation of each sensor.
In addition to this, this guis can also:
- do some contact clustering to reduce the number of skin contact to deal with.
- simulate contacts with a "virtual" skin, given a stream of contact points.

\section intro_sec Description

\subsection subsec_basicgui Basic Gui:

This module loads two files (see desciption below) that represent, respectively,
the general structure of the cover, and the location of the sensor units.
Connecting its skin port to the iCub allows to see everything in 3D :)

For an example, assuming that you added the lasaBox's binaries to your path:
\verbatim
iCubSkinGui3D --name MySkinGUI3DLeftForearm --meshPath  $ICUB_ROOT/contrib/src/lasaBox/data/skinMeshes/iCubForearm
\endverbatim

<b>If you have a forearm skin on your iCub:</B>

Before connecting, you need to mix the upper and lower skin port together, using the provided mixer
\verbatim
YarpVectorMixer --name iCubSkinLeftForearm --portNum 2 --auto
yarp connect /icub/skin/left_forearm_lower /YarpVectorMixer/iCubSkinLeftForearm/input00
yarp connect /icub/skin/left_forearm_upper /YarpVectorMixer/iCubSkinLeftForearm/input01
yarp connect /YarpVectorMixer/iCubSkinLeftForearm/output /MySkinGUI3DLeftForearm/skin:i
\endverbatim
And here you go... You should be able to touch the icub's left forearm skin and see the result.

<b>If you don't have a forearm skin on your iCub:</B>

You may still see something nice by replaying some demo data that was recorded on our iCub
while running the code shown just above. For this we use our \ref icub_YarpVectorBag module.
\verbatim
YarpVectorBag --loop --bag $ICUB_ROOT/contrib/src/lasaBox/data/bags/iCubSkinLeft.bag
yarp connect /YarpVectorMixer/iCubSkinLeftForearm/output /MySkinGUI3DLeftForearm/skin:i
\endverbatim




\subsection subsec_clustering Using the contact clustering:

From the example above, you simply need to connect to /.../clusters:o port.
When N>0 contact are detected: this port streams a vector containing N contiguous 
blocks of 7 values. The cluster mean 3D position (3 values), its normal direction (3 values) 
and the "penetration" (1 values) which determines with a gross heuristic, how deep
the contact is inside the shape. When, there is no contact, the port streams a unique 
dummy value for the sake of continuous transmission.

\subsection subsec_virtualskin Using a virtual skin:

To simulate a virtual skin, first of all, do not connect anything into the /.../skin:i port, but
you shall stream point obstacles into the /.../obstacles:i port. For N obstacles, the data should be a Matrix of size
(N+1)x3 where each row is the obstacle position in the skin frame. The last row (+1) is used as
dummy value, again, for the sake of continuous data transmission.

Example:

\verbatim
iCubSkinGui3D --name MySkinGUI3DLeftVirtualForearm --meshPath  $ICUB_ROOT/contrib/src/lasaBox/data/skinMeshes/iCubVirtualForearm
yarp connect /someOutputObstaclePort /MySkinGUI3DLeftVirtualForearm/obstaclesskin:i
\endverbatim
And here you go...




\section dependencies_sec Dependencies

- YARP

\section parameters_sec Parameters

\verbatim
--name <string>:        the module base name for port creation (iCubSkinGUI3D by default)
--period <double>:      control loop period (0.01 sec by default)
--meshPath <string>:    the path where to look for the mesh files (i.e. cover.obj and sensorMap.txt)
--showNormal:           turn on the display of the mesh normals
\endverbatim


\section Input ports:

- /moduleName/skin:i
- /moduleName/obstacles:i

\section Output ports:

- /moduleName/clusters:o

\section in_files_sec Input Data Files

- cover.obj : 
  - A 3D object in ".obj" format (plain text) that define the general shape of the skin cover.

- sensorMap.txt : 
  - A text file with 6 columns and N rows. The first 3 columns consists of the 3D position
    of each sensor node and the last 3 columns, of their normal direction. When connected to 
    a skin port through the /skin:i input port, the number N
    of rows corresponds to the number of sensor entry in the skin port (it should usually be 
    a multiple of 12*16 = 192). If not, N can be of any size.


\section out_data_sec Output Data Files

None

\section conf_file_sec Configuration Files

None

\section tested_os_sec Tested OS

Linux

\section example_sec Example Instantiation of the Module

Some examples meshes can be found in $ICUB_ROOT/contrib/src/lasaBox/data/skinMeshes

\verbatim
iCubSkinGui3D --name mySKinGUI3D --meshPath $ICUB_ROOT/contrib/src/lasaBox/data/skinMeshes/iCubForearm --showNormal
\endverbatim

\author Eric Sauser

Copyright (C) 2011 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at $ICUB_ROOT/contrib/src/lasaBox/src/guis/iCubSKinGUI3D
**/



#ifndef ICUBSKINGUI3D_H_
#define ICUBSKINGUI3D_H_

#include "iCubSkinGui3DWidget.h"

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

using namespace yarp::os;

class iCubSkinGui3DThread: public RateThread
{
private:    
    Semaphore               mMutex;
    int                     mPeriod;
    char                    mBaseName[256];

    double                  mTime;
    double                  mPrevTime;

    BufferedPort<yarp::sig::Vector>    mInputPort;
    BufferedPort<yarp::sig::Matrix>    mObsInputPort;
    BufferedPort<yarp::sig::Vector>    mOutputPort;
    
public:
    iCubSkinGui3DWidget    *mWidget;

public:
    iCubSkinGui3DThread(int period, const char* baseName);
    virtual ~iCubSkinGui3DThread();


    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();
};

#endif

