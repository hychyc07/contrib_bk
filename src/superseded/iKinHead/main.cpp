/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Francesco Nori
 * email:  francesco.nori@iit.it
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

#include <yarp/os/Network.h>
#include "eyeTriangulation.h"

/**
  *
  * @ingroup icub_superseded
  * \defgroup icub_iKinHead iKinHead
  *
  * Extracts absolute coordinates from image planes coordinates
  * 
  * \section intro_sec Description
  * 
  * \note This module has been superseded by the functionalities
  *       available from within the <a
  *       href="http://wiki.icub.org/iCub/main/dox/html/icub_gaze_interface.html">Gaze
  *       Interface</a>. Please consider upgrading your
  *       application.
  * 
  * This module transform the coordinates xr and xl
  * (image plane coordinates in the left and right eye)
  * into the coorinates X of the point in the root reference
  * frame (see <a
  * href="http://wiki.icub.org/wiki/ICubForwardKinematics">here</a>).
  * Image plane coordinates are defined so that the origin of
  * the reference frame is in the top left corner. The x axis is
  * pointing rightward and the y axis is pointing downward.
  * 
  * Furthermore, through a request forwarded to the rpc port, it
  * is also possibile to ask for a 3d point which corresponds to
  * a pixel in one image plane (the component z along the
  * optical axis in the eye frame shall be given too).
  *
  *\section dep_sec Dependencies
  * This application requires the iKin library and the GSL library.
  * At run time it requires the port /icub/head/state:o and
  * /icub/torso/state:o
  *
  *\section parameters_sec Parameters
  * This application requires a file describing the cameras calibration
  * matrices for the left and the right eye. These parameters can be
  * obtained with the module \ref icub_camcalibconf. The calibration matrices
  * are specified via the parameters (fx,fy,cx,cy). The left and right 
  * camera parms should be specifed in the
  * [CAMERA_CALIBRATION_LEFT] and the [CAMERA_CALIBRATION_RIGHT]
  * group respctively. An additional parameter can be specified
  * (--const). When this parameter is specifed the module does
  * not use the input port /eyeTriangulation/x:i (and therefore
  * does not wait for data to be written in this port). In this
  * configuration the image plane coordinates are substituted
  * with the left and right principal coordinates (cx,cy).
  * 
  * Moreover, two further groups [ALIGN_KIN_LEFT] and
  * [ALIGN_KIN_RIGHT] can be added to the configuration
  * file to describe the virtual links (given in terms of
  * lenght,offset,twist parameters) which are appended to the
  * eye kinematic in order to achieve the alignment with the
  * optical axes.
  * 
  * By adding the option --kalman the 3D output position is
  * filtered with a Kalman state estimator.
  *
  * Example: ./iKinHead --from icubEyes.ini --const --kalman
  * 
  * There are also the usual <i>--robot</i> and <i>--name</i>
  * command line options to change the name of the robot to
  * connect to and the module name.
  *
  *\section accessedPort_sec Ports Accessed
  * The module requires an access to the robot head configuration.
  * Therefore it assumes that the ports /icub/head/state:o and
  * /icub/torso/state:o are available (i.e. the iCubInterface is
  * running).
  *
  *\section openedPort_sec Ports Opened
  * The module opens four ports:
  *
  * <ul>
  * <li> /eyeTriangulation/qTorso:i is an input port for the
  * torso position vector (this port is automatically connected
  * to the port icub/torso/state:o)
  * <li> /eyeTriangulation/qHead:i is an input port for the 
  * head position vector (this port is automatically connected to
  * the port icub/head/state:o)
  * <li> /eyeTriangulation/x:i is an input port for the
  * 4-dimensional vector xlr (left+right image plane
  * coordinates)
  * <li> /eyeTriangulation/X:o is an output port for the three
  * dimensional vector X (root reference frames coordinates)
  * <li> /eyeTriangulation/rpc is the usual remote procedure
  * call port
  * </ul>
  * 
  * \section rpcProto_sec RPC protocol
  * Through the rpc port the following commands are available:
  * 
  * - <b>set eyedist \e type \e z</b>: sets the current distance
  *   from the eye along the optical axis (measured in the eye
  *   reference frame), where \e type can be \e left or \e right
  *   and \e z is the distance given in meters.
  * 
  * - <b>get eyedist \e type</b>: returns the current distance
  *   from the left/right eye [in meters].
  * 
  * - <b>get 3dpoint \e type \e u \e v [<i>z</i>]</b>: ask for
  *   the 3d point which corresponds to the (\e u,\e v)
  *   coordinates in the left/right image plane, given the
  *   distance \e z from the eye [in meters]. The coordinates of
  *   the resulting 3dpoint refer to the root frame attached to
  *   the waist.
  *
  * \author Francesco Nori
  *
  * Copyright (C) 2010 RobotCub Consortium
  *
  * CopyPolicy: Released under the terms of the GNU GPL v2.0.
**/


bool getProjectionMatrix(Bottle b, Matrix &P)
{
  double fx,fy,cx,cy;

  if(b.check("fx"))
    fx = b.find("fx").asDouble();
  else 
    return 0;
  if(b.check("fy"))
    fy = b.find("fy").asDouble();
  else 
    return 0;  

  // we suppose that the center distorsion is already compensated
  if(b.check("w"))
    cx = b.find("w").asDouble()/2.0;
  else 
    return 0;
  if(b.check("h"))
    cy = b.find("h").asDouble()/2.0;
  else 
    return 0;
  
  double sth= 0.0; double sx = fx; double sy = fy;
  double ox = cx;  double oy = cy;
  double f  = 1.0;

  Matrix K = eye(3,3);
  K(0,0)=sx*f;   K(1,1)=sy*f;   K(0,1)=sth*f;  K(0,2)=ox;   K(1,2)=oy; 

  Matrix Pi = zeros(3,4); 
  Pi(0,0)=1.0;  Pi(1,1)=1.0;  Pi(2,2)=1.0; 

  P = K*Pi;

  //fprintf(stderr, "Working with Projection %s\n", P.toString().c_str());

  return 1;
}

class CtrlModule: public RFModule
{
protected:
    eyeTriangulation *eT;
    Port              rpcPort;

public:
    CtrlModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        fprintf(stderr, "Getting projections\n");

        Matrix PiRight;
        Bottle b;
        b = rf.findGroup("CAMERA_CALIBRATION_RIGHT");
        //fprintf(stderr, "CAMERA_CALIBRATION_RIGHT contains: %s\n", b.toString().c_str());
        if (getProjectionMatrix(b, PiRight) == 0)
          {
            fprintf(stderr, "CAMERA_CALIBRATION_RIGHT was missing some params\n");
            return false;
          }
        else
          {
            fprintf(stderr, "Working with RightProjection \n");
            for (int i=0; i < PiRight.rows(); i++)
              fprintf(stderr, "%s\n", PiRight.getRow(i).toString().c_str());
          }

        Matrix PiLeft;
        b = rf.findGroup("CAMERA_CALIBRATION_LEFT");
        //fprintf(stderr, "CAMERA_CALIBRATION_LEFT contains: %s\n", b.toString().c_str());
        if (getProjectionMatrix(b, PiLeft) == 0)
          {
            fprintf(stderr, "CAMERA_CALIBRATION_LEFT was missing some params\n");
            return false;
          }
        else
          {
            fprintf(stderr, "Working with LeftProjection \n");
            for (int i=0; i < PiLeft.rows(); i++)
              fprintf(stderr, "%s\n", PiLeft.getRow(i).toString().c_str());
          }

        int period=50;
        if (rf.check("period"))
            period=rf.find("period").asInt();

        bool enableKalman=false;
        if (rf.check("kalman"))
            enableKalman=true;

        string ctrlName=rf.find("name").asString().c_str();
        string robotName=rf.find("robot").asString().c_str();

        fprintf(stderr, "Initializing eT\n");
        eT=new eyeTriangulation(rf, PiLeft, PiRight, enableKalman, period, ctrlName, robotName);

        Vector xr(3); xr(0)=PiRight(0,2); xr(1)=PiRight(1,2); xr(2)=1.0; 
        Vector xl(3); xl(0)=PiLeft(0,2);  xl(1)=PiLeft(1,2);  xl(2)=1.0; 
        eT->xInit(xl, xr);

        if (rf.check("const"))
            eT->xSetConstant(xl, xr);

        eT->start();    

        string rpcPortName="/"+ctrlName+"/rpc";
        rpcPort.open(rpcPortName.c_str());
        attach(rpcPort);

        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        if (eT->xExecReq(command,reply))
            return true;
        else
            return RFModule::respond(command,reply);
    }

    virtual bool close()
    {
        eT->stop();
        delete eT;

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("name","eyeTriangulation");
    rf.setDefault("robot","icub");
    rf.setDefaultContext("cameraCalibration/conf");
    rf.setDefaultConfigFile("icubEyes.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    CtrlModule mod;

    return mod.runModule(rf);
}



