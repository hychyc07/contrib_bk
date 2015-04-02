/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Bjoern Browatzki
 * email:   bjoern.browatzki@tuebingen.mpg.de
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

#ifndef _OBJREC_MODULE_H_
#define _OBJREC_MODULE_H_

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Thread.h>
#include <yarp/dev/PolyDriver.h>

#include "iCub/Util.h"
#include "iCub/HandGazeControl.h"

class ObjRecThread;
class ObjExplThread;
class BuildVtmThread;
class RecogVtmThread;
class InvKinThread;
class OpenHandThread;
class CalibGazeThread;
class HandControlThread;
class CreateObjectThread;
class HandGazeControl;
class VisSphereThread;


class ObjRecModule : public yarp::os::RFModule 
{
public:
    enum RobotArm
    {
        ARM_LEFT,
        ARM_RIGHT
    };
    
    enum CommandType
    {
        OPEN_HAND = 0,
        GRASP,
        BUILD_VTM,
        RECOG_VTM,
        LOOK,
        HAND_CONTROL,
        KINEXP,
        EXPLORE,
        RECOG,
        CALIB_GAZE,
        SET_PLANNING,
        SET_BOOSTING,
        STOP,
        HELP,
        QUIT,
        SUSPEND,
        RESUME,
        SNAPSHOT,
        SET_VTMRESULT_DIR,
        SET_RESULT_DIR
    };
    class CommandDef
    {
    public:
        CommandType id;
        std::string rpctxt;
        std::string desc;
        CommandDef(const CommandType _id, const std::string& _rpctxt, const std::string _desc)
            : id(_id), rpctxt(_rpctxt), desc(_desc)
        {}
    };

    typedef std::vector<CommandDef> CommandTable;

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();

    double getHandDist() const;
    cv::Rect getObjRect(const cv::Mat &src, double handDist) const;
    cv::Mat getCameraImage();
    Vector getEyePose(bool left) const;
    void getHandGazeAngles(double &e, double &r) const;
    Vector getHandPosition() const;
    Vector getEyePosition() const;
    Vector getTorsoQ() const;
    Vector getHeadQ() const;

    void sendActionDoneMsg(const std::string &msg);

    inline bool isRightArm() const { return activeArm == ARM_RIGHT; }
    inline bool isLeftEye() const { return leftEye; }
    
    HandGazeControl *handGazeControl;
private:
    static const int NUM_ARM_JOINTS = 7;
    static const int NUM_JOINTS = 16;

    static const double START_POSITION[];
    static const double START_POSITION_SIM[];
 
    PolyDriver robotHead;
    PolyDriver robotArm;
    PolyDriver robotTorso;
    PolyDriver armCartDriver;
    PolyDriver gazeControlDriver;

    IEncoders *encHead;
    IEncoders *encArm;
    IEncoders *encTorso;
    IPositionControl *armCtrl;
    IPositionControl *headCtrl;
    IPositionControl *torsoCtrl;
    IVelocityControl *velCtrl;
    IControlLimits *limArm;
    IControlLimits *limTorso;
    ICartesianControl *armCart;

    yarp::os::Port handlerPort;
    yarp::os::Port outPort;

    BufferedPort<ImageOf<PixelBgr> > camLeftPort;
    BufferedPort<ImageOf<PixelBgr> > camRightPort;
    BufferedPort<Bottle> worldPort;
    Port worldPortSync;

    std::string camLeftPortName;
    std::string camRightPortName;

    bool isSimulation;
    std::string robotName;

    int viewWindowHeight;
    int viewWindowWidth;

    double gazeOffsetX;
    double gazeOffsetY;
    double gazeOffsetZ;

    CommandTable commands;
    void defineCommands();
    void buildHelpMessage(Bottle &bottle);
    bool identifyCommand(const Bottle &commandBot, CommandType &com);
    bool openArm(RobotArm side, const std::string &handGazeDataFile, const Vector &handTargetPos);
    void lookAtAngle(double ed, double rd, int k = 1);

    bool leftEye;
    RobotArm activeArm;
    double objectRotation;

    /* pointer to a new thread to be created and started in configure() and stopped in close() */
    ObjRecThread *objRecThread;
    ObjExplThread *objExplThread;
    BuildVtmThread *buildVtmThread;
    RecogVtmThread *recogVtmThread;
    InvKinThread *invKinThread;
    OpenHandThread *openHandThread;
    CalibGazeThread *calibGazeThread;
    HandControlThread *handControlThread;
    CreateObjectThread *createObjectThread;
    RateThread *currentThread;
};

 
#endif // __OBJREC_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

