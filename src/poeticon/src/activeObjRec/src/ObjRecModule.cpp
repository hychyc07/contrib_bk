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

/**
 * @file ObjRecModule.cpp
 * @brief implementation of the ObjRecModule methods following the RFModule standards.
 */

#include <boost/algorithm/string.hpp>

#include <yarp/dev/Drivers.h>
YARP_DECLARE_DEVICES(icubmod)

#include <iCub/iKin/iKinFwd.h>

#include <iCub/UtilityThreads.h>
#include "iCub/ObjRecModule.h"
#include "iCub/HandPoseUtil.h"
#include <iCub/ObjExplThread.h>
#include <iCub/BuildVtmThread.h>
#include <iCub/RecogVtmThread.h>
#include <iCub/InvKinThread.h>
#include <iCub/ObjRecThread.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

#define SEGMENTATION_ENABLED 1 // only affects simulation

const double ObjRecModule::START_POSITION[]={
    // arm
    -20, 30, 0, 65, -30, 0, -20,  
    // wrist
    15, 90, 20, 0,
    // fingers
    5, 5, 5, 5, 5 
};

const double ObjRecModule::START_POSITION_SIM[]={
    // arm
    -10, 0, 0, 80, -90, 0, 0,
    // wrist
    //15, 90, 20, 0,
    0, 0, 0, 0,
    // fingers
    //5, 5, 5, 5, 5 
    0, 0, 0, 0, 0 
};

void ObjRecModule::defineCommands()
{
    commands.clear();

    //                            command id,   rpc text,       desc     
    commands.push_back(CommandDef(OPEN_HAND,    "open hand",    "set the arm joints to the initial position")); 
    commands.push_back(CommandDef(GRASP,        "grasp",        "grasp using tactile feedback")); 
    commands.push_back(CommandDef(BUILD_VTM,    "buildvtm",     "explore object and create VTM")); 
    commands.push_back(CommandDef(RECOG,        "recog",        "recognize object minimizing entropy")); 
    commands.push_back(CommandDef(LOOK,         "look",         "look at object from certain angle")); 
    commands.push_back(CommandDef(HAND_CONTROL, "control hand", "allows controlling hand gaze using simple gui")); 
    commands.push_back(CommandDef(KINEXP,       "kinexp",       "execute random arm movement to learn inverse kinematics")); 
    commands.push_back(CommandDef(EXPLORE,      "explore",      "look at object while turning it around")); 
    commands.push_back(CommandDef(CALIB_GAZE,   "calib gaze",   "set parameters for gaze control")); 
    commands.push_back(CommandDef(SET_PLANNING, "set planning", "enable/disable motion planning")); 
    commands.push_back(CommandDef(SET_BOOSTING, "set boosting", "enable/disable pf boosting")); 
    commands.push_back(CommandDef(STOP,         "stop",         "stop learning/regnizing and reset the arm joint positions")); 
    commands.push_back(CommandDef(HELP,         "help",         "get this list")); 
    commands.push_back(CommandDef(QUIT,         "quit",         "quit the module")); 
    commands.push_back(CommandDef(SUSPEND,      "suspend",      "suspends the module"));
    commands.push_back(CommandDef(RESUME,       "resume",       "resumes the module"));
    commands.push_back(CommandDef(SNAPSHOT,     "snapshot",     "save snapshot of current view")); 
    commands.push_back(CommandDef(RECOG_VTM,    "vtm",          "recognize object using VTM")); 
    commands.push_back(CommandDef(SET_VTMRESULT_DIR,  "set vtmresultdir",  "specify output dir for vtm recog results")); 
    commands.push_back(CommandDef(SET_RESULT_DIR, "set recogresultdir", "specify output dir for recog results")); 

}

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 * equivalent of the "open" method.
 */

bool ObjRecModule::configure(yarp::os::ResourceFinder &rf) 
{    

    /* Process all parameters from both command-line and .ini file */
    /* get the module name which will form the stem of all module port names */
    std::string moduleName = rf.check("name", Value("activeObjRec"), "module name (string)").asString().c_str();

    std::string defaultDataRoot = "./data";

    std::string gazeCtrlName = rf.check("gazeCtrl", Value("iKinGazeCtrl"), "port name of gaze controller").asString().c_str();

    std::string arm = rf.check("arm", Value("right"), "left or right arm (string)").asString().c_str();
    activeArm = (arm == "right") ? ARM_RIGHT : ARM_LEFT;

    leftEye = (arm == "right") ? true : false; // use opposite eye
    

    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    /* now, get the rest of the parameters */
    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName = rf.check("robot", Value("icubSim"), "Robot name (string)").asString().c_str();

    isSimulation = (robotName.find("icubSim") != std::string::npos);
    
    Vector startPosition = isSimulation ? Vector(NUM_JOINTS, START_POSITION_SIM) : Vector(NUM_JOINTS,START_POSITION);
    
    viewWindowWidth   = rf.check("viewWindowSize", Value(160), "sub image width").asInt();
    viewWindowHeight  = viewWindowWidth;//rf->check("viewWindowHeight", Value(180), "sub image height").asInt();

    /* get the name of the input and output ports, automatically prefixing the module name by using getName() */
    camLeftPortName = "/";
    camRightPortName = "/";
    camLeftPortName += getName( rf.check("camLeftPort", Value("/camLeft:i"), "Input image port (ImageOf)").asString()).c_str();
    camRightPortName += getName( rf.check("camRightPort", Value("/camRight:i"), "Input image port (ImageOf)").asString()).c_str();

    gazeOffsetX = rf.check("gazeOffsetX", Value(0.0), "offset towards fingers").asDouble();
    gazeOffsetY = rf.check("gazeOffsetY", Value(0.0), "offset orthogonal to palm").asDouble();
    gazeOffsetZ = rf.check("gazeOffsetZ", Value(0.0), "offset in opposite direction of thumb").asDouble();

    double handTargetPosX  = rf.check("handTargetPosX", Value(-0.30)).asDouble();
    double handTargetPosY  = rf.check("handTargetPosY", Value(0.1)).asDouble();
    double handTargetPosZ  = rf.check("handTargetPosZ", Value(0.2)).asDouble();
    Vector handTargetPos(3);
    handTargetPos[0] = handTargetPosX;
    handTargetPos[1] = isRightArm() ? handTargetPosY : -handTargetPosY;
    handTargetPos[2] = handTargetPosZ;

    std::string defaultHandGazeDataFileRight = defaultDataRoot+"/handgazecontrol/samplesRight.txt";
    std::string defaultHandGazeDataFileLeft = defaultDataRoot+"/handgazecontrol/samplesLeft.txt";
    std::string handGazeDataFileRight = rf.check("handGazeDataFileRight", Value(defaultHandGazeDataFileRight.c_str()), "Training data for HandGazeControl").asString().c_str();
    std::string handGazeDataFileLeft = rf.check("handGazeDataFileLeft", Value(defaultHandGazeDataFileLeft.c_str()), "Training data for HandGazeControl").asString().c_str();
    std::string handGazeDataFile = isRightArm() ? handGazeDataFileRight : handGazeDataFileLeft; 
    std::cout << "HandGazeControl data file: " << handGazeDataFile << std::endl;

        
    /* do all initialization here */
    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    std::string handlerPortName = "/";
	handlerPortName	+= getName(rf.check("handlerPort", Value("/rpc:i")).asString());

    if (!handlerPort.open(handlerPortName.c_str()))
    {           
        std::cout << getName() << ": Unable to open port " << handlerPortName << std::endl;  
        return false;
    }

    attach(handlerPort);                  // attach to port

    std::string outPortName = "/"+moduleName+":o";
    if (! outPort.open(outPortName.c_str()))
    {
        std::cout << "unable to open output port " << std::endl;
        return false;
    }
    
    /* connect to robot head  */ 

    std::string localHeadPortName = "/";
    localHeadPortName += getName("/head");
    std::string remoteHeadPortName = "/" + robotName + "/head";
    Property optionsHead;
    optionsHead.put("device", "remote_controlboard");
    optionsHead.put("local", localHeadPortName.c_str());
    optionsHead.put("remote", remoteHeadPortName.c_str());

    robotHead.open(optionsHead);
    if (! robotHead.isValid())
    {
        std::cout << getName() << ": Cannot connect to robot head" << std::endl;  
        std::cout << "Device not available.  Here are the known devices:" << std::endl;
        std::cout << Drivers::factory().toString() << std::endl;
        return false;
    }


    // Torso
    std::string localTorsoPortName = "/";
    localTorsoPortName += getName("/torso");
    std::string remoteTorsoPortName = "/" + robotName + "/torso";

    Property torsoOptions;
    torsoOptions.put("device", "remote_controlboard");
    torsoOptions.put("local", localTorsoPortName.c_str());
    torsoOptions.put("remote", remoteTorsoPortName.c_str());

    robotTorso.open(torsoOptions);
    if (! robotTorso.isValid()) 
    {
        std::cout << getName() << ": Cannot connect to robot torso" << std::endl;  
        std::cout << "Device not available.  Here are the known devices:" << std::endl;
        std::cout << Drivers::factory().toString() << std::endl;
        return false;
    }

    /* set up gaze control interface */

    IGazeControl *gazeCtrl;
    std::string localGazePortName = "/";
    localGazePortName += getName("/gaze");
    std::string remoteGazePortName = "/"+gazeCtrlName;

    Property gazeOption;
    gazeOption.put("device","gazecontrollerclient");
    gazeOption.put("local", localGazePortName.c_str());
    gazeOption.put("remote", remoteGazePortName.c_str());

    gazeControlDriver.open(gazeOption);
    if (! gazeControlDriver.isValid()) 
    {
        std::cout << getName() << ": Cannot open gaze control driver" << std::endl;  
        std::cout << "Device not available.  Here are the known devices:" << std::endl;
        std::cout << Drivers::factory().toString() << std::endl;
        return false;
    }
    if (! gazeControlDriver.view(gazeCtrl))
    {
        std::cout << "Cannot get gaze control interface" << std::endl;  
        gazeControlDriver.close();
        return false;
    }
    gazeCtrl->setNeckTrajTime(0.5);
    gazeCtrl->setEyesTrajTime(0.5);
    
    /* open ports  */ 

    if (! camLeftPort.open(camLeftPortName.c_str()))
    {
        std::cout << getName() << ": unable to open port " << camLeftPortName << std::endl;
        return false; 
    }
    //if (! camRightPort.open(camRightPortName.c_str())) 
    //{
        //cout << getName() << ": unable to open port " << camRightPortName << endl;
        //return false;
    //}


    if (! robotTorso.view(torsoCtrl))
    {
        std::cout << "Cannot get interface to torso" << std::endl;  
        robotTorso.close();
        return false;
    }
    if (! robotTorso.view(encTorso))
    {
        std::cout << "Cannot get interface to torso" << std::endl;  
        robotTorso.close();
        return false;
    }
    if (! robotTorso.view(limTorso))
    {
        std::cout << "Cannot get interface to torso" << std::endl;  
        robotTorso.close();
        return false;
    }

    if (! robotHead.view(encHead))
    {
        std::cout << "Cannot get interface to the head" << std::endl;  
        robotHead.close();
        return false;
    }

    // Create interfaces and hand gaze control for left or right arm
    if (! openArm(activeArm, handGazeDataFile, handTargetPos))
        return false;

    // read default arm speeds and accelerations
    double armSpeed    = rf.check("armSpeed", Value(5.0), "default arm speed").asDouble();
    double armAcc      = rf.check("armAcc", Value(5.0), "default arm acceleration").asDouble();
    double fingerSpeed = rf.check("fingerSpeed", Value(80.0), "default arm speed").asDouble();
    double fingerAcc   = rf.check("fingerAcc", Value(80.0), "default finger acceleration").asDouble();
    const double offset = 1.0;
    for (int i=0; i<NUM_JOINTS; i++)
    {
        if (i < 7)
        {
    
            armCtrl->setRefSpeed(i, armSpeed + offset*yarp::os::Random::uniform());
            armCtrl->setRefAcceleration(i, armAcc);
        }
        else
        {
            armCtrl->setRefSpeed(i, fingerSpeed);
            armCtrl->setRefAcceleration(i, fingerAcc);
        }
    }

    if (isSimulation)
    {
        std::string worldPortName     = "/";
        std::string worldPortSyncName = "/";
        worldPortName     += getName(rf.check("worldPort", Value("/world:o"), "Output world port (string)").asString());
        worldPortSyncName += getName(rf.check("worldPortSync", Value("/worldSync:o"), "Output world port (string)").asString());

        if (! worldPort.open(worldPortName.c_str())) 
        {
            std::cout << getName() << ": unable to open port " << worldPortName << std::endl;
            return false;
        }
        if (! worldPortSync.open(worldPortSyncName.c_str())) 
        {
            std::cout << getName() << ": unable to open port " << worldPortSyncName << std::endl;
            return false;
        }

        std::string robotWorldPortName  = "/"+robotName+"/world";
        Network::connect(worldPortName.c_str(), robotWorldPortName.c_str(), "tcp"); 
        Network::connect(worldPortSyncName.c_str(), robotWorldPortName.c_str(), "tcp"); 

        // set path to 3d models
        std::string defaultObjectModelDir = defaultDataRoot+"/models";
        std::string objectModelDir = rf.check("objectModelDir", Value(objectModelDir.c_str()), "Directory with 3D object models for simulation").asString().c_str();
        std::cout << "Object model directory: " << objectModelDir << std::endl;
        std::cout << "sending model path to simulator..." << std::endl;
        // disabled for quad
        //Bottle& setModelDir = worldPort.prepare();
        //setModelDir.clear();
        //setModelDir.addString("world");
        //setModelDir.addString("set");
        //setModelDir.addString("mdir");
        //setModelDir.addString(objectModelDir.c_str());
        //worldPort.write();

#if SEGMENTATION_ENABLED
        Network::connect("/objSeg/obj:o", camLeftPortName.c_str(), "tcp"); 
#else
        std::string robotCamLeftPortName = "/"+robotName+"/cam/left";
        Network::connect(robotCamLeftPortName.c_str(), camLeftPortName.c_str(), "tcp"); 
#endif
    }

    // create threads and pass pointers to the module parameters 
    std::cout << "Creating threads..." << std::endl;

    int period_ms = 50;
    objRecThread = new ObjRecThread(this, period_ms, &rf, isSimulation);
    int doMotionPlanning = rf.check("motionPlanning", Value(1)).asInt();
    int doBoosting = rf.check("boosting", Value(1)).asInt();
    objRecThread->setBoosting(doBoosting);
    objRecThread->setMotionPlanning(doMotionPlanning);
    std::string defaultResultDir = defaultDataRoot+"/results";
    std::string recogResultDir = rf.check("recogResultDir", Value(defaultResultDir.c_str())).asString().c_str();
    std::cout << "Result directory: " << recogResultDir << std::endl;
    objRecThread->setResultDir(recogResultDir);

    objExplThread = new ObjExplThread(this, period_ms, armCartDriver, &rf, isSimulation);
    buildVtmThread = new BuildVtmThread(this, period_ms, robotArm, &rf);

    recogVtmThread = new RecogVtmThread(this, period_ms, robotArm, &rf);
    std::string vtmResultDir = rf.check("vtmResultDir", Value("./data/vtm_results"), "output directory for recognition results").asString().c_str();
    recogVtmThread->setResultDir(vtmResultDir);

    invKinThread = new InvKinThread(this, period_ms, robotArm, robotTorso, armCartDriver, &rf);
    invKinThread->setSampleFile(handGazeDataFile);

    openHandThread = new OpenHandThread(this, armCtrl, torsoCtrl, limArm, limTorso, startPosition);
    calibGazeThread = new CalibGazeThread(this, &rf); 
    handControlThread = new HandControlThread(this); 
    createObjectThread = isSimulation ? new CreateObjectThread(this, worldPort, worldPortSync) : NULL;

    // 
    // Set up hand gaze control 
    //
    handGazeControl = new HandGazeControl(50, handGazeDataFile, robotArm, robotTorso, armCartDriver, gazeCtrl, !isRightArm());
    handGazeControl->setHandTargetPos(handTargetPos);
    handGazeControl->setGazeOffsets(gazeOffsetX, gazeOffsetY, gazeOffsetZ);
    handGazeControl->start();

    std::cout << "Done." << std::endl;

    currentThread = NULL;

    // register RPC commands;
    defineCommands(); 

    return true;     // let the RFModule know everything went well
                      // so that it will then run the module
}

bool ObjRecModule::interruptModule()
{
    std::cout << "Interrupting module..." << std::endl;
    handlerPort.interrupt();
    camLeftPort.interrupt();
    camRightPort.interrupt();
    worldPort.interrupt();
    worldPortSync.interrupt();
    return true;
}

bool ObjRecModule::close()
{
    std::cout << "Closing module..." << std::endl;

    /* stop threads */
    if (objRecThread) objRecThread->stop();
    if (objExplThread) objExplThread->stop();
    if (invKinThread) invKinThread->stop();
    if (recogVtmThread) recogVtmThread->stop();
    if (buildVtmThread) buildVtmThread->stop();
    if (calibGazeThread) calibGazeThread->stop();
    if (handControlThread) handControlThread->stop();
    if (handGazeControl) handGazeControl->stop();

    handlerPort.close();
    camLeftPort.close();
    camRightPort.close();
    worldPort.close();
    worldPortSync.close();

    robotHead.close();
    robotArm.close();
    robotTorso.close();
    armCartDriver.close();
    gazeControlDriver.close();

    return true;
}

bool ObjRecModule::respond(const Bottle& command, Bottle& reply) 
{
    reply.clear();

    CommandType com;
    if(!identifyCommand(command, com))
    {
        reply.addString("Unknown command. Input 'help' to get a list of the available commands.");
        return true;
    }

    if (currentThread)
        currentThread->stop();

    // wait for the the old thread to finish FIXME: do proper syncronization
    Time::delay(0.1); 

    switch( com )
    {
        case SUSPEND:
            reply.addString("suspending");
            currentThread->suspend();
            break;
        case RESUME:
            reply.addString("resuming");
            currentThread->resume();
            break;
        case QUIT:
            reply.addString("quitting");
            return false;
        case HELP:
            buildHelpMessage(reply);
            return true;

        case OPEN_HAND:
            openHandThread->start();
            currentThread = openHandThread;
            break;

        case STOP:
            if (currentThread) 
            {
                std::cout << "stopping current thread" << std::endl;
                currentThread->stop();
            }
            break;

        case RECOG_VTM:
            {
                std::string trueLabel = "obj";
                if (command.size() >= 2 && command.get(1).isString())
                {
                    trueLabel = command.get(1).asString().c_str();
                    //reply.addString("Name of object missing!");
                    //return true;
                }
                recogVtmThread->setObjectName(trueLabel);
                recogVtmThread->start();
                currentThread = recogVtmThread;
            }
            break;

        case GRASP:
            if (isSimulation)
            {
                std::string objectName;
                if (command.size() >= 2 && command.get(1).isString())
                {
                    objectName = command.get(1).asString();
                }
                else
                {
                    reply.addString("Please specify the object that shall be grasped in the simulator (e.g. 'grasp redbox')");
                    return true;
                }
                double rotation = 0;
                std::string textureName = "colors";
                if (command.size() >= 3 && command.get(2).isInt())
                {
                    rotation = command.get(2).asInt();
                }
                if (command.size() >= 3 && command.get(2).isDouble())
                {
                    rotation = command.get(2).asDouble();
                }
                if (command.size() >= 3 && command.get(2).isString())
                {
                    textureName = command.get(2).asString().c_str();

                    if (command.size() >= 4 && command.get(3).isInt())
                        rotation = command.get(3).asInt();
                    if (command.size() >= 4 && command.get(3).isDouble())
                        rotation = command.get(3).asDouble();
                }

                createObjectThread->set3DModelParams(objectName, textureName, rotation);
                createObjectThread->start();
                currentThread = createObjectThread;
            }
            else
            {
                // currently only used in simulation
            }
            break;

        case LOOK:
            {
                double elevation = 0;
                double rotation = 0;
                if (command.size() >= 3 && command.get(1).isInt() && command.get(2).isInt())
                {
                    elevation = static_cast<double>(command.get(1).asInt());
                    rotation = static_cast<double>(command.get(2).asInt());
                }
                else
                {
                    reply.addString("No angles specified! Using default values 0 0.");
                }

                int k = 1;
                if (command.size() >= 4)
                {
                    k = command.get(3).asInt();
                }
                lookAtAngle(elevation, rotation, k);
            }
            break;
        case HAND_CONTROL:
            {
                currentThread = handControlThread;
                currentThread->start();
            }
            break;

        case KINEXP:
            {
                std::cout << "starting kinexp..." << std::endl;
                invKinThread->start();
                currentThread = invKinThread;
            }
            break;
        case EXPLORE:
            {
                if (command.size() < 2 || !command.get(1).isString())
                {
                    reply.addString("Name of object missing!");
                    return true;
                }
                if (isSimulation)
                {
                    Network::connect("/objSeg/obj:o", camLeftPortName.c_str(), "tcp"); 
                }
                std::string objName = command.get(1).asString().c_str();
                objExplThread->setObjectName(objName);

                std::cout << "Running thread..." << std::endl;
                objExplThread->start(); 

                currentThread = objExplThread;
            }
            break;
        case RECOG:
            {
                std::string trueLabel = "obj";
                if (command.size() >= 2 && command.get(1).isString())
                {
                    trueLabel = command.get(1).asString().c_str();
                    //reply.addString("Name of object missing!");
                    //return true;
                }
                if (isSimulation)
                {
                    Network::connect("/objSeg/obj:o", camLeftPortName.c_str(), "tcp"); 
                }
                std::cout << "object name:" << trueLabel << std::endl;

                double trueRotation = createObjectThread ? createObjectThread->getObjectRotation() : 0.0;
                objRecThread->setTrueLabel(trueLabel);
                objRecThread->setObjectRotation(trueRotation);
                objRecThread->startRecogTrials();
                objRecThread->start();

                currentThread = objRecThread;
            }
            break;
        case SET_PLANNING:
            {
                if (command.size() != 3 || !command.get(2).isInt())
                {
                    reply.addString("Usage: set planning 1|0");
                    return true;
                }
                bool status = command.get(2).asInt();
                objRecThread->setMotionPlanning(status);
                std::cout << "motion planning set to " << status << std::endl;
            }
            break;
        case SET_BOOSTING:
            {
                if (command.size() != 3 || !command.get(2).isInt())
                {
                    reply.addString("Usage: set boosting 1|0");
                    return true;
                }
                bool status = command.get(2).asInt();
                objRecThread->setBoosting(status);
                std::cout << "boosting set to " << objRecThread->isBoosting() << std::endl;
            }
            break;
        case SET_VTMRESULT_DIR:
            {
                if (command.size() != 3 || !command.get(2).isString())
                {
                    reply.addString("Usage: set vtmresultdir <path>");
                    return true;
                }
                std::string path = command.get(2).asString().c_str();
                recogVtmThread->setResultDir(path);
                std::cout << "vtm result dir set to " << path << std::endl;
            }
            break;
        case SET_RESULT_DIR:
            {
                if (command.size() != 3 || !command.get(2).isString())
                {
                    reply.addString("Usage: set recogresultdir <path>");
                    return true;
                }
                std::string path = command.get(2).asString().c_str();
                objRecThread->setResultDir(path);
                std::cout << "recog result dir set to " << path << std::endl;
            }
            break;

        case CALIB_GAZE:
            {
                calibGazeThread->start();
                currentThread = calibGazeThread;
            }
            break;
        case SNAPSHOT:
            {

                std::cout << "Take snapshot command received" << std::endl;
                if (command.size() < 2 || !command.get(1).isString())
                {
                    reply.addString("Filename missing!");
                    return true;
                }
                std::string fname = command.get(1).asString().c_str();

                if (camLeftPort.getInputCount() == 0)
                    return false;

                ImageOf<PixelBgr> *img = camLeftPort.read(false);
                if (! img)
                    return false;

                cv::Mat camImgLeft(img->height(), img->width(), CV_8UC3);
                camImgLeft = (IplImage*)img->getIplImage();

                cv::Mat cropped;
                cv::getRectSubPix(camImgLeft, cv::Size(150, 150), cv::Point(165, 145), cropped);

                cv::imwrite(fname, cropped);
                std::cout << "Image written to " << fname << std::endl;

            }
            break;
        case BUILD_VTM:
            {
                std::cout << "buildvtmcommand" << std::endl;
                if (command.size() < 2 || !command.get(1).isString())
                {
                    reply.addString("Name of object missing!");
                    return true;
                }
                std::string objName = command.get(1).asString().c_str();
                buildVtmThread->setObjectName(objName);
                buildVtmThread->start();
                currentThread = buildVtmThread;
            }
            break;
        default:
            reply.addString("ERROR: This command is known but it is not managed in the code.");
            return true;
    }

    reply.addString( (command.toString()+" command received.").c_str() );

    return true;
}

/* Called periodically every getPeriod() seconds */
bool ObjRecModule::updateModule() 
{
    return true;
}

double ObjRecModule::getPeriod() 
{
    /* module periodicity (seconds), called implicitly by myModule */    
    return 0.1;
}

void ObjRecModule::buildHelpMessage(Bottle &bottle)
{
    // print every string added to the bottle on a new line
    bottle.addVocab(Vocab::encode("many"));
    bottle.addString((std::string(getName().c_str()) + " commands are: ").c_str());
    CommandTable::const_iterator cmd;
    for (cmd = commands.begin(); cmd != commands.end(); ++cmd)
    {
        bottle.addString( ("- "+cmd->rpctxt+": "+cmd->desc).c_str() );
    }
}

/**
  * Identify the command in the bottle and return the correspondent enum value.
  */
bool ObjRecModule::identifyCommand(const Bottle &commandBot, CommandType &com)
{ 
    CommandTable::const_iterator cmd;
    for (cmd = commands.begin(); cmd != commands.end(); ++cmd)
    {
        //int pos = commandBot.toString().find(cmd->rpctxt.c_str());
        //if (pos != ConstString::npos)
        //{
            //com = cmd->id;
            //return true;
        //}

        std::vector<std::string> exp_words;
        boost::split(exp_words, cmd->rpctxt, boost::is_any_of(" "));

        bool found = true;
        for (int i=0; i<exp_words.size(); i++)
        {
            if (commandBot.size() < i)
            {
                found = false;
                break;
            }
            std::string rec_word = commandBot.get(i).asString().c_str();
            if (rec_word != exp_words[i])
                found = false;
        }
        if (found)
        {
            com = cmd->id;
            return true;
        }
    }
    return false;
}


cv::Mat ObjRecModule::getCameraImage()
{
    BufferedPort<ImageOf<PixelBgr> > *camPort = &camLeftPort;
    if (camPort->getInputCount() == 0)
    {
        std::cout << "Camera not connected!" << std::endl;
        Time::delay(1.0);
        return cv::Mat();
    }

    ImageOf<PixelBgr> *img = camPort->read(false);
    if (! img)
        return cv::Mat();

    cv::Mat camImg(img->height(), img->width(), CV_8UC3);
    camImg = (IplImage*)img->getIplImage();

    // disabled, we get cropped images from segmentation module
    //camImg = cropImageCenter(camImg, viewWindowWidth, viewWindowHeight, handDist);

    if (cv::countNonZero(camImg.reshape(1,0)) == 0)
        return cv::Mat();

    //
    // DEBUG (FOR SIMULATION ONLY)
    // background disabled in simulator -> no segmentation but images have to be cropped
    //

    // get distance to hand
    double handDist = getHandDist();

    // create sub-images
    cv::Rect objRect = getObjRect(camImg, handDist);

    objRect.x = std::max(0, objRect.x);
    objRect.y = std::max(0, objRect.y);
    objRect.width  = std::min(objRect.width, camImg.cols-objRect.x);
    objRect.height = std::min(objRect.height, camImg.rows-objRect.y);


    //////////////////////
    // crop
    //cv::Mat objimg = Util::removeBorder(camImgLeft, cv::Vec3b::all(0));

    cv::Mat objimg = camImg(objRect);
    
    // resize to common image size
    
    // calc dst img size (make resized img fit into fixed sized rectangle)
    int dstWidth;
    int dstHeight;
    if (objimg.cols > objimg.rows)
    {
        dstWidth = viewWindowWidth;
        dstHeight = ((double)viewWindowWidth/objimg.cols)*objimg.rows;
    }
    else
    {
        dstHeight = viewWindowHeight;
        dstWidth = ((double)viewWindowHeight/objimg.rows)*objimg.cols;
    }
    
    // make sure it's big enough for processing
    const int minSizePx = 40;
    dstHeight = std::max(minSizePx, dstHeight);
    dstWidth  = std::max(minSizePx, dstWidth);

    cv::Mat objimg_big;
    cv::resize(objimg, objimg_big, cv::Size(dstWidth,dstHeight), 0,0, cv::INTER_CUBIC);
    cv::imshow("object segmented", objimg_big);
    cv::waitKey(5);

    return objimg_big;
}

Vector ObjRecModule::getHandPosition() const
{
    Vector handCoord, handOrient;
    armCart->getPose(handCoord, handOrient);
    return handCoord;
}

double ObjRecModule::getHandDist() const
{
    return yarp::math::norm(getEyePosition()-getHandPosition());
}

Vector ObjRecModule::getEyePosition() const
{
    Vector eyeX(3);
    Vector eyePose = getEyePose(isLeftEye());
    eyeX[0] = eyePose[0];
    eyeX[1] = eyePose[1];
    eyeX[2] = eyePose[2];
    return eyeX;
}


Vector ObjRecModule::getTorsoQ() const
{
    int nAxesTorso;
    encTorso->getAxes(&nAxesTorso);
    Vector torso(nAxesTorso);
    encTorso->getEncoders(torso.data());
    return torso;
}

Vector ObjRecModule::getHeadQ() const
{
    int nAxesHead;
    encHead->getAxes(&nAxesHead);
    Vector head(nAxesHead);
    encHead->getEncoders(head.data());
    return head; 
}



Vector ObjRecModule::getEyePose(bool left) const
{
    std::string sideEye = left ? "right" : "left";

    iCub::iKin::iCubEye libEye(sideEye);         
    iCub::iKin::iKinChain *chain = libEye.asChain();     

    chain->releaseLink(0);
    chain->releaseLink(1);
    chain->releaseLink(2);

    Vector torso = getTorsoQ();
    Vector head  = getHeadQ();

    Vector ang(8);
    // torso
    ang[0] = torso[2]*M_PI/180;
    ang[1] = torso[1]*M_PI/180;
    ang[2] = torso[0]*M_PI/180;
    // neck
    ang[3] = head[0]*M_PI/180;
    ang[4] = head[1]*M_PI/180;
    ang[5] = head[2]*M_PI/180;
    // tilt
    ang[6] = head[3]*M_PI/180;
    // pan
    double vers = head[4]*M_PI/180;
    double verg = head[5]*M_PI/180;
    double eyeL =  vers + verg/2.0;  
    double eyeR =  vers - verg/2.0;  
    ang[7] = eyeL;

    chain->setAng(ang);
    Vector eyePose = chain->EndEffPose(false);

    return eyePose; 
}

cv::Rect ObjRecModule::getObjRect(const cv::Mat &src, double handDist) const
{
    double maxDist = 0.35;
    double minDist = 0.15;
    int minW = 100;  
    int maxW = 240;  
    int dW = maxW - minW;

    double s = (handDist-minDist) / (maxDist-minDist);
    int scaledWidth = minW + (1-s)*dW;
    scaledWidth = std::max(minW, std::min(maxW, scaledWidth));

    int width = scaledWidth;

    int cx = src.cols / 2;
    int cy = src.rows / 2;
    
    return cv::Rect(cx-width/2, cy-width/2, width, width); 

}

void ObjRecModule::getHandGazeAngles(double &e, double &r) const
{
    Vector armX, armO;
    armCart->getPose(armX, armO);
    Vector eyeX = getEyePosition();
    HandPoseUtil::getGazeAngles(eyeX, armX, armO, e, r, !isRightArm());
}

void ObjRecModule::sendActionDoneMsg(const std::string &msg)
{
    Bottle b;
    b.addString(msg.c_str());
    outPort.write(b);
}


bool ObjRecModule::openArm(RobotArm activeArm, const std::string &handGazeDataFile, const Vector &handTargetPos)
{

    std::string localArmPortName = "/";
    std::string remoteArmPortName;
    if (isRightArm())
    {
        localArmPortName += getName("/right_arm");
        remoteArmPortName = "/" + robotName + "/right_arm";
    }
    else
    {
        localArmPortName += getName("/left_arm");
        remoteArmPortName = "/" + robotName + "/left_arm";
    }

    Property armOptions;
    armOptions.put("device", "remote_controlboard");
    armOptions.put("local", localArmPortName.c_str());
    armOptions.put("remote", remoteArmPortName.c_str());

    robotArm.open(armOptions);
    if (! robotArm.isValid()) 
    {
        std::cout << getName() << ": Cannot connect to robot right arm" << std::endl;  
        std::cout << "Device not available.  Here are the known devices:" << std::endl;
        std::cout << Drivers::factory().toString() << std::endl;
        return false;
    }
    if (! robotArm.isValid()) 
    {
        std::cout << "Cannot connect to robot right arm" << std::endl;  
        std::cout << "Device not available.  Here are the known devices:" << std::endl;
        std::cout << Drivers::factory().toString() << std::endl;
        return false;
    }
    if (! robotArm.view(armCtrl))
    {
        std::cout << "Cannot get interface to the arm" << std::endl;  
        robotArm.close();
        return false;
    }
    if (! robotArm.view(velCtrl))
    {
        std::cout << "Cannot get interface to the arm" << std::endl;  
        robotArm.close();
        return false;
    }
    if (! robotArm.view(limArm))
    {
        std::cout << "Cannot get interface to the arm" << std::endl;  
        robotArm.close();
        return false;
    }
    if (! robotArm.view(encArm))
    {
        std::cout << "Cannot get interface to the arm" << std::endl;  
        robotArm.close();
        return false;
    }

    
    /* set up cartesian interface */

    std::string localCartArmPortName = "/";
    std::string remoteCartArmPortName;
    if (isRightArm())
    {
        localCartArmPortName += getName("/cartesian/right_arm");
        remoteCartArmPortName = "/" + robotName + "/cartesianController/right_arm";
    }
    else
    {
        localCartArmPortName += getName("/cartesian/left_arm");
        remoteCartArmPortName = "/" + robotName + "/cartesianController/left_arm";
    }

    Property option("(device cartesiancontrollerclient)");
    option.put("local", localCartArmPortName.c_str());
    option.put("remote", remoteCartArmPortName.c_str());

    armCartDriver.open(option);
    if (!armCartDriver.isValid()) 
    {
        std::cout << getName() << ": Cannot open cartesian interface driver for arm" << std::endl;  
        std::cout << "Device not available.  Here are the known devices:" << std::endl;
        std::cout << Drivers::factory().toString() << std::endl;
        return false;
    }

    if (! armCartDriver.view(armCart))
    {
        std::cout <<  "Cannot get cartesian interface to the arm" << std::endl;  
        armCartDriver.close();
        return false;
    }
    

    if (isSimulation)
        armCart->setTrajTime(2);
    
    return true;
}

void ObjRecModule::lookAtAngle(double ed, double rd, int k)
{
    handGazeControl->setK(k);
    handGazeControl->lookAtViewpoint(ed, rd);
    while (! handGazeControl->targetReached()) 
    {
    }
    double e, r;
    getHandGazeAngles(e, r);
    double err = Util::calcCentralAngle(ed, e, rd, r);

    std::cout << "\nk: ed/rd, e/r: " << k << ": " << ed << "/" << rd 
        << " " << e << "/" << r << "\t err = " << err << "°" << std::endl; 
} 


//double ObjRecModule::getObjectRotation() const 
//{ 
    //if (createObjectThread)
        //return createObjectThread->getObjectRotation();

    //return 0.0; 
//}

extern void testVtk();

int main(int argc, char * argv[])
{
    YARP_REGISTER_DEVICES(icubmod);

    /* initialize yarp network */

    Network yarp;

    /* create your module */

    ObjRecModule ObjRecModule; 

    /* prepare and configure the resource finder */

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("activeObjRec.ini");                //overridden by --from parameter
    rf.setDefaultContext("../contrib/src/poeticon/app/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    ObjRecModule.runModule(rf);

    return 0;
}
