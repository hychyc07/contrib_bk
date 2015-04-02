

/*
 robotMotorGui 
 lumaChroma --name niko --image hsv
 yarp connect /niko/H/image:o /Y
blobExtractor
yarp connect /niko/H/image:o /blobExtractor/img:i

yarp rpc /blobExtractor/rpc
    details on

* offset right hand:
          1           0 1.49012e-08    -40.7661
          0           1           0     19.8968
1.49012e-08 2.98023e-08           1     13.4058
          0           0           0           1
          * 
          1 7.45058e-09           0     2.91475
-1.49012e-08           1           0    -32.9242
-4.47035e-08           0           1     10.9863
          0           0           0           1



* 
* 
*  * */
#include "SimoxHandTrackerModule.h"
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/CollisionDetection/CDManager.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include <yarp/math/Math.h>
#include <cv.h>
#include <highgui.h>
#include <algorithm>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace std;
using namespace VirtualRobot;
//using namespace cv;

SimoxHandTrackerModule::SimoxHandTrackerModule()
{
    if (!SoDB::isInitialized())
        SoDB::init();
    timeUpdateEncodersS = 0.1;
    lastEncoderUpdateS = Time::now();
    encRightArm = encLeftArm = encHead = encTorso = NULL;
    imgsReady = false;
    currentTrackedTCP_rootCoord = Eigen::Matrix4f::Identity();
    showTrackedRobotNodes = true;
    showTrackedTCP = true;
    showTrackedEEF = true;
    showBlobs3D = true;
	localizationUseStereo = true;
    cvStorage = cvCreateMemStorage(0);
    cvInitFont( &font, CV_FONT_HERSHEY_COMPLEX_SMALL, .6, .6, 0, 1, 6);

}


bool SimoxHandTrackerModule::configure( yarp::os::ResourceFinder &rf )
{
    moduleName = rf.check("name",
                          Value("SimoxHandTracker"),
                          "module name (string)").asString();
    setName(moduleName.c_str());

    robotBase = rf.check("robot",
                         Value("icubSim"),
                         "robot name (string)").asString();

    std::string side = rf.check("side",
                                Value("left"),
                                "which side (string: left or right)").asString().c_str();

	cout << "Side check:" << side << endl;
    isLeft = true;
    if (side=="right")
    {
        cout << " Using right side" << endl;
        isLeft = false;
    } else
        cout << " Using left side" << endl;


    imgThreshold = (float)rf.check("imgThreshold",
                                   Value(160.0),
                                   "segmentation threshold (double)").asDouble();

	// stereo
	int locStereo = rf.check("localizationUseStereo",
		Value(1),
		"use stereo for localization (int) 0/1").asInt();
	if (locStereo==0)
		localizationUseStereo = false;
	else
		localizationUseStereo = true;

	if (localizationUseStereo)
	{
		cout << "Using stereo" << endl;
		std::string remoteStereoModule = "/";
		remoteStereoModule += rf.check("StereoDisparityModule",Value("stereoDisparity"),"Stereo Disparity module").asString().c_str();
		remoteStereoModule += "/rpc";
		std::string localStereoModule = "/";
		localStereoModule += getName().c_str();
		localStereoModule += "/stereoRPC:o";

		stereoDisparityRPC.open(localStereoModule.c_str());
		yarp::os::Network yarp;
		bool ok1 = yarp.connect(localStereoModule.c_str(),remoteStereoModule.c_str());
		
		if (ok1)
        {
			cout << "Successfully connected to " << remoteStereoModule << endl;
            //cout << "Turning off disparity map computation..." << endl;
            //yarp::os::Bottle cmd,response;
            //cmd.addString("disparity");
            //cmd.addString("OFF");
            //ok1 = stereoDisparityRPC.write(cmd,response);
            //cout << " response:" << response.toString() << endl;
        } else
		{
			cout << "##### Failed connecting to " << remoteStereoModule << ". Disabling stereo!" << endl;
			localizationUseStereo = false;
		}
	}

    VR_INFO << "Using robot base string " << robotBase << endl;

    // rpc handler port
    handlerPortName =  "/";
    handlerPortName += getName();         // use getName() rather than a literal
	handlerPortName += "/rpc:i";
    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        return false;
    }

    attach(handlerPort);                  // attach to port

    // image ports
    std::string imgPort1 ="/";
    imgPort1 += getName();
    imgPort1 += "/imgL:i";
    inImgLeftPort.open(imgPort1.c_str());
    std::string imgPort2 ="/";
    imgPort2 += getName();
    imgPort2 += "/segmentationL:o";
    outImgLeftPort.open(imgPort2.c_str());
    std::string imgPort3 ="/";
    imgPort3 += getName();
    imgPort3 += "/blobL:o";
    outBlobImgLeftPort.open(imgPort3.c_str());

    std::string imgPort1b ="/";
    imgPort1b += getName();
    imgPort1b += "/imgR:i";
    inImgRightPort.open(imgPort1b.c_str());
    std::string imgPort2b ="/";
    imgPort2b += getName();
    imgPort2b += "/segmentationR:o";
    outImgRightPort.open(imgPort2b.c_str());
    std::string imgPort3b ="/";
    imgPort3b += getName();
    imgPort3b += "/blobR:o";
    outBlobImgRightPort.open(imgPort3b.c_str());

    if (rf.check("SimoxDataPath"))
    {
        ConstString dataPath=rf.find("SimoxDataPath").asString();
        VR_INFO << "Adding rf.SimoxDataPath: " << dataPath.c_str() << endl;
        VirtualRobot::RuntimeEnvironment::addDataPath(dataPath.c_str());
    }
    ConstString robotFile=rf.find("RobotFile").asString();
    std::string robFileComplete = robotFile.c_str();
    if (!VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFileComplete))
    {
        VR_ERROR << " Could not find file: " << robFileComplete << endl;
    } else
    {
        std::cout << "Using robot file: " << robFileComplete << endl;
        loadRobot(robFileComplete);
    }

    ConstString rootNode=rf.find("RootCoordSystem").asString();
    if (!robot || !robot->hasRobotNode(rootNode.c_str()))
    {
        VR_ERROR << " Root Node <" << rootNode.c_str() << "> not found..." << endl;
    } else
    {
        std::cout << "Using root coordinate system: " << rootNode.c_str() << endl;
        rootCoordSystem = robot->getRobotNode(rootNode.c_str()) ;
    }

    ConstString tcpNode=rf.find("TcpCoordSystem").asString();
    if (!robot || !robot->hasRobotNode(tcpNode.c_str()))
    {
        VR_ERROR << " TCP Node <" << tcpNode.c_str() << "> not found..." << endl;
    } else
    {
        std::cout << "Using tcp coordinate system: " << tcpNode.c_str() << endl;
        tcpCoordSystem = robot->getRobotNode(tcpNode.c_str()) ;
    }

    trackRobotNodes.clear();
    Value *val;
    if (robot && rf.check("TrackRobotNodes",val))
    {
        //yarp::os::Bottle *jv = rf.find("IkInitConfig").asList();
        if (val->isList())
        {
            yarp::os::Bottle *jv = val->asList();
            if (jv->size() > 0)
            {
                VR_INFO << "TrackRobotNodes:";
                for (int i=0;i<jv->size();i++)
                {
                    std::string rn = jv->get(i).asString().c_str();
                    if (!robot->hasRobotNode(rn) || !robot->getRobotNode(rn))
                    {
                        VR_ERROR << "Wrong TrackRobotNode entry: <" << rn << "> is not present in robot <" << robot->getName() << ">" << endl;
                    } else
                    {
                        trackRobotNodes.push_back(robot->getRobotNode(rn));
                        cout << rn << endl;
                    }
                }
                cout << endl;
            } else
            {
                VR_ERROR << "No TrackRobotNodes defined, need at least 1 RobotNode that should be tracked!!" << endl;
            }
        }
    }

    // VISUALIZATION FLAGS
    yarp::os::Bottle visu = rf.findGroup("Visualization");
    bool enableVisu = false;
    if (!visu.isNull() && robot)
    {
        if (visu.check("EnableVisualization"))
        {
            std::string cdE = visu.find("EnableVisualization").asString().c_str();
            if (cdE=="true" || cdE=="on" || cdE=="1")
                enableVisu = true;
        }
        if (visu.check("ShowHandEstimation"))
        {
            std::string eefName = visu.find("ShowHandEstimation").asString().c_str();

            if (!robot || !robot->hasEndEffector(eefName.c_str()))
            {
                VR_ERROR << " EEF definition <" << eefName.c_str() << "> not found..." << endl;
            } else
            {
                std::cout << "Using EEF definition: " << eefName.c_str() << endl;
                eef = robot->getEndEffector(eefName.c_str()) ;
            }
        } else
            std::cout << "Disabling visualization of Hand estimation" << endl;

    }
    if (enableVisu)
    {
        VR_INFO << "Viewer is ON..." << endl;
        setupViewer();
    } else
        VR_INFO << "Viewer is OFF..." << endl;


    // setup LEFT ARM
    std::string localLeftArmName = "/";
    localLeftArmName += getName();
    localLeftArmName += "/left_arm";
    Property optionsLeftArm;
    optionsLeftArm.put("device", "remote_controlboard");
    optionsLeftArm.put("local", localLeftArmName.c_str());      //local port names
    std::string remoteLeftArm = ("/" + robotBase + "/left_arm");
    optionsLeftArm.put("remote",remoteLeftArm.c_str());        //where we connect to
    encLeftArm = NULL;
    if (robotDeviceLeftArm.open(optionsLeftArm))
    {
        robotDeviceLeftArm.view(encLeftArm);
        int axesLeftArm;
        if (!encLeftArm || !encLeftArm->getAxes(&axesLeftArm) || axesLeftArm<=0) {
            printf("Could not get encoder values from left_arm\n");
            close();
            return false;
        }
        jointValuesLeftArm.resize(axesLeftArm,0.0);
    }

    // setup RIGHT ARM
    std::string localRightArmName = "/";
    localRightArmName += getName();
    localRightArmName += "/right_arm";
    Property optionsRightArm;
    optionsRightArm.put("device", "remote_controlboard");
    optionsRightArm.put("local", localRightArmName.c_str());      //local port names
    std::string remoteRightArm = ("/" + robotBase + "/right_arm");
    optionsRightArm.put("remote", remoteRightArm.c_str());         //where we connect to
    encRightArm = NULL;
    if (robotDeviceRightArm.open(optionsRightArm))
    {
        robotDeviceRightArm.view(encRightArm);
        int axesRightArm;
        if (!encRightArm || !encRightArm->getAxes(&axesRightArm) || axesRightArm<=0) {
            printf("Could not get encoder values from right_arm\n");
            close();
            return false;
        }
        jointValuesRightArm.resize(axesRightArm,0.0);
    }


    // setup HEAD
    std::string localHeadName = "/";
    localHeadName += getName();
    localHeadName += "/head";
    Property optionsHead;
    optionsHead.put("device", "remote_controlboard");
    optionsHead.put("local", localHeadName.c_str());      //local port names
    std::string remoteHead = ("/" + robotBase + "/head");
    optionsHead.put("remote", remoteHead.c_str());         //where we connect to
    encHead = NULL;
    if (robotDeviceHead.open(optionsHead))
    {
        robotDeviceHead.view(encHead);
        int axesHead;
        if (!encHead || !encHead->getAxes(&axesHead) || axesHead<=0) {
            printf("Could not get encoder values from head\n");
            close();
            return false;
        }
        jointValuesHead.resize(axesHead,0.0);
    }

    // setup TORSO
    std::string localTorsoName = "/";
    localTorsoName += getName();
    localTorsoName += "/torso";
    Property optionsTorso;
    optionsTorso.put("device", "remote_controlboard");
    optionsTorso.put("local", localTorsoName.c_str());      //local port names
    std::string remoteTorso = ("/" + robotBase + "/torso");
    optionsTorso.put("remote", remoteTorso.c_str());     //where we connect to
    encTorso = NULL;
    if (robotDeviceTorso.open(optionsTorso))
    {
        robotDeviceTorso.view(encTorso);
        int axesTorso;
        if (!encTorso || !encTorso->getAxes(&axesTorso) || axesTorso<=0) {
            printf("Could not get encoder values from Torso\n");
            close();
            return false;
        }
        jointValuesTorso.resize(axesTorso,0.0);
    }

    // connect to gaze control
    Property optionGaze;
    optionGaze.put("device","gazecontrollerclient");
    optionGaze.put("remote","/iKinGazeCtrl");
    optionGaze.put("local","/client_handtracker/gaze");
    if (!iCubGazeClient.open(optionGaze))
    {
        VR_ERROR << " Could not open iKinGazeCtrl for GazeClient" << endl;
        return false;
    }
    // open the view
    iCubGazeClient.view(iCubGazeControl);
    if (!iCubGazeControl)
    {
        VR_ERROR << "Could not get iCub gaze controller..." << endl;
        return false;
    }

    // RobotNodeSets
    rnsArmLeft = rf.find("RobotNodeSet_LeftArm").asString();
    rnsArmRight = rf.find("RobotNodeSet_RightArm").asString();
    rnsHandLeft = rf.find("RobotNodeSet_LeftHand").asString();
    rnsHandRight = rf.find("RobotNodeSet_RightHand").asString();
    rnsArmLeft = rf.find("RobotNodeSet_LeftArm").asString();
    rnsArmRight = rf.find("RobotNodeSet_RightArm").asString();
    rnsTorso = rf.find("RobotNodeSet_Torso").asString();
    rnsLegLeft = rf.find("RobotNodeSet_LeftLeg").asString();
    rnsLegRight = rf.find("RobotNodeSet_RightLeg").asString();
    rnsHead = rf.find("RobotNodeSet_Head").asString();
	cout << "4" << endl;


    cout <<"\n**** config end\n" << endl;
    return true;
}

bool SimoxHandTrackerModule::setupViewer()
{
    SoQt::init("SimoxHandTrackerModule");
    viewer.reset(new SimoxRobotViewer("Simox Hand Tracker Status"));
    if (robot)
    {
    	//viewer->loadRobot(robotFile);
    	// todo check clone!!!
        viewer->setRobot(robot->clone("SimoxHandTracker_Visu"));
        viewer->showRobot(true,VirtualRobot::SceneObject::Collision);

        // enable coord systems of tracked joints
        showTrackedPositionsVisu(true);

        // set tcp coord system (model)
        if (tcpCoordSystem)
            viewer->showCoordSystem(tcpCoordSystem->getName(),true);
        // set tcp coord system (tracked)
        showTrackedTCPVisu(true);

        if (rootCoordSystem)
            viewer->showCoordSystem(rootCoordSystem->getName(),true);

        if (eef)
        {
            cout << "Enabling eef visu for " << eef->getName() << endl;
            showTrackedEEF = true;
            viewer->setEEFVisu("EEFPose",eef->getName(),VirtualRobot::SceneObject::Full);
        }

        viewer->viewAll();
    }
    return true;
}

bool SimoxHandTrackerModule::updateModule()
{
    if (viewer)
    {
        if (viewer->wasClosed())
        {
            cout << "Viewer was closed, quitting..." << endl;
            return false;
        }
    }

    grabImages();

    // get joint values
    double delay = Time::now() - lastEncoderUpdateS;
    if (delay> timeUpdateEncodersS)
    {
        lastEncoderUpdateS = Time::now();

        if (encLeftArm)
        {
            if (encLeftArm->getEncoders(jointValuesLeftArm.data()) && jointValuesLeftArm.size()==16)
            {
                std::vector<float> jv;
                // get first 7 values
                for (int i=0;i<7;i++)
                    jv.push_back(jointValuesLeftArm[i]*M_PI/180.0);
                setJointValues(rnsArmLeft,jv);
            }
            //std::vector<float> jvHand = getFingerJoints(jointValuesLeftArm);
            //setJointValues(rnsHandLeft,jvHand);
        }
        if (encRightArm)
        {
            if (encRightArm->getEncoders(jointValuesRightArm.data()) && jointValuesRightArm.size()==16)
            {
                std::vector<float> jv;
                // get first 7 values
                for (int i=0;i<7;i++)
                    jv.push_back(jointValuesRightArm[i]*M_PI/180.0);
                setJointValues(rnsArmRight,jv);
            }
            //std::vector<float> jvHand = getFingerJoints(jointValuesRightArm);
            //setJointValues(rnsHandRight,jvHand);
        }
        if (encHead)
        {
            if (encHead->getEncoders(jointValuesHead.data()) && jointValuesHead.size()==6)
            {
                std::vector<float> jv;
                float vs = jointValuesHead[4]*M_PI/180.0;
                float vg = jointValuesHead[5]*M_PI/180.0;
                jv.push_back(-jointValuesHead[2]*M_PI/180.0);
                jv.push_back(jointValuesHead[0]*M_PI/180.0);
                jv.push_back(jointValuesHead[1]*M_PI/180.0);
                jv.push_back(jointValuesHead[3]*M_PI/180.0);
                jv.push_back(vs + vg/2.0f);  // L = vs + vg/2
                jv.push_back(jointValuesHead[3]*M_PI/180.0);
                jv.push_back(vs - vg/2.0f); // R = vs - vg/2

                setJointValues(rnsHead,jv);
            }
        }
        if (encTorso)
        {
            if (encTorso->getEncoders(jointValuesTorso.data()) && jointValuesTorso.size()==3)
            {
                std::vector<float> jv;
                jv.push_back(jointValuesTorso[2]*M_PI/180.0);// xchange yaw and pitch
                jv.push_back(jointValuesTorso[1]*M_PI/180.0);
                jv.push_back(jointValuesTorso[0]*M_PI/180.0);

                setJointValues(rnsTorso,jv);
            }
        }
    }

    if (viewer)
    {
        // send alive signal
        viewer->ping();

        // update gui
        if (qApp)
        {
            //viewer->lock();
            qApp->processEvents();
            //viewer->unlock();
        }
    }

    return true;
}

float SimoxHandTrackerModule::getMinDist(std::vector<float> &point3D, int &storeIndex)
{
    float minD = 1000000.0f;
    if (point3D.size()!=3)
    {
        cout << " expecting 3dim coordinate, aborting..." << endl;
        storeIndex = 0;
        return -1.0f;
    }
    cout << "Going through all model nodes, to get min dist"<< endl;
    mutexRobotAccess.wait();
    for (size_t i=0; i<currentModelPositions_rootCoord.size(); i++)
    {
        float x = point3D[0] - currentModelPositions_rootCoord[i](0)/1000.0f;
        x = x*x;
        float y = point3D[1] - currentModelPositions_rootCoord[i](1)/1000.0f;
        y = y*y;
        float z = point3D[2] - currentModelPositions_rootCoord[i](2)/1000.0f;
        z = z*z;
        float d = sqrtf(x+y+z);
        cout << "Dist to Model " << i << ":" << d << endl;
        if (d<minD)
        {
            minD = d;
            storeIndex = (int)i;
        }
    }
    mutexRobotAccess.post();
    cout << "Min dist to Model " << storeIndex << ":" << minD << endl;

    return minD;
}
/*
bool SimoxHandTrackerModule::get3DPointStereo(int cx, int cy, Eigen::Vector3f &storePoint3d_rootCoord)
{
	if (!robot || trackRobotNodes.size()==0)
		return false;
	mutexRobotAccess.wait();
	iCubGazeControl->get3DPoint(camNr,px,z,x);
	storePoint3d_rootCoord(0) = x[0]*1000.0f;
	storePoint3d_rootCoord(1) = x[1]*1000.0f;
	storePoint3d_rootCoord(2) = x[2]*1000.0f;
	mutexRobotAccess.post();

	return true;
}*/

float SimoxHandTrackerModule::getFingerToCamDist(bool leftCam)
{
	// todo: read from file
    std::string camNode;
	if (leftCam)
		camNode = "EyeLeftCam";
	else
		camNode = "EyeRightCam";
	if (!robot || trackRobotNodes.size()==0)
		return false;
	mutexRobotAccess.wait();

	VirtualRobot::RobotNodePtr rn = robot->getRobotNode(camNode);
	if (!rn)
	{
		cout << " Error: no rn with name " << camNode << endl;
		mutexRobotAccess.post();
		return false;
	}
	if (trackRobotNodes.size()==0)
	{
		cout << "Wrong nr of nodes to track?!" << endl;
		mutexRobotAccess.post();
		return false;
	}
	Eigen::Matrix4f pose = trackRobotNodes[0]->getGlobalPose();
	pose = rn->toLocalCoordinateSystem(pose);
	float z = fabs(pose(2,3));
	mutexRobotAccess.post();
	return z;
}

bool SimoxHandTrackerModule::get3DPointMono(bool leftCam, int cx, int cy, float distCamMM, Eigen::Vector3f &storePoint3d_rootCoord)
{
    if (!iCubGazeControl)
    {
        cout << "No iCubGazeControl?!" << endl;
        return false;
    }

    float z = distCamMM / 1000.0f;// dist in m
    cout << " DIST from cam [mm]:" << z*1000.0f << endl;
    Vector x;
    Vector px(2);
    px[0] = cx;
    px[1] = cy;
    // 0: left, 1: right
	int camNr;
	if (leftCam)
		camNr = 0;
	else
		camNr = 1;

	 mutexRobotAccess.wait();
	iCubGazeControl->get3DPoint(camNr,px,z,x);
    storePoint3d_rootCoord(0) = x[0]*1000.0f;
    storePoint3d_rootCoord(1) = x[1]*1000.0f;
    storePoint3d_rootCoord(2) = x[2]*1000.0f;
    mutexRobotAccess.post();

    return true;
}

bool SimoxHandTrackerModule::searchBlobs(IplImage *gray, std::vector< CvPoint > &points2d, int w_offset, int h_offset, int minPixels, int maxPixels, IplImage *result)
{
    if (!gray)
        return false;
    points2d.clear();
    for(int row=h_offset; row<gray->height-h_offset; row++)
    {
        uchar *ptr=(uchar*) gray->imageData + row*gray->widthStep;
        for(int col=w_offset; col<gray->width-w_offset; col++)
        {
            if(ptr[col]==255)
            {
                cout <<"********************** found new blob"<<endl;
                CvConnectedComp comp;
                // draw black, so the pixels will not be selected inone of the succeeding loops
                cvFloodFill(gray,cvPoint(col,row),cvScalar(0),cvScalar(0),cvScalar(0),&comp);
                cout << "BLOB , pixels:" << comp.area << ", pos:" << comp.rect.x << "," << comp.rect.y << " size: " << comp.rect.width << "," << comp.rect.height << endl;
                if (comp.area<minPixels)
                {
                    cout << "Small -> ignoring" << endl;
                    continue;
                }
                if (comp.area>maxPixels)
                {
                    cout << "Huge -> ignoring" << endl;
                    continue;
                }

                double x = comp.rect.x + comp.rect.width/2;
                double y = comp.rect.y + comp.rect.height/2;
                CvPoint a;
                a.x = comp.rect.x;
                a.y = comp.rect.y;
                CvPoint b;
                b.x = comp.rect.x + comp.rect.width;
                b.y = comp.rect.y + comp.rect.height;

                if (result)
                    cvRectangle(result,a,b,cvScalar(255,0,0));

                cout << "detecting blob NR " << points2d.size() << " center:" << x << "," << y << " size: " << comp.rect.width << "," << comp.rect.height << endl;

                CvPoint c;
                c.x = x;
                c.y = y;
                points2d.push_back(c);
            }
        }
    }
    if (result)
    {
        // draw search area
        CvPoint a;
        a.x = w_offset;
        a.y = h_offset;
        CvPoint b;
        b.x = result->width - w_offset;
        b.y = result->height - h_offset;
        cout << "minX:" << a.x << ", maxX:" << b.x << endl;
        cout << "minY:" << a.y << ", maxY:" << b.y << endl;
        cvRectangle(result,a,b,cvScalar(0,255,0));
    }

    return (points2d.size()>0);
}

float SimoxHandTrackerModule::getMinDistToModelNodes(Eigen::Vector3f &pos)
{
    float res = FLT_MAX;
    mutexRobotAccess.wait();
    cout << " distance to all robotNodes: *********\n";
    for (size_t j=0; j<trackRobotNodes.size();j++)
    {
        Eigen::Matrix4f m = trackRobotNodes[j]->getGlobalPose();
        if (rootCoordSystem)
            m = rootCoordSystem->toLocalCoordinateSystem(m);
        Eigen::Vector3f rnPos = m.block(0,3,3,1);
        rnPos -= pos;
        float d = rnPos.norm();
        cout << " d:" << d << endl;
        if (d<res)
            res = d;
    }
    cout << "Min dist: " << res << endl;
    mutexRobotAccess.post();
    return res;
}

bool SimoxHandTrackerModule::get3DPositionStereo(const std::vector< CvPoint > &points2d_l, const std::vector< CvPoint > &points2d_r, std::vector< Eigen::Vector3f > &storeBlobs3D_rootCoord)
{
	if (points2d_l.size()==0 || points2d_r.size()==0)
		return false;
	if (!localizationUseStereo)
	{
		cout << "Stereo disabled!" << endl;
		return false;
	}
	if (points2d_l.size()!=points2d_r.size())
	{
		cout << "vector sized do not match" << endl;
		return false;
	}
	Bottle cmd,response;
	CvPoint p;
	for (size_t i=0;i<points2d_l.size();i++)
	{
		p = points2d_l[i];
		cmd.addDouble((double)p.x);
		cmd.addDouble((double)p.y);
		p = points2d_r[i];
		cmd.addDouble((double)p.x);
		cmd.addDouble((double)p.y);
	}
	cout << "sending " << cmd.toString().c_str() << " to stereoDisparityRPC" << endl;
	bool ok = stereoDisparityRPC.write(cmd,response);
	cout << "respond: " << response.toString().c_str() << endl;
	if (!ok)
	{
		cout << "Error requesting 3d stereo data..." << endl;
		return false;
	}
	int nrPoints = response.size() / 3;
	cout << "Retrieving " << nrPoints << " stereo 3d points_rootCoord:" << endl;
	Eigen::Vector3f pos;
	storeBlobs3D_rootCoord.clear();
	for (int i=0; i<nrPoints;i++)
	{
		pos(0) = response.get(i*3).asDouble() * 1000.0f; // we use mm
		pos(1) = response.get(i*3+1).asDouble() * 1000.0f;
		pos(2) = response.get(i*3+2).asDouble() * 1000.0f;
		cout << "** Point in 3D (root coord):" << pos[0] << "," << pos[1] << "," << pos[2] << endl;
		storeBlobs3D_rootCoord.push_back(pos);
	}
	return storeBlobs3D_rootCoord.size()>0;
}

bool SimoxHandTrackerModule::get3DPositionMono(bool leftCam, float distCamMM, const std::vector< CvPoint > &points2d, std::vector< Eigen::Vector3f > &storeBlobs3D_rootCoord)
{
    if (points2d.size()==0)
        return false;
    Eigen::Vector3f pos;
    storeBlobs3D_rootCoord.clear();
    for (size_t i=0; i<points2d.size();i++)
    {
        if (get3DPointMono(leftCam,points2d[i].x,points2d[i].y, distCamMM, pos))
        {
            cout << "Point in 3D (root coord):" << pos[0] << "," << pos[1] << "," << pos[2] << endl;
            storeBlobs3D_rootCoord.push_back(pos);
        }
    }
    return storeBlobs3D_rootCoord.size()>0;
}

bool SimoxHandTrackerModule::sortPositionsLine(const std::vector< CvPoint > &points2d, const std::vector< Eigen::Vector3f > &inputBlobs3D, std::vector< Eigen::Vector3f > &blobs3D, std::vector<int> &indexMapping)
{
	double minX = points2d[indexMapping[0]].x;
	double maxX = points2d[indexMapping[0]].x;
	double minY = points2d[indexMapping[0]].y;
	double maxY = points2d[indexMapping[0]].y;
	for (size_t j=0;j<indexMapping.size();j++)
	{
		if (points2d[indexMapping[j]].x < minX)
			minX = points2d[indexMapping[j]].x;
		if (points2d[indexMapping[j]].x > maxX)
			maxX = points2d[indexMapping[j]].x;
		if (points2d[indexMapping[j]].y < minY)
			minY = points2d[indexMapping[j]].y;
		if (points2d[indexMapping[j]].y > maxY)
			maxY = points2d[indexMapping[j]].y;
	}
	cout << " MinX:" << minX << "," << " MaxX:" << maxX << "," << " MinY:" << minY << "," << " MaxY:" << maxY << endl;
	std::map<float,size_t> yIndexMap;
	for (size_t j=0; j<indexMapping.size();j++)
    {
        float y = points2d[indexMapping[j]].y;

        yIndexMap[y] = indexMapping[j];
    }
    
    std::vector< Eigen::Vector3f > resultBlobs;
    std::vector<int> resultMapping;
    // map is sorted internally -> that's fine for us now
    std::map<float,size_t>::iterator it = yIndexMap.begin();
    while (it != yIndexMap.end())
    {
    	cout << "## y value:" << it->first << endl;
		resultBlobs.push_back(inputBlobs3D[it->second]);
		resultMapping.push_back(it->second);
		it++;
	}
	
	blobs3D = resultBlobs;
	indexMapping = resultMapping;
	
	return true;
}

bool SimoxHandTrackerModule::getBestMatchesDistance(const std::vector< Eigen::Vector3f > &inputBlobs3D, std::vector< Eigen::Vector3f > &filteredBlobs3D, float maxModelDistanceMM, std::vector<int> *storeIndexMapping)
{
    if (inputBlobs3D.size()<trackRobotNodes.size())
    {
        VR_WARNING << "Could not get " << trackRobotNodes.size() << " entries, input vector is of size " << inputBlobs3D.size() << endl;
        return false;
    }
    filteredBlobs3D.clear();
    std::map<float,size_t> distIndexMap;
    for (size_t j=0; j<inputBlobs3D.size();j++)
    {
        Eigen::Vector3f p = inputBlobs3D[j];
        float minDist = getMinDistToModelNodes(p);

        if (maxModelDistanceMM>0 && minDist>maxModelDistanceMM)
        {
            cout << " skipping blob " << j << ": dist too large:" << minDist << endl;
            continue;
        }

        distIndexMap[minDist] = j;
    }

    // map is sorted internally -> that's fine for us now
    size_t count = 0;
    std::map<float,size_t>::iterator it = distIndexMap.begin();
    while (it!= distIndexMap.end())
    {
        cout << "DIST i " << count << " : " << it->first << " indx:" << it->second << endl;
        if (count<trackRobotNodes.size())
        {
             filteredBlobs3D.push_back(inputBlobs3D[it->second]);
             if (storeIndexMapping)
             {
                 storeIndexMapping->push_back((int)it->second);
             }
        }
        count++;
        it++;
    }
 
    return (filteredBlobs3D.size() == trackRobotNodes.size());
}

bool SimoxHandTrackerModule::paint2dBlobSelection(IplImage* img, std::vector< CvPoint > &allBlobs, std::vector<int> &indexMapping, int size)
{
    if (!img)
        return false;
    for (size_t i=0;i<indexMapping.size();i++)
    {
        int indx = indexMapping[i];
        if (indx<0 || indx>=(int)allBlobs.size())
        {
            VR_ERROR << " Internal error, skipping 2d visu" << endl;
            return false;
        }
        // draw rect
        CvPoint a = allBlobs[indx];
        CvPoint b = allBlobs[indx];
        a.x -= size;
        a.y -= size;
        b.x += size;
        b.y += size;
        cvRectangle(img,a,b,cvScalar(30,30,255),CV_FILLED);
        std::stringstream ss;
        ss << i;
        std::string s = ss.str();
		cvPutText( img, s.c_str(), a, &font, CV_RGB(255,255,255) );
		cout << "Blob " << i << " pos: " << allBlobs[indx].x << "," << allBlobs[indx].y << endl;
    }
    return true;
}

bool SimoxHandTrackerModule::processAllBlobs(float distCamMM_left,float distCamMM_right, std::vector< CvPoint > &points2d_left, std::vector< CvPoint > &points2d_right, std::vector< Eigen::Vector3f > &leftBlobs3D_rootCoord, std::vector< Eigen::Vector3f > &rightBlobs3D_rootCoord, IplImage* resultCVL, IplImage* resultCVR, double window_ratio)
{
	cout << "1" << endl;
	if (!cvStorage)
		return false;
	cout << "2" << endl;

	mutexImgAccess.wait();
	if (!imgsReady)
	{
		mutexImgAccess.post();
		return false;
	}
	cout << "3" << endl;
	// new images are ready, copy them
	Image il = currentImageLeft;
	Image ir = currentImageRight;
	imgsReady = false;
	mutexImgAccess.post();


	IplImage *grayL=(IplImage*) il.getIplImage();
	IplImage *grayR=(IplImage*) ir.getIplImage();
	
	int blobs =0;

	int w_offset=cvRound(0.5*grayL->width*(1.0-window_ratio));
	int h_offset=cvRound(0.5*grayL->height*(1.0-window_ratio));
	int minPixels = 5;
	int maxPixels = 60;
	cout << "processing images, w_offset:" << w_offset << ", h_offset:" << h_offset << ", minPixels:" << minPixels << ", maxPixels:" << maxPixels << endl;
	mutexRobotAccess.wait();
	blobs3D_left_rootCoord.clear();
	blobs3D_right_rootCoord.clear();
	blobs3D_stereo_rootCoord.clear();
	mutexRobotAccess.post();

	cout << " *** searching blobs in left image..." << endl;
	bool leftOK = searchBlobs(grayL,points2d_left,w_offset,h_offset,minPixels,maxPixels,resultCVL);

	if (!leftOK)
	{
		VR_ERROR << "Could not get blobs in left image..." << endl;
		return false;
	}
	cout << " *** searching blobs in right image..." << endl;
	bool rightOK = searchBlobs(grayR,points2d_right,w_offset,h_offset,minPixels,maxPixels,resultCVR);

	if (!rightOK)
	{
		VR_ERROR << "Could not get blobs in right image..." << endl;
		return false;
	}

	cout << " *** checking mono positions..." << endl;
	// check mono positions
	leftOK = get3DPositionMono(true, distCamMM_left, points2d_left, leftBlobs3D_rootCoord);
	if (!leftOK)
	{
		VR_ERROR << "Could not get 3d poses for left blobs ..." << endl;
		return false;
	}
	rightOK = get3DPositionMono(false, distCamMM_right, points2d_right, rightBlobs3D_rootCoord);
	if (!rightOK)
	{
		VR_ERROR << "Could not get 3d poses for right blobs ..." << endl;
		return false;
	}
	return true;
}

bool SimoxHandTrackerModule::processImages()
{
	std::vector< CvPoint > points2d_left; // in cam coord system
	std::vector< CvPoint > points2d_right; // in cam coord system
	std::vector< Eigen::Vector3f > stereoBlobs3D_rootCoord;
	std::vector< Eigen::Vector3f > rightBlobs3D_rootCoord;
	std::vector< Eigen::Vector3f > leftBlobs3D_rootCoord;

	std::vector< Eigen::Vector3f > rightBlobs3DFiltered_rootCoord;
	std::vector< Eigen::Vector3f > leftBlobs3DFiltered_rootCoord;
	std::vector< Eigen::Vector3f > stereoBlobs3DFiltered_rootCoord;
	
	updateCurrentModelPosition();

	// our result images
	ImageOf<PixelRgb> resultL;
	ImageOf<PixelRgb> resultR;
	mutexImgAccess.wait();
	resultL.resize(currentImageLeft);
	resultR.resize(currentImageRight);	
	mutexImgAccess.post();
	IplImage *resultCVL=(IplImage*) resultL.getIplImage();
	IplImage *resultCVR=(IplImage*) resultR.getIplImage();
	cvZero(resultCVL);
	cvZero(resultCVR);


	cout << " todo: using the first finger as distance for all 3d points_rootCoord !!!!" << endl;
	float zL = getFingerToCamDist(true);
	cout << "dist to cam (left):" << zL << endl;
	float zR = getFingerToCamDist(false);
	cout << "dist to cam (right):" << zR << endl;

	if (!processAllBlobs(zL, zR, points2d_left,points2d_right,leftBlobs3D_rootCoord,rightBlobs3D_rootCoord,resultCVL,resultCVR))
		return false;



	

	// remove blob visu, but save status for potential re-enabling visu
	bool showBlobsInViewer = showBlobs3D;
	showTrackedBlobs3DVisu(false);


   
	std::vector<int> indexMappingL;
	std::vector<int> indexMappingR;
	std::vector<int> indexMappingStereo;
	
	if (points2d_left.size() != leftBlobs3D_rootCoord.size())
	{
		cout << "Need all left 2d->3d matches, todo: store 2d points_rootCoord..." << endl;
		return false;
	}

	if (points2d_right.size() != rightBlobs3D_rootCoord.size())
	{
		cout << "Need all right 2d->3d matches, todo: store 2d points_rootCoord..." << endl;
		return false;
	}


	float maxModelDistanceMM = 150.0f; // 15 cm
    cout << " *** getting best matching mapping..." << endl;
	bool leftOK = getBestMatchesDistance(leftBlobs3D_rootCoord, leftBlobs3DFiltered_rootCoord, maxModelDistanceMM, &indexMappingL);
	if (!leftOK)
	{
		VR_ERROR << "Could not get " << trackRobotNodes.size() << " good 3d poses for left blobs ..." << endl;
		return false;
	}
	bool rightOK = getBestMatchesDistance(rightBlobs3D_rootCoord, rightBlobs3DFiltered_rootCoord, maxModelDistanceMM, &indexMappingR);
	if (!rightOK)
	{
		VR_ERROR << "Could not get " << trackRobotNodes.size() << " good 3d poses for right blobs ..." << endl;
		return false;
	}
	// the fingers are usually found at this stage, but sometimes they are not ordered correctly

	// now we are assuming that the blobs are located on a line
	cout << "Assuming that blobs are located on a line!!!" << endl;
	sortPositionsLine(points2d_left, leftBlobs3D_rootCoord, leftBlobs3DFiltered_rootCoord, indexMappingL);
	sortPositionsLine(points2d_right, rightBlobs3D_rootCoord, rightBlobs3DFiltered_rootCoord, indexMappingR);
	
	
    cout << " *** painting results..." << endl;
    // paint resulting 2d blobs
    cout << "Left blobs:" << endl;
	paint2dBlobSelection(resultCVL,points2d_left,indexMappingL,2);
    cout << "Right blobs:" << endl;
	paint2dBlobSelection(resultCVR,points2d_right,indexMappingR,2);
	outBlobImgLeftPort.write(resultL);
    outBlobImgRightPort.write(resultR);

	// use left / right indx mapping to figure out blob matching between left and right cam image
	if (indexMappingL.size()!=indexMappingR.size())
	{
		cout << "mapping sizes do not match..." << endl;
		return false;
	}
	std::vector< CvPoint > points2d_right_ordered;
	std::vector< CvPoint > points2d_left_ordered;

	for (size_t i=0;i<indexMappingL.size();i++)
	{
		points2d_left_ordered.push_back(points2d_left[indexMappingL[i]]);
		points2d_right_ordered.push_back(points2d_right[indexMappingR[i]]);
	}



    cout << " *** getting stereo positions..." << endl;
	bool stereoOK = get3DPositionStereo(points2d_left_ordered, points2d_right_ordered, stereoBlobs3D_rootCoord);
	if (!stereoOK)
	{
		VR_ERROR << "Could not get stereo 3d poses..." << endl;
		return false;
	}

	stereoBlobs3DFiltered_rootCoord = stereoBlobs3D_rootCoord;
	//reverse(stereoBlobs3DFiltered_rootCoord.begin(),stereoBlobs3DFiltered_rootCoord.end());


    //cout << " *** getting best matching mapping (stereo)..." << endl;
	//stereoOK = getBestMatchesDistance(stereoBlobs3D_rootCoord, stereoBlobs3DFiltered_rootCoord, maxModelDistanceMM, &indexMappingStereo);
	//if (!stereoOK)
	//{
	//	VR_ERROR << "Could not get " << trackRobotNodes.size() << " good 3d poses (stereo)..." << endl;
	//	return false;
	//}


    mutexRobotAccess.wait();
	blobs3D_left_rootCoord = leftBlobs3DFiltered_rootCoord;
	blobs3D_right_rootCoord = rightBlobs3DFiltered_rootCoord;
	blobs3D_stereo_rootCoord = stereoBlobs3DFiltered_rootCoord;
	mutexRobotAccess.post();



/*
    cout << "Found " <<  points3d.size() << " points_rootCoord, now trying to match with finegrtips..." << endl;

    // try to match finger tips

    std::vector<bool> found(trackRobotNodes.size(),false);
    float maxDistFingers = 0.1f; // m

    for (size_t i=0; i<points3d.size(); i++)
    {
        int indx = 0;
        cout << "loop " << i << endl;
        float dist = getMinDist(points3d[i],indx);
        cout << "dist " << dist << endl;
        if (dist>=0 && dist < maxDistFingers && indx>=0 && indx<currentModelPositions_rootCoord.size())
        {
            mutexRobotAccess.wait();
            cout << "matching blob " << i << " to finger " << indx << endl;
            cout << "Dist [m]: " << dist << endl;
            cout << " Pos Finger: " << currentModelPositions_rootCoord[indx][0] << "," << currentModelPositions_rootCoord[indx][1] << "," << currentModelPositions_rootCoord[indx][2] << endl;
            cout << " Pos blob  : " << points3d[i][0] << "," << points3d[i][1] << "," << points3d[i][2] << endl;
            found[indx] = true;
            // draw rect
            CvPoint a = points2d[indx];
            CvPoint b = points2d[indx];
            a.x -= 2;
            a.y -= 2;
            b.x += 2;
            b.y += 2;
            mutexRobotAccess.post();
            cvRectangle(resultCVL,a,b,cvScalar(80,80,255),CV_FILLED);
        }
    }
*/

    // draw new positions
    if (showBlobsInViewer)
        showTrackedBlobs3DVisu(true);


    // now checking if the new positions can be matched
    cout << "update tracking information..." << endl;
    bool resOK = false;
	if (localizationUseStereo)
		resOK = updateTracking(blobs3D_stereo_rootCoord);
	else
		resOK = updateTracking(blobs3D_left_rootCoord);  //  (here: only positions retrieved from processing the left cam image)

    return resOK;
}

bool SimoxHandTrackerModule::grabImages()
{
    Image *imgL=inImgLeftPort.read(false);       
    Image *imgR=inImgRightPort.read(false);       
    if(imgL!=NULL && imgR!=NULL)
    {

        IplImage *grayL=(IplImage*) imgL->getIplImage();
        IplImage *grayR=(IplImage*) imgR->getIplImage();

        int gaussian_winsize = 9;
        double thresh = imgThreshold;
        int erode_itr = 0;

        
        cvSmooth(grayL,grayL,CV_GAUSSIAN,gaussian_winsize);
        cvThreshold(grayL,grayL,thresh,255.0,CV_THRESH_BINARY);
        cvEqualizeHist(grayL,grayL); //normalize brightness and increase contrast.
        //cvErode(gray,gray,NULL,8);
        //cvDilate(gray,gray,0,3/*erode_itr*/);
        outImgLeftPort.write(*imgL);

        cvSmooth(grayR,grayR,CV_GAUSSIAN,gaussian_winsize);
        cvThreshold(grayR,grayR,thresh,255.0,CV_THRESH_BINARY);
        cvEqualizeHist(grayR,grayR); //normalize brightness and increase contrast.
        //cvErode(gray,gray,NULL,8);
        //cvDilate(gray,gray,0,3/*erode_itr*/);
        outImgRightPort.write(*imgR);

        mutexImgAccess.wait();
        imgsReady = true;
        currentImageLeft = *imgL;
        currentImageRight = *imgR;
        mutexImgAccess.post();
    }
    return true;
}

bool SimoxHandTrackerModule::respond( const Bottle& command, Bottle& reply )
{
    std::vector<std::string> helpMessages;

    helpMessages.push_back(string(getName().c_str()) + " commands are: \n" );
    helpMessages.push_back("* help");
    helpMessages.push_back("* quit");
    helpMessages.push_back("* info ... list information about internal state");
    helpMessages.push_back("* show coordsystem <jointname> on/off ... enable/disable coordinate system visualization for joint <jointname>");
    helpMessages.push_back("* show robot on/off ... enable/disable robot visualization");
    helpMessages.push_back("* show TCPcoordsystem model/tracked on/off ... enable/disable tcp visualization for model/for tracked poses");
    helpMessages.push_back("* show positions model/tracked on/off ... enable/disable visualization of robotNodes either in model or the tracked positions");
    helpMessages.push_back("* show hand tracked on/off ... enable/disable tracked hand pose visualization");
    helpMessages.push_back("* show blobs3D on/off ... enable/disable visualization of 3D reconstructed image blobs");
    helpMessages.push_back("* set joints <RobotNodeSetName> (j0 j1 ...) ... set joints of given robot node set");
	helpMessages.push_back("* process image ... grab image and process it (search fingertips)");
	helpMessages.push_back("* process blobs ... grab image and print information about all blobs");
    helpMessages.push_back("* set threshold <t>... set segmenting threshold to t");
    helpMessages.push_back("* get threshold ... get the segmenting threshold");
    helpMessages.push_back("* track reset ... Forget all tracking data");
    helpMessages.push_back("* track update (x1 y1 z1 x2 y2 z2 ...) ... Apply new tracking data. The positions x,y,z must be given in the RootCoordSystem as defined in the ini file. The number of positions must be equal to the number of TrackRobotNodes.");
    helpMessages.push_back("* get TCPpose model/tracked ... Returns the current TCP pose (either of model or tracked one): (x,y,z,axis1,axis2,axis3,angle) The pose is given in the rootNode coordinate system.");
    helpMessages.push_back("* get positions model ... Returns a list of the current position of the robot nodes: (x1,y1,z1,x2,y2,z2,...) The position is given in the rootNode coordinate system.");
    helpMessages.push_back("* get positions tracked ... Returns a list of the tracked positions: (x1,y1,z1,x2,y2,z2,...) The position is given in the rootNode coordinate system.");
    helpMessages.push_back("* get name root ... Returns name of root RobotNode (model).");
    helpMessages.push_back("* get name tcp ... Returns name of tcp RobotNode (model).");
    helpMessages.push_back("** RETURN: First return value is always 0/1 indicating success or failure.");
    string helpMessage;
    for (size_t i=0;i<helpMessages.size();i++)
        helpMessage = helpMessage + helpMessages[i] + string("\n");

    reply.clear();
    bool commandProcessed = false;
    bool responseOK = false;
    bool customResponse = false;
    stringstream responseStream;

    if (command.get(0).asString()=="quit")
    {
        reply.addString("quitting");
        return false;
    } else if (command.get(0).asString()=="help")
    {
        cout << helpMessage;
        responseStream << helpMessage;
        responseOK = true;
        commandProcessed = true;
    } else if (command.get(0).asString()=="info") {
        print();
        responseOK = true;
        responseStream << "printing info";
        commandProcessed = true;
    } else if (command.get(0).asString()=="get")
    {
        if (command.get(1).asString()=="threshold")
        {
            reply.addInt(1);//ok
            reply.addDouble((double)imgThreshold);
            customResponse = true;
            commandProcessed = true;
        } else if (command.get(1).asString()=="positions")
        {
            if (command.get(2).asString()=="model")
            {
                updateCurrentModelPosition();
                if (currentModelPositions_rootCoord.size()==0)
                    reply.addInt(0);//failure
                else
                {
                    reply.addInt(1);//ok
                    for (size_t i=0;i<currentModelPositions_rootCoord.size();i++)
                    {
                        reply.addDouble(currentModelPositions_rootCoord[i](0));
                        reply.addDouble(currentModelPositions_rootCoord[i](1));
                        reply.addDouble(currentModelPositions_rootCoord[i](2));
                    }
                }
                customResponse = true;
                commandProcessed = true;
            } else if (command.get(2).asString()=="tracked")
            {

                if (currentTrackedPositions_rootCoord.size()==0)
                    reply.addInt(0);//failure
                else
                {
                    reply.addInt(1);//ok
                    for (size_t i=0;i<currentTrackedPositions_rootCoord.size();i++)
                    {
                        reply.addDouble(currentTrackedPositions_rootCoord[i](0));
                        reply.addDouble(currentTrackedPositions_rootCoord[i](1));
                        reply.addDouble(currentTrackedPositions_rootCoord[i](2));
                    }
                }
                customResponse = true;
                commandProcessed = true;
            }
        } else if (command.get(1).asString()=="TCPpose")
        {
            Eigen::Vector3f axis;
            Eigen::Vector3f pos;
            float angle;
            Eigen::Matrix4f mTcp;
            bool transformMatrix = false;
            if (command.get(2).asString()=="model" && tcpCoordSystem)
            {
                mTcp = tcpCoordSystem->getGlobalPose();
                if (rootCoordSystem)
                    mTcp = rootCoordSystem->toLocalCoordinateSystem(mTcp);
                commandProcessed = true;
                transformMatrix = true;
                customResponse = true;
            } else if (command.get(2).asString()=="tracked")
            {
                mTcp = currentTrackedTCP_rootCoord;
                commandProcessed = true;
                transformMatrix = true;
                customResponse = true;
            } /*else if (command.get(2).asString()=="cartInterface")
			{				
				mTcp = currentTrackedTCP_rootCoord;
				commandProcessed = true;
                customResponse = true;
			}*/
            if (transformMatrix)
            {
                MathTools::eigen4f2axisangle(mTcp,axis,angle);
                axis.normalize();
                pos = MathTools::getTranslation(mTcp);

                reply.addInt(1);//ok
                for (int i=0;i<3;i++)
                    reply.addDouble((double)pos(i));
                for (int i=0;i<3;i++)
                    reply.addDouble((double)axis(i));
                reply.addDouble((double)angle);
            }
        } else if (command.get(1).asString()=="name")
        {
            if (command.get(2).asString()=="root")
            {
                if (rootCoordSystem)
                {
                    reply.addInt(1);
                    reply.addString(rootCoordSystem->getName().c_str());
                } else
                    reply.addInt(0);
                customResponse = true;
                commandProcessed = true;
            } else if (command.get(2).asString()=="tcp")
            {
                if (tcpCoordSystem)
                {
                    reply.addInt(1);
                    reply.addString(tcpCoordSystem->getName().c_str());
                } else
                    reply.addInt(0);
                customResponse = true;
                commandProcessed = true;
            }
        }
    } else if (command.get(0).asString()=="process")
    {
		if (command.get(1).asString()=="image")
		{
			responseOK = processImages();
			if (responseOK)
			{
				cout << "process images OK. Sending calibration offset back:\n";
				cout << currentCalibrationResult << endl;
				reply.addInt(1);
				reply.addDouble( (double)currentCalibrationResult(0,3) );
				reply.addDouble( (double)currentCalibrationResult(1,3) );
				reply.addDouble( (double)currentCalibrationResult(2,3) );
				
			} else
			{
				cout << "process images FAILED\n";
				reply.addInt(0);
				reply.addDouble(0);
				reply.addDouble(0);
				reply.addDouble(0);
			}
			customResponse = true;
			commandProcessed = true;
		} else if (command.get(1).asString()=="blobs")
		{
			responseOK = true;
			printBlobInfo();
			commandProcessed = true;
		}
    } else if (command.get(0).asString()=="show") 
    {
        responseOK = true; // only viewer commands -> if command is processed response is always OK
        if (command.get(1).asString()=="coordsystem")
        {
            ConstString name = command.get(2).asString();
            ConstString onOff = command.get(3).asString();

            responseStream <<"Showing coord system: " << name;
            bool showC = !(onOff == "off");
            if (viewer)
                responseOK = viewer->showCoordSystem(name.c_str(),showC);
            if (responseOK)
                responseStream << " ok";
            else
                responseStream << " failed";
            commandProcessed = true;
        } else if (command.get(1).asString()=="robot")
        {
            ConstString onOff = command.get(2).asString();

            responseStream <<"Showing robot : " << onOff.c_str() << endl;
            bool showC = !(onOff == "off");
            if (viewer)
                viewer->showRobot(showC,VirtualRobot::SceneObject::Collision);
            commandProcessed = true;
        } else if (command.get(1).asString()=="blobs3D")
        {
            ConstString onOff = command.get(2).asString();

            responseStream << "Showing blobs3D : " << onOff.c_str() << endl;
            bool showC = !(onOff == "off");
            showTrackedBlobs3DVisu(showC);
            commandProcessed = true;
        } else if (command.get(1).asString()=="TCPcoordsystem")
        {
            ConstString modelTracked = command.get(2).asString();
            ConstString onOff = command.get(3).asString();

            responseStream <<"Showing TCP coord system: " << modelTracked << ":" << onOff << endl;
            bool showC = !(onOff == "off");
            bool model = (modelTracked == "model" || modelTracked=="Model");
            if (tcpCoordSystem && model && viewer)
                viewer->showCoordSystem(tcpCoordSystem->getName(),showC);
            if (!model)
                showTrackedTCPVisu(showC);
            commandProcessed = true;
        } else if (command.get(1).asString()=="positions")
        {
            ConstString modelTracked = command.get(2).asString();
            ConstString onOff = command.get(3).asString();

            responseStream <<"Showing positions: " << modelTracked << ":" << onOff << endl;
            bool showC = !(onOff == "off");
            bool model = (modelTracked == "model" || modelTracked=="Model");
            if (tcpCoordSystem && model && viewer)
            {
                for (size_t i=0;i<trackRobotNodes.size();i++)
                {
                    viewer->showCoordSystem(trackRobotNodes[i]->getName(),showC);
                }
            }
            if (!model)
            {
                showTrackedPositionsVisu(showC);
            }
            commandProcessed = true;
        } else if (command.get(1).asString()=="hand")
        {
            ConstString modelTracked = command.get(2).asString();
            ConstString onOff = command.get(3).asString();

            responseStream <<"Showing hand: " << modelTracked << ":" << onOff << endl;
            bool showC = !(onOff == "off");
            bool model = (modelTracked == "model" || modelTracked=="Model");
            if (!model && viewer)
            {
                showTrackedEEF = showC;
                if (showC)
                    viewer->setEEFVisu("EEFPose",eef->getName(),VirtualRobot::SceneObject::Full);
                else
                    viewer->removeEEFVisu("EEFPose");
            }
            commandProcessed = true;
        }
	
    } else if (command.get(0).asString()=="set")
    {
        if (command.get(1).asString()=="joints")
        {
            std::string rns = command.get(2).asString().c_str();
            yarp::os::Bottle *pos = command.get(3).asList();
            if (pos && !pos->isNull())
            {
		
                std::vector< float > jv;
                for (int i=0;i<pos->size();i++)
                {
                    jv.push_back( (float)(pos->get(i).asDouble()) );
                }
                responseStream << "Updating joint values:";
                responseOK = setJointValues(rns,jv);
                if (responseOK)
                    responseStream << " ok";
                else
                    responseStream << " failed";

                commandProcessed = true;
            }
        } else if (command.get(1).asString()=="threshold") 
        {

            imgThreshold =  (float)command.get(2).asDouble();
            responseStream << "Setting threshold to " << imgThreshold;
            responseOK = true;
            commandProcessed = true;
        }
	
    } else if (command.get(0).asString()=="track")
    {
        if (command.get(1).asString()=="reset")
        {
            responseStream << "Resetting tracking data:";
            responseOK = resetTracking();
            if (responseOK)
                responseStream << " ok";
            else
                responseStream << " failed";
            commandProcessed = true;
        } else if (command.get(1).asString()=="update")
        {
            yarp::os::Bottle *pos = command.get(2).asList();
            if (pos && !pos->isNull())
            {
                if (pos->size() != (int)trackRobotNodes.size()*3)
                {
                    responseStream << "Wrong dimension, need " << trackRobotNodes.size() << " positions x y z. But received " << pos->size()/3;
                } else
                {
                    std::vector< Eigen::Vector3f > positions;
                    for (int i=0;i<pos->size()/3;i++)
                    {
                        Eigen::Vector3f p;
                        p(0) = (float)(pos->get(i*3+0).asDouble());
                        p(1) = (float)(pos->get(i*3+1).asDouble());
                        p(2) = (float)(pos->get(i*3+2).asDouble());
                        positions.push_back(p);
                    }
                    responseStream << "Updating positions:";
                    responseOK = updateTracking(positions);
                    if (responseOK)
                        responseStream << " ok";
                    else
                        responseStream << " failed";
                }
            } else
            {
                responseStream << "Internal error, could not grab positions_rootCoord form stream...";
            }
            commandProcessed = true;
        }

    } else
        commandProcessed = false;

    if (!customResponse)
    {
        reply.addInt((int)responseOK);

        if (!commandProcessed)
        {
            responseStream << "Unknown command: " << command.toString().c_str() << "\n Try 'help' \n";
            cout << helpMessage;
        }
    }
    return true;
}

bool SimoxHandTrackerModule::interruptModule()
{
    handlerPort.interrupt();
    printf ("INTERRUPT\n");
    return true;
}

bool SimoxHandTrackerModule::close()
{	
    handlerPort.close();
    robotDeviceLeftArm.close();
    robotDeviceRightArm.close();
    robotDeviceHead.close();
    robotDeviceTorso.close();
    encRightArm = encLeftArm = encHead = encTorso = NULL;

    inImgLeftPort.close();
    outImgLeftPort.close();
    outBlobImgLeftPort.close();
    inImgRightPort.close();
    outImgRightPort.close();
    outBlobImgRightPort.close();

    cvReleaseMemStorage(&cvStorage);
    cvStorage = NULL;
    return true;
}
void SimoxHandTrackerModule::showTrackedTCPVisu(bool enable)
{
    showTrackedTCP = enable;
    if (!viewer)
        return;
    std::string sphereFile = "objects/sphere10.xml";
    VirtualRobot::ColorPtr colTCP(new VirtualRobot::VisualizationFactory::Color(VirtualRobot::VisualizationFactory::Color::Gray()));
    if (enable)
    {
        viewer->setObject("Tracked TCP",sphereFile,colTCP);
        viewer->showObjectCoordSystem("Tracked TCP",true);
        updateTrackingVisu();
    } else
        viewer->removeObject("Tracked TCP");
}

void SimoxHandTrackerModule::showTrackedPositionsVisu(bool enable)
{
    showTrackedRobotNodes = enable;
    if (!viewer)
        return;
    std::string sphereFile = "objects/sphere10.xml";
    VirtualRobot::ColorMap cm(VirtualRobot::ColorMap::eHot);
    trackedObjectVisuNames.clear();
    for (size_t i=0;i<trackRobotNodes.size();i++)
    {
        //viewer->showCoordSystem(trackRobotNodes[i]->getName(),true);
        stringstream ss;
        ss << "Object" << i;
        trackedObjectVisuNames.push_back(ss.str());
        VirtualRobot::VisualizationFactory::Color c = cm.getColor((float)i / (float)(trackRobotNodes.size()-1));
        VirtualRobot::ColorPtr cp(new VirtualRobot::VisualizationFactory::Color(c));
        if (enable)
        {
            viewer->setObject(ss.str(),sphereFile,cp);
            viewer->showObjectCoordSystem(ss.str(),true);
        } else
            viewer->removeObject(ss.str());
    }

    if (enable)
    {
        updateTrackingVisu();
    }
}

void SimoxHandTrackerModule::showTrackedBlobs3DVisu(bool enable)
{
    cout << "SHOW BLOBS ";
    if (enable)
        cout << " ON " << endl;
    else
        cout << " OFF " << endl;
    showBlobs3D = enable;
    if (!viewer)
        return;

    std::string sphereFile = "objects/sphere10.xml";
    VirtualRobot::VisualizationFactory::Color c = VirtualRobot::VisualizationFactory::Color::Red();
    VirtualRobot::ColorPtr cp(new VirtualRobot::VisualizationFactory::Color(c));
	std::vector< Eigen::Vector3f > pts;
	if (localizationUseStereo)
		pts = blobs3D_stereo_rootCoord;
	else
		pts = blobs3D_left_rootCoord;
	cout << "Stereo:" << localizationUseStereo << ", Nr blobs:  " << pts.size() << endl;
    for (size_t i=0;i<pts.size();i++)
    {
        //viewer->showCoordSystem(trackRobotNodes[i]->getName(),true);
        stringstream ss;
        ss << "Blob_" << i;
        std::string name = ss.str();
        cout << " blob name:" << name << endl;
        if (enable)
        {
            viewer->setObject(name,sphereFile,cp);
            viewer->showObjectCoordSystem(name,true);

            // set position
            mutexRobotAccess.wait();
            Eigen::Matrix4f blobPos = Eigen::Matrix4f::Identity();
            Eigen::Vector3f posBlobGlobal = pts[i];
            if (rootCoordSystem)
                posBlobGlobal = rootCoordSystem->toGlobalCoordinateSystemVec(posBlobGlobal);
            blobPos.block(0,3,3,1) = posBlobGlobal;
            mutexRobotAccess.post();
            cout << " blob blobPos:\n" << blobPos << endl;
            viewer->setObjectPose(name,blobPos);


        } else
            viewer->removeObject(name);
    }
}

double SimoxHandTrackerModule::getPeriod()
{
    // 50 fps
    return 0.02;
}


bool SimoxHandTrackerModule::loadRobot( const std::string &filename )
{
    VR_INFO << "Loading robot from " << filename << endl;
    try
    {
        // we don't need the visualization model
        robot = RobotIO::loadRobot(filename);
    }
    catch (VirtualRobotException &e)
    {
        VR_ERROR << " ERROR while creating robot" << endl;
        VR_ERROR << e.what();
        return false;
    }

    if (!robot)
    {
        VR_ERROR << " ERROR while creating robot" << endl;
        return false;
    }
    
    robotFile = filename;

    return true;
}

void SimoxHandTrackerModule::print()
{
    cout << "***** SimoxHandTrackerModule *****" << endl;
    if (isLeft)
        cout << "Configured for the LEFT hand" << endl;
    else
        cout << "Configured for the RIGHT hand" << endl;

    cout << "Robot: ";
    if (robot)
        cout << robot->getName();
    else
        cout << "<none>";
    cout << endl;

    cout << "Tracking the following RobotNodes:" << endl;
    for (size_t i=0;i<trackRobotNodes.size();i++)
    {
        cout << "* " << trackRobotNodes[i]->getName() << endl;
    }
}

bool SimoxHandTrackerModule::setJointValues(const std::string &rns, std::vector< float> &jv)
{
    if (!robot)
        return false;
    mutexRobotAccess.wait();
    if (!robot->hasRobotNodeSet(rns))
    {
        mutexRobotAccess.post();
        return false;
    }

    RobotNodeSetPtr rnsp = robot->getRobotNodeSet(rns);
    if (rnsp->getSize() != jv.size())
    {
        VR_WARNING << "RobotNodeSet " << rns << " has " << rnsp->getSize() << " joints, but received " << jv.size() << " values, aborting.." << endl;
        mutexRobotAccess.post();
        return false;
    }
    rnsp->setJointValues(jv);
    mutexRobotAccess.post();
    // viewer internally protects robot access, so its fine just to call it
    return viewer->setJoints(rns,jv);
}

bool SimoxHandTrackerModule::resetTracking()
{
    currentTrackedPositions_rootCoord.clear();
    return true;
}

void SimoxHandTrackerModule::createPermutationsVisit(std::vector<int> &processing, int k, std::vector< std::vector<int> > &storeResult)
{
    static int level = -1;
    level = level+1; processing[k] = level;
    int N = (int)processing.size();
    if (level == N)
    {
        std::vector<int> r = processing;
        for (size_t z=0;z<processing.size();z++)
            r[z] -= 1;
        storeResult.push_back(r);
    }
    else
        for (int i = 0; i < N; i++)
            if (processing[i] == 0)
                createPermutationsVisit(processing, i, storeResult);

    level = level-1; processing[k] = 0;
}

std::vector< std::vector<int> > SimoxHandTrackerModule::createPermutations(unsigned int size)
{
    std::vector< std::vector<int> > result;
    std::vector<int> processing(size,0);
    createPermutationsVisit(processing,0,result);

    /*for (size_t i = 0;i<result.size();i++)
	{
		cout << "perm " << i << ":";
		for (size_t j=0;j<result[i].size();j++)
		{
			cout << result[i][j] << ",";
		}
		cout << endl;
	}*/
    return result;
}



bool SimoxHandTrackerModule::orderTrackedPositions()
{
    mutexRobotAccess.wait();

    if (!trackRobotNodes.size() == currentTrackedPositions_rootCoord.size())
    {
        mutexRobotAccess.post();
        return false;
    }

    int nrTracks = (int)currentTrackedPositions_rootCoord.size();
    std::vector< std::vector<int> > permutation = createPermutations(nrTracks);
    std::vector< Eigen::Vector3f > bestPerm = currentTrackedPositions_rootCoord;
    float bestDist = 1000000.0f;

    for (int i=0;i<(int)permutation.size();i++)
    {
        std::vector< Eigen::Vector3f > currentPerm;
        for (size_t j=0;j<permutation[i].size();j++)
        {
            currentPerm.push_back(currentTrackedPositions_rootCoord[ permutation[i][j] ] );
        }
        float dist = getPointsToFingertipsDistance(currentPerm,false);
        if (dist<bestDist)
        {
            bestDist = dist;
            bestPerm = currentPerm;
            //cout << "perm: " << i << ", dist:" << dist << endl;
        }
    }
    cout << "Reordering tracking points_rootCoord form " << endl;
    for (size_t j=0;j<currentTrackedPositions_rootCoord.size();j++)
    {
        cout << "(" << currentTrackedPositions_rootCoord[j](0) << ", " << currentTrackedPositions_rootCoord[j](1) << "," << currentTrackedPositions_rootCoord[j](2)  << ") / ";
    }
    cout << endl  << " to ";
    for (size_t j=0;j<bestPerm.size();j++)
    {
        cout << "(" <<  bestPerm[j](0) << ", " << bestPerm[j](1) << "," << bestPerm[j](2)  << ") / ";
    }
    cout << endl;

    currentTrackedPositions_rootCoord = bestPerm;
    mutexRobotAccess.post();
    return true;
}

float SimoxHandTrackerModule::getPointsToFingertipsDistance(std::vector< Eigen::Vector3f > &points_rootCoord, bool lockMutex)
{
    if (lockMutex)
        mutexRobotAccess.wait();

    if (points_rootCoord.size() != currentModelPositions_rootCoord.size())
    {
        VR_ERROR << "internal error" << endl;
        if (lockMutex)
            mutexRobotAccess.post();
        return -1.0f;
    }
    float res = 0;
    for (size_t i=0;i<points_rootCoord.size();i++)
    {
        Eigen::Vector3f v = points_rootCoord[i] - currentModelPositions_rootCoord[i];
        float d = v.norm();
        res += d*d;
    }
    if (lockMutex)
        mutexRobotAccess.post();
    return sqrtf(res);
}


bool SimoxHandTrackerModule::updateTrackingVisu( )
{
    if (!viewer || !trackedObjectVisuNames.size() == currentTrackedPositions_rootCoord.size())
        return false;
    for (size_t i=0;i<currentTrackedPositions_rootCoord.size();i++)
    {
        //Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        //m.block(0,3,3,1) = currentTrackedPositions_rootCoord[i]; // <- set only position, without orientation

        // better visualization: use orientation of model
        mutexRobotAccess.wait();
        Eigen::Matrix4f fingerPos = trackRobotNodes[i]->getGlobalPose();
        // set position of tracked fingers, but remain orientation
        Eigen::Vector3f posFingerGlobal = currentTrackedPositions_rootCoord[i];
        //cout << "vec posFinger_root:\n" << posFingerGlobal << endl;
        if (rootCoordSystem)
            posFingerGlobal = rootCoordSystem->toGlobalCoordinateSystemVec(posFingerGlobal);
        //cout << "vec posFinger_global:\n" << posFingerGlobal << endl;
       // cout << "mat posFinger_global (model):\n" << fingerPos << endl;
        fingerPos.block(0,3,3,1) = posFingerGlobal;
        //cout << "mat posFinger_global (vision):\n" << fingerPos << endl;
        mutexRobotAccess.post();
        if (showTrackedRobotNodes)
            viewer->setObjectPose(trackedObjectVisuNames[i],fingerPos);
    }
    if (showTrackedTCP)
	{
		Eigen::Matrix4f m = currentTrackedTCP_rootCoord;
		if (rootCoordSystem)
			m = rootCoordSystem->toGlobalCoordinateSystem(m);
        viewer->setObjectPose("Tracked TCP",m);
	}

    mutexRobotAccess.post();
    return true;
}

bool SimoxHandTrackerModule::updateTracking( const std::vector< Eigen::Vector3f > &positions_rootCoord )
{
    mutexRobotAccess.wait();
    if (positions_rootCoord.size() != trackRobotNodes.size())
    {
        VR_WARNING << "Invalid number of positions_rootCoord: Expecting " << trackRobotNodes.size() << " positions_rootCoord, but received " << positions_rootCoord.size() << endl;
        mutexRobotAccess.post();
        return false;
    }
    if (currentTrackedPositions_rootCoord.size()==0)
        currentTrackedPositions_rootCoord = positions_rootCoord;
    else
    {
        cout << "todo..." << endl;
        currentTrackedPositions_rootCoord = positions_rootCoord;
    }

    mutexRobotAccess.post();
    cout << " ************ update current model positions_rootCoord" << endl;
    updateCurrentModelPosition();
    cout << " ************ order tracked positions_rootCoord" << endl;
    orderTrackedPositions();
    cout << " ************ update tracking visualization" << endl;
    updateTrackingVisu();
    cout << " ************ matching positions_rootCoord" << endl;
    return matchPositions();
}

bool SimoxHandTrackerModule::updateCurrentModelPosition()
{
    mutexRobotAccess.wait();
    currentModelPositions_rootCoord.clear();
    for (size_t i=0;i<trackRobotNodes.size();i++)
    {
        // get pose
        Eigen::Matrix4f m = trackRobotNodes[i]->getGlobalPose();
        // transform to root coord system
        if (rootCoordSystem)
            m = rootCoordSystem->toLocalCoordinateSystem(m);
        // get position
        Eigen::Vector3f pos = MathTools::getTranslation(m);
        currentModelPositions_rootCoord.push_back(pos);
    }
    mutexRobotAccess.post();
    return true;
}
Eigen::Matrix4f SimoxHandTrackerModule::estimateTCP(std::vector<Eigen::Vector3f > &positionsFingers_rootCoord)
{
    if (!robot || trackRobotNodes.size()==0)
        return Eigen::Matrix4f::Identity();

    mutexRobotAccess.wait();
	Eigen::Matrix4f result = tcpCoordSystem->getGlobalPose();
    cout << "TCP GLOBAL POSE (MODEL)\n" << tcpCoordSystem->getGlobalPose() << endl;
    robot->setUpdateVisualization(false);
    RobotConfigPtr c = robot->getConfig();
    Eigen::Matrix4f gpRob = robot->getGlobalPose();
    std::vector< MathTools::Quaternion > poses;
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    for (size_t i=0;i<trackRobotNodes.size();i++)
    {
  
		robot->setConfig(c);
		robot->setGlobalPose(gpRob);
        Eigen::Matrix4f fingerPos = trackRobotNodes[i]->getGlobalPose();
		

        // set position of tracked fingers, but remain orientation
        Eigen::Vector3f posFingerGlobal = positionsFingers_rootCoord[i]; // == currentTrackedPositions_rootCoord[i]
        if (rootCoordSystem)
            posFingerGlobal = rootCoordSystem->toGlobalCoordinateSystemVec(posFingerGlobal);
        fingerPos.block(0,3,3,1) = posFingerGlobal;
        robot->setGlobalPoseForRobotNode(trackRobotNodes[i],fingerPos);
        //cout << " fingerPos global:\n" << fingerPos << endl;
		//cout << " should be: fingerPos global:(ok)\n" << trackRobotNodes[i]->getGlobalPose() << endl;
        Eigen::Matrix4f globalPose = tcpCoordSystem->getGlobalPose();
        cout << "  ESTIMATED (" << i << ") TCP GLOBAL POSE (MODEL)\n" << globalPose << endl;

        MathTools::Quaternion q = MathTools::eigen4f2quat(globalPose);
        poses.push_back(q);
        position += globalPose.block(0,3,3,1);
        cout << "  ESTIMATED Quat: " << q.x << "," << q.y << "," << q.z << ", w:" << q.w << endl;
    }
    position /= trackRobotNodes.size();
    MathTools::Quaternion qAvg = MathTools::getMean(poses);
    cout << "  (disabled) ESTIMATED Quat Avg: " << qAvg.x << "," << qAvg.y << "," << qAvg.z << ", w:" << qAvg.w << endl;
    //result = MathTools::quat2eigen4f(qAvg);
    // just move tcp pose, remain original orientation
    result.block(0,3,3,1) = position;

    robot->setConfig(c);
    robot->setGlobalPose(gpRob);
    robot->setUpdateVisualization(true);

    cout << "  ESTIMATED (AVERAGE) TCP GLOBAL POSE\n" << result << endl;
     if (rootCoordSystem)
        result = rootCoordSystem->toLocalCoordinateSystem(result);
    cout << "  ESTIMATED (AVERAGE) TCP root coords:\n" << result << endl;
    currentTrackedTCP_rootCoord = result;
    mutexRobotAccess.post();

    return result;
}

bool SimoxHandTrackerModule::matchPositions()
{
    if (currentTrackedPositions_rootCoord.size() != trackRobotNodes.size())
    {
        return false;
    }

    updateCurrentModelPosition();

    mutexRobotAccess.wait();

    // compute estimated TCP
    Eigen::Matrix4f tcpTracked_rootCoord = estimateTCP(currentTrackedPositions_rootCoord); // in root coord
    Eigen::Matrix4f tcpModel_rootCoord = tcpCoordSystem->getGlobalPose();
    if (rootCoordSystem)
            tcpModel_rootCoord = rootCoordSystem->toLocalCoordinateSystem(tcpModel_rootCoord);
    std::vector< Eigen::Vector3f > trackedPos = currentTrackedPositions_rootCoord;
    std::vector< Eigen::Vector3f > modelPos = currentModelPositions_rootCoord;
    // add tcp position to ensure correct orientation
    trackedPos.push_back(MathTools::getTranslation(tcpTracked_rootCoord));
    modelPos.push_back(MathTools::getTranslation(tcpModel_rootCoord));
    
    /*cout << "Tracked positions (root):\n";
    for (size_t i=0;i<trackedPos.size();i++)
    {
    	cout << trackedPos[i] << endl;
    }
    cout << "Model positions (root):\n";
    for (size_t i=0;i<trackedPos.size();i++)
    {
    	cout << modelPos[i] << endl;
    }*/

    // in root coord system
    Eigen::Matrix4f tr = getTransformationMatrix(trackedPos,modelPos);
    cout << "(TEST, currently disabled!!) MinDistMatching: Transformation from observed points_rootCoord to current model points_rootCoord:" << endl;
    cout << tr << endl;
    
    
	//cout << "######## Check this!!!!" << endl;
	cout << " tcp pose model (root):\n" << tcpModel_rootCoord << endl;
	cout << " tcp pose vision (root):\n" << currentTrackedTCP_rootCoord << endl;
	tr = tcpModel_rootCoord.inverse() * currentTrackedTCP_rootCoord;
	cout << "TCP model<->visu transformation: Transformation from observed tcp (derived from finger positions) to current model tcp:" << endl;
	cout << tr << endl;
	cout << "Hmm, this is the transformation in TCP coords, we need a transformation in global coords (or root):\n:" << endl;
	
	
	currentCalibrationResult.setIdentity();
	currentCalibrationResult(0,3) = tcpModel_rootCoord(0,3) - currentTrackedTCP_rootCoord(0,3);
	currentCalibrationResult(1,3) = tcpModel_rootCoord(1,3) - currentTrackedTCP_rootCoord(1,3);
	currentCalibrationResult(2,3) = tcpModel_rootCoord(2,3) - currentTrackedTCP_rootCoord(2,3);
	cout << "displacement in root coords: (FINAL CALIBRATION VALUES)|n";
	cout << currentCalibrationResult << endl;
	
    if (eef && eef->getTcp())
    {
        cout << "updating visu..." << endl;
        Eigen::Matrix4f pose;
        RobotNodePtr rn = eef->getTcp();
        cout << "tcp global:" << endl;
        cout << rn->getGlobalPose();
        Eigen::Matrix4f tcpRoot = rn->getGlobalPose();
        if (rootCoordSystem)
            tcpRoot = rootCoordSystem->toLocalCoordinateSystem(tcpRoot);
        pose = tcpRoot * tr;//.inverse();
        if (rootCoordSystem)
            pose = rootCoordSystem->toGlobalCoordinateSystem(pose);
        cout << "\n tracked eef pose global:" << endl;
        cout << pose;
        //pose = rn->getGlobalPose();//Eigen::Matrix4f::Identity();
        if (showTrackedEEF)
            viewer->setEEFVisuPose("EEFPose",pose);
    }
    mutexRobotAccess.post();

    return true;
}

Eigen::Vector3f SimoxHandTrackerModule::getMean(const std::vector< Eigen::Vector3f > &p)
{
    Eigen::Vector3f result = Eigen::Vector3f::Zero();
    for (size_t i=0;i<p.size();i++)
    {
        result += p[i];
    }
    if (p.size()>0)
        result /= (float)p.size();
    return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f SimoxHandTrackerModule::getTransformationMatrix(const std::vector< Eigen::Vector3f > &p1, const std::vector< Eigen::Vector3f > &p2)
{
    // todo: check Eigen::Geometry::umeyama
    // faster, more reliable?

    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();

    if (p1.size() != p2.size() || p1.size()==0 )
    {
        VR_ERROR << "Wrong input?!" << endl;
        return result;
    }
    Eigen::Vector3f mean1 = getMean(p1);
    Eigen::Vector3f mean2 = getMean(p2);

    // create matrices from p1 and p2
    Eigen::MatrixXf m1(4,p1.size());
    m1.setZero();
    Eigen::MatrixXf m2(4,p1.size());
    m2.setZero();
    for (size_t i=0;i<p1.size();i++)
    {
        m1.block(0,i,3,1) = p1[i] - mean1;
        m2.block(0,i,3,1) = p2[i] - mean2;
    }

    // Correlation Matrix C = m1*m2'
    Eigen::Matrix3f C = (m1 * m2.transpose ()).topLeftCorner<3, 3>();

    // Compute the Singular Value Decomposition
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    if (U.determinant()*V.determinant()<0)
        for (int a=0;a<3;a++)
            V(a, 2) *= -1;

    // R = V * U'
    Eigen::Matrix3f R = V*U.transpose();

    // Transformation
    result.topLeftCorner<3, 3>() = R;
    Eigen::Vector3f Rc = R*mean1.head<3>();
    result.block<3,1>(0,3) = mean2.head<3>() - Rc;

    return result;
}

void SimoxHandTrackerModule::printBlobInfo()
{
	std::vector< CvPoint > points2d_left; // in cam coord system
	std::vector< CvPoint > points2d_right; // in cam coord system
	std::vector< Eigen::Vector3f > stereoBlobs3D_rootCoord;
	std::vector< Eigen::Vector3f > rightBlobs3D_rootCoord;
	std::vector< Eigen::Vector3f > leftBlobs3D_rootCoord;

	std::vector< Eigen::Vector3f > rightBlobs3DFiltered_rootCoord;
	std::vector< Eigen::Vector3f > leftBlobs3DFiltered_rootCoord;
	std::vector< Eigen::Vector3f > stereoBlobs3DFiltered_rootCoord;

	updateCurrentModelPosition();

	// our result images
	ImageOf<PixelRgb> resultL;
	ImageOf<PixelRgb> resultR;
	mutexImgAccess.wait();
	resultL.resize(currentImageLeft);
	resultR.resize(currentImageRight);	
	mutexImgAccess.post();
	IplImage *resultCVL=(IplImage*) resultL.getIplImage();
	IplImage *resultCVR=(IplImage*) resultR.getIplImage();
	cvZero(resultCVL);
	cvZero(resultCVR);


	cout << " Expecting an object in 1m distance" << endl;
	float zL = 1000.0f;
	float zR = 1000.0f;

	if (!processAllBlobs(zL, zR, points2d_left,points2d_right,leftBlobs3D_rootCoord,rightBlobs3D_rootCoord,resultCVL,resultCVR,0.4))
		return;

	std::vector<int> indexMappingL;
	for (int i=0;i < (int)points2d_left.size();i++)
		indexMappingL.push_back(i);
	std::vector<int> indexMappingR;
	for (int i=0;i < (int)points2d_right.size();i++)
		indexMappingR.push_back(i);
	cout << " *** painting results..." << endl;
	// paint resulting 2d blobs
	cout << "Left blobs:" << endl;
	paint2dBlobSelection(resultCVL,points2d_left,indexMappingL,2);
	cout << "Right blobs:" << endl;
	paint2dBlobSelection(resultCVR,points2d_right,indexMappingR,2);
	outBlobImgLeftPort.write(resultL);
	outBlobImgRightPort.write(resultR);
}
