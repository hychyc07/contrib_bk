
#include "SimoxHandEyeCalibrationGui.h"
#include <VirtualRobot/MathTools.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;
using namespace VirtualRobot;


SimoxHandEyeCalibrationGui:: SimoxHandEyeCalibrationGui(bool isLeft)
{
    this->isLeft = isLeft;
    iCubIkSolverControl = NULL;
    iCubGazeControl = NULL;
    encLeftArm = encRightArm = encHead = encTorso = NULL;
    posLeftArm = posRightArm = posHead = posTorso = NULL;
}



bool SimoxHandEyeCalibrationGui::configure( yarp::os::ResourceFinder &rf )
{
    moduleName = rf.check("name",
                          Value("SimoxHandEyeCalibrationGui"),
                          "module name (string)").asString();
    setName(moduleName.c_str());

    robotBase = rf.check("robot",
                         Value("icubSim"),
                         "robot name (string)").asString();

    VR_INFO << "Using robot base string " << robotBase << endl;

    // rpc handler port
    handlerPortName =  "/";
    handlerPortName += getName();         // use getName() rather than a literal
	handlerPortName += "/rpc:i";
    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        return false;
    }

    handPoses.clear();
    Value *val;
    int nrEntries = 7;

    std::string handPosesName = isLeft?"HandPosesLeft":"HandPosesRight";

    if (rf.check(handPosesName.c_str(),val))
    {
        if (val->isList())
        {
            yarp::os::Bottle *valList = val->asList();
            for (int v=0;v<valList->size();v++)
            {
                yarp::os::Bottle *configs = valList->get(v).asList();
                if (configs->size() == 2)
                {
                    std::string name = configs->get(0).asString().c_str();
                    std::vector<float> vec;
                    yarp::os::Bottle *jv = configs->get(1).asList();
                    if (nrEntries>0 && jv->size() !=nrEntries)
                    {
                        VR_ERROR << "Expecting 7d cart pose (" << name <<"), aborting..." << endl;
                    } else
                    {
                        VR_INFO << "Cartesian pose found:";
                        cout << name << ": ";
                        for (int i=0;i<jv->size();i++)
                        {
                            float v = (float)jv->get(i).asDouble();
                            vec.push_back(v);
                            cout << v << ", ";
                        }
                        cout << endl;
                        SimoxHandEyeCalibrationWindow::handPose ac;
                        ac.name = name;
                        ac.cartPos_root = vec;
                        handPoses.push_back(ac);
                    }
                }
            }
        } else
        {
            VR_ERROR << "Wrong HandPoses list size: must be 2: (nameString (7d pose as list) )" << endl;
        }
    } else
    {
        VR_WARNING << "No HandPoses found..." << endl;
    }
    preshapes.clear();
    nrEntries = 9;
    if (rf.check("Preshapes",val))
    {
        if (val->isList())
        {
            yarp::os::Bottle *valList = val->asList();
            for (int v=0;v<valList->size();v++)
            {
                yarp::os::Bottle *configs = valList->get(v).asList();
                if (configs->size() == 2)
                {
                    std::string name = configs->get(0).asString().c_str();
                    std::vector<float> vec;
                    yarp::os::Bottle *jv = configs->get(1).asList();
                    if (nrEntries>0 && jv->size() !=nrEntries)
                    {
                        VR_ERROR << "Expecting 9d joint value vector (" << name <<"), aborting..." << endl;
                    } else
                    {
                        VR_INFO << "Preshape found:";
                        cout << name << ": ";
                        for (int i=0;i<jv->size();i++)
                        {
                            float v = (float)jv->get(i).asDouble();
                            vec.push_back(v);
                            cout << v << ", ";
                        }
                        cout << endl;
                        SimoxHandEyeCalibrationWindow::preshape ps;
                        ps.name = name;
                        ps.fingerJV = vec;
                        preshapes.push_back(ps);
                    }
                }
            }
        } else
        {
            VR_ERROR << "Wrong Preshapes list size: must be 2: (nameString (9d jv list) )" << endl;
        }
    } else
    {
        VR_WARNING << "No Preshapes found..." << endl;
    }

    attach(handlerPort);                  // attach to port

    setupConnections();

    controlWindow.reset(new SimoxHandEyeCalibrationWindow(this,handPoses,preshapes));


    return true;
}

bool SimoxHandEyeCalibrationGui::setupConnections()
{    

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
    posLeftArm = NULL;
    if (robotDeviceLeftArm.open(optionsLeftArm))
    {
        robotDeviceLeftArm.view(encLeftArm);
        robotDeviceLeftArm.view(posLeftArm);
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
    posRightArm = NULL;
    if (robotDeviceRightArm.open(optionsRightArm))
    {
        robotDeviceRightArm.view(encRightArm);
        robotDeviceRightArm.view(posRightArm);
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
    posHead = NULL;
    if (robotDeviceHead.open(optionsHead))
    {
        robotDeviceHead.view(encHead);
        robotDeviceHead.view(posHead);
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
    posTorso = NULL;
    if (robotDeviceTorso.open(optionsTorso))
    {
        robotDeviceTorso.view(encTorso);
        robotDeviceTorso.view(posTorso);
        int axesTorso;
        if (!encTorso || !encTorso->getAxes(&axesTorso) || axesTorso<=0) {
            printf("Could not get encoder values from Torso\n");
            close();
            return false;
        }
        jointValuesTorso.resize(axesTorso,0.0);
    }


    // connect to hand tracker
    std::string clientName1 =  "/";
    clientName1 += getName();
    clientName1 += "/SimoxHandTracker/rpc:o";
    std::string serverName1 = "/SimoxHandTracker/rpc:i";
    bool result = true;
    if (!simoxHandTrackerPort.open(clientName1.c_str()))
    {
        VR_ERROR << "Could not open port " << clientName1 << endl;
        result = false;
    } else
    {
        if (!yarp.connect(clientName1.c_str(),serverName1.c_str()))
        {
            VR_ERROR << "Could not connect to simox hand tracker..." << endl;
            result = false;
        }
    }
    
    
    // connect to icub IK solver
    Property option("(device cartesiancontrollerclient)");
    std::string remoteStr = "/" + robotBase;
    std::string armString = "left_arm";
    if (!isLeft)
        armString = "right_arm";
    remoteStr += "/cartesianController/";
    remoteStr += armString;
    option.put("remote",remoteStr.c_str());
    std::string localString ="/cartesian_client/";
    localString += armString;
    option.put("local",localString.c_str());

    if (!iCubIkSolverClient.open(option))
    {
        VR_ERROR << " Could not open " << remoteStr << " for Cartesian solver client" << endl;
        return false;
    }

    // open the view
    iCubIkSolverClient.view(iCubIkSolverControl);
    if (!iCubIkSolverControl)
    {
        VR_ERROR << "Could not get iCub Cartesian controller..." << endl;
        return false;
    }

    iCubIkSolverControl->setTrajTime(1.0);

    // get the torso dofs
    Vector newDof, curDof;
    iCubIkSolverControl->getDOF(curDof);
    newDof=curDof;

    // no hip
    newDof[0]=0;
    newDof[1]=0;//0;// todo:disbale torso?
    newDof[2]=0;
    // send the request for dofs reconfiguration
    iCubIkSolverControl->setDOF(newDof,curDof);



    // connect to gaze control
    Property optionGaze;
    optionGaze.put("device","gazecontrollerclient");
    optionGaze.put("remote","/iKinGazeCtrl");
    optionGaze.put("local","/client/simoxHandTracker");
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

    return true;
}

void SimoxHandEyeCalibrationGui::lookToTCP()
{
    if (!iCubGazeControl)
    {
        VR_ERROR << "No gaze control connection..." << endl;
        return;
    }
    std::vector<float> tcp = getTCPPosition(true);
    Vector fp(3);
    fp[0]= tcp[0];
    fp[1]= tcp[1];
    fp[2]= tcp[2] + 0.1;// look above tcp, so that the fingertips are centered...

    iCubGazeControl->lookAtFixationPoint(fp); // move the gaze to the desired fixation point
    iCubGazeControl->waitMotionDone();        // wait until the operation is done

	encHead->getEncoders(jointValuesHead.data());
	cout << "Head joints:" << endl;
	for (int i=0;i<(int)jointValuesHead.size();i++)
		cout << jointValuesHead[i] << ", " << endl;
	
	cout << "todo: test straight eyes..." << endl;
	jointValuesHead[4] = 0;
	jointValuesHead[5] = 0;
	posHead->positionMove(jointValuesHead.data());

	//posHead->positionMove()
    //Vector x(3);
    //igaze->getFixationPoint(x);                             // retrieve the current fixation point

    //cout<<"final error = "<<ctrl::norm(fp-x)<<endl;         // return a measure of the displacement error
}

void SimoxHandEyeCalibrationGui::lookToTable()
{
	if (!iCubGazeControl)
	{
		VR_ERROR << "No gaze control connection..." << endl;
		return;
	}

	Vector fp(3);
	fp[0]= -1,0; // 1 m in front
	fp[1]= 0; // centered
	fp[2]= 0; // at height of waist

	iCubGazeControl->lookAtFixationPoint(fp); // move the gaze to the desired fixation point
	iCubGazeControl->waitMotionDone();        // wait until the operation is done

	encHead->getEncoders(jointValuesHead.data());
	cout << "Head joints:" << endl;
	for (int i=0;i<(int)jointValuesHead.size();i++)
		cout << jointValuesHead[i] << ", " << endl;
	cout << "todo: test straight eyes..." << endl;
	jointValuesHead[4] = 0;
	jointValuesHead[5] = 0;
	posHead->positionMove(jointValuesHead.data());
}


void SimoxHandEyeCalibrationGui::lookToHomePos()
{
	if (!iCubGazeControl)
	{
		VR_ERROR << "No gaze control connection..." << endl;
		return;
	}
	Vector fp(3);
	fp[0]= 0;
	fp[1]= 0;
	fp[2]= 0;

	iCubGazeControl->lookAtAbsAngles(fp); // move the gaze to zero position
}


bool SimoxHandEyeCalibrationGui::updateModule()
{
    static bool firstRun = true;
    if (firstRun)
    {
        firstRun = false;
        if (controlWindow)
            controlWindow->firstRun();
    } else
    {
        if (controlWindow)
        {
            controlWindow->updateWindow();
        }
    }

    if (controlWindow && controlWindow->wasClosed())
    {
        cout << "Window closed, exiting..." << endl;
        return false;
    }
    if (qApp)
        qApp->processEvents();
    return true;
}

bool SimoxHandEyeCalibrationGui::respond( const Bottle& command, Bottle& reply )
{
    std::vector<std::string> helpMessages;

    helpMessages.push_back(string(getName().c_str()) + " commands are: \n" );
    helpMessages.push_back("help");
    helpMessages.push_back("quit");

    string helpMessage;
    for (size_t i=0;i<helpMessages.size();i++)
        helpMessage = helpMessage + helpMessages[i] + string("\n");

    bool eaten = false;
    reply.clear();

    if (!controlWindow)
    {
        reply.addString ("Window closed, could not perform any actions. Quitting...");
        return false;
    }

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("Help printed...");
        eaten = true;
    }


    if (!eaten || reply.isNull())
    {
        reply.addString("unknown command:\n");
        reply.addString(command.toString());
        reply.addString("\nTry 'help'");
        cout << helpMessage;
    }
    return true;
}

bool SimoxHandEyeCalibrationGui::interruptModule()
{
    handlerPort.interrupt();
    printf ("INTERRUPT\n");
    return true;
}

bool SimoxHandEyeCalibrationGui::close()
{
    handlerPort.close();
    iCubIkSolverClient.close();
    simoxHandTrackerPort.close();

    if (controlWindow)
        controlWindow->quit();

    // reports a memory leak in Coin's font manager?!
    //SoQt::done();

    return true;
}

double SimoxHandEyeCalibrationGui::getPeriod()
{
    // 50 fps
    return 0.02;
}

bool SimoxHandEyeCalibrationGui::moveArm( std::vector<float> handPose_7fd_rootCoords )
{
    if (!iCubIkSolverControl)
    {
        VR_ERROR << "No cart interface?!" << endl;
        return false;
    }
    yarp::sig::Vector xd(3);
    xd(0) = handPose_7fd_rootCoords[0];
    xd(1) = handPose_7fd_rootCoords[1];
    xd(2) = handPose_7fd_rootCoords[2];
    yarp::sig::Vector od(4);
    od(0) = handPose_7fd_rootCoords[3];
    od(1) = handPose_7fd_rootCoords[4];
    od(2) = handPose_7fd_rootCoords[5];
    od(3) = handPose_7fd_rootCoords[6];
    yarp::sig::Vector xdhat(3);
    yarp::sig::Vector odhat(4);
    yarp::sig::Vector qdhat(10);
    bool ikOK = iCubIkSolverControl->askForPose(xd,od,xdhat,odhat,qdhat);
    if (!ikOK)
    {
        cout << "could not get IK solution..." << endl;
        return false;
    }
    double l = 0;
    for (int i=0;i<3;i++)
    {
        xdhat(i) -= xd(i);
        l += xdhat(i)*xdhat(i);
    }
    l = sqrt(l);
    if (l>0.01)
    {
        cout << " IK solution too far away:" << l << "m\n";
        return false;
    }

    cout << "Moving arm to (";
    for (int i=0;i<6;i++)
        cout << handPose_7fd_rootCoords[i] << ",";
    cout << handPose_7fd_rootCoords[6] << endl;
    return iCubIkSolverControl->goToPose(xd,od);
}


bool SimoxHandEyeCalibrationGui::moveFingerJoints(std::vector<float> &fingerJV)
{
    yarp::dev::IEncoders *armEnc =encLeftArm;
    yarp::sig::Vector *jv = &jointValuesLeftArm;
    yarp::dev::IPositionControl *posArm = posLeftArm;
    if (!isLeft)
    {
        armEnc = encRightArm;
        jv = &jointValuesRightArm;
        posArm = posRightArm;
    }
    if (fingerJV.size()!=9)
    {
        VR_ERROR << "wrong sized joint vector..." << endl;
        return false;
    }
    if (!armEnc || !posArm)
    {
        cout << "No arm encoder, aborting..." << endl;
        return false;
    }
    if (!armEnc->getEncoders(jv->data()) || jv->size()!=16)
    {
        cout << "error retrieving arm joint values..." << endl;
        return false;
    }
    yarp::sig::Vector newJV = *jv;
    for (int i=0;i<9;i++)
    {
        newJV[i+7] = fingerJV[i];
    }
    return posArm->positionMove(newJV.data());
}


bool SimoxHandEyeCalibrationGui::sendToHandTracker(yarp::os::Bottle &cmd,yarp::os::Bottle &response)
{
    bool writeOK = simoxHandTrackerPort.write(cmd,response);
    cout << "Sending command " << cmd.toString() << " to simoxHandTrackerPort:" << writeOK << endl;
    cout << "Response: " << response.toString().c_str() << endl;
    if (!writeOK)
        return false;

    bool responseOK = response.get(0).asInt()==1;
    return (responseOK);
}

std::string SimoxHandEyeCalibrationGui::getRootName()
{
    yarp::os::Bottle cmd;
    cmd.addString("get");
    cmd.addString("name");
    cmd.addString("root");
    Bottle response;
    bool writeOK = simoxHandTrackerPort.write(cmd,response);
    if (!writeOK)
        return "<none>";

    bool responseOK = response.get(0).asInt()==1;
    if (!responseOK)
        return "<none>";

    return response.get(1).asString().c_str();
}

float SimoxHandEyeCalibrationGui::getThreshold()
{
    yarp::os::Bottle cmd;
    cmd.addString("get");
    cmd.addString("threshold");
    Bottle response;
    bool writeOK = simoxHandTrackerPort.write(cmd,response);
    if (!writeOK)
        return 0.0f;

    bool responseOK = response.get(0).asInt()==1;
    if (!responseOK)
        return 0.0f;

    return (float)response.get(1).asDouble();
}
std::string SimoxHandEyeCalibrationGui::getTcpName()
{
    yarp::os::Bottle cmd;
    cmd.addString("get");
    cmd.addString("name");
    cmd.addString("tcp");
    Bottle response;
    bool writeOK = simoxHandTrackerPort.write(cmd,response);
    if (!writeOK)
        return "<none>";

    bool responseOK = response.get(0).asInt()==1;
    if (!responseOK)
        return "<none>";

    return response.get(1).asString().c_str();
}

std::string SimoxHandEyeCalibrationGui::getRNSName()
{
    // todo: generic....
    if (isLeft)
        return "Hip Left Arm";
    else
        return "Hip Right Arm";
}

std::vector<float> SimoxHandEyeCalibrationGui::getTCPPosition( bool model )
{
    if (model)
    {
        yarp::sig::Vector x;
        yarp::sig::Vector o;
        if (iCubIkSolverControl->getPose(x,o))
        {
            std::vector<float> result;
            for (int i=0;i<3;i++)
                result.push_back( (float)x(i) );
            for (int i=0;i<4;i++)
                result.push_back( (float)o(i) );
#if 1
            //test difference between simox model and iCartSolver from iCub
            yarp::os::Bottle cmd;
            cmd.addString("get");
            cmd.addString("TCPpose");
            cmd.addString("model");
            Bottle response;
            simoxHandTrackerPort.write(cmd,response);

            std::vector<float> result2;
            for (int i=1;i<8;i++)
                result2.push_back( (float)response.get(i).asDouble());
            /*cout << "ICUB iCartSolver: tcp pose 7D:" << endl;
			for (int i=0;i<7;i++)
				cout << result[i] << ",";
			cout << endl;            
			cout << "Simox model: tcp pose 7D:" << endl;
			for (int i=0;i<7;i++)
				cout << result2[i] << ",";*/
            //cout << "\nDiff:" << endl;
            float xd = result2[0] - result[0]*1000.0f;
            float yd = result2[1] - result[1]*1000.0f;
            float zd = result2[2] - result[2]*1000.0f;

            float posDiff = sqrtf(xd*xd + yd*yd + zd*zd);
            if (posDiff>10.0f)
            {
                cout << "TCP Simox<->iCubCart diff is large (pos): " << posDiff << " mm" << endl;
            }
            MathTools::Quaternion q1 = MathTools::axisangle2quat(Eigen::Vector3f(result[3],result[4],result[5]),result[6]);
            MathTools::Quaternion q2 = MathTools::axisangle2quat(Eigen::Vector3f(result2[3],result2[4],result2[5]),result2[6]);
            MathTools::Quaternion diffQ = MathTools::getDelta(q1,q2);
            float diffAngle = MathTools::getAngle(diffQ);
            if (diffAngle>0.1f)
                cout << "TCP Simox<->iCubCart diff is large (angle): " << diffAngle << " rad" << endl;

#endif
            return result;
        } else
        {
            return std::vector<float>(7,0);
        }
    } else
    {
        yarp::os::Bottle cmd;
        cmd.addString("get");
        cmd.addString("TCPpose");
        //if (model)
        //    cmd.addString("model");
        //else
        cmd.addString("tracked");
        Bottle response;
        bool writeOK = simoxHandTrackerPort.write(cmd,response);
        if (!writeOK)
            return std::vector<float>(7,0);

        bool responseOK = response.get(0).asInt()==1;
        if (!responseOK)
        {
            VR_ERROR << "Could not get TCP pose from simox hand tracker?!" << endl;
            return std::vector<float>(7,0);
        }

        std::vector<float> result;
        for (int i=1;i<8;i++)
            result.push_back( (float)response.get(i).asDouble());
        return result;
    }
}
