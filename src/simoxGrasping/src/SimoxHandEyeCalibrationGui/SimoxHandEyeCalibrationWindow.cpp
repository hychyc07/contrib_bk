
#include "SimoxHandEyeCalibrationWindow.h"
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/SceneObject.h>
#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/XML/ObjectIO.h>


#include "ui_SimoxHandEyeCalibration.h"

#include "SimoxHandEyeCalibrationGui.h"


using namespace VirtualRobot;
using namespace yarp::os;

SimoxHandEyeCalibrationWindow:: SimoxHandEyeCalibrationWindow(SimoxHandEyeCalibrationGui *communication, std::vector<handPose> &handPoses, std::vector<preshape> &preshapes)
    :QMainWindow(NULL)
{
    pingValue = 0;
    isClosed = false;
    this->handPoses = handPoses;
    this->preshapes = preshapes;
    timeUpdateWindowS = 0.5;
    lastWindowUpdateS = Time::now();

    VR_ASSERT_MESSAGE(communication,"Need a communication interface...");
    com = communication;

    setupUI();

    SoQt::show(this);
}

SimoxHandEyeCalibrationWindow::~ SimoxHandEyeCalibrationWindow()
{
    delete UI;
}



void  SimoxHandEyeCalibrationWindow::setupUI()
{
    UI = new Ui::MainWindowHandEyeCalibration;

    UI->setupUi(this);

    // setup robot control
    connect(UI->pushButtonMoveArmConfig, SIGNAL(clicked()), this, SLOT(executeArmConfig()));
    for (size_t i=0;i<handPoses.size();i++)
        UI->comboBoxArmConfig->addItem(QString(handPoses[i].name.c_str()));
    for (size_t i=0;i<preshapes.size();i++)
        UI->comboBoxPreshape->addItem(QString(preshapes[i].name.c_str()));
    connect(UI->comboBoxArmConfig, SIGNAL(activated(int)), this, SLOT(selectArmConfig(int)));
    connect(UI->comboBoxPreshape, SIGNAL(activated(int)), this, SLOT(selectPreshape(int)));
	connect(UI->radioButtonLookHome, SIGNAL(clicked()), this, SLOT(moveHead()));
	connect(UI->radioButtonLookTable, SIGNAL(clicked()), this, SLOT(moveHead()));
	connect(UI->radioButtonLookTCP, SIGNAL(clicked()), this, SLOT(moveHead()));
	connect(UI->radioButtonStopHead, SIGNAL(clicked()), this, SLOT(moveHead()));

    // setup display options
    UI->checkBoxVisuRobotModel->setChecked(true);
    UI->checkBoxVisuTCPCoordSystem->setChecked(true);
    UI->checkBoxVisuFingerCoordSystems->setChecked(true);
    UI->checkBoxVisuTCPEstimated->setChecked(true);
    UI->checkBoxVisuFingerEstimated->setChecked(true);
    UI->checkBoxVisuHandEstimated->setChecked(true);
    UI->checkBoxBlobs3D->setChecked(true);
    connect(UI->checkBoxVisuRobotModel, SIGNAL(clicked()), this, SLOT(updateVisuOptions()));
    connect(UI->checkBoxVisuTCPCoordSystem, SIGNAL(clicked()), this, SLOT(updateVisuOptions()));
    connect(UI->checkBoxVisuFingerCoordSystems, SIGNAL(clicked()), this, SLOT(updateVisuOptions()));
    connect(UI->checkBoxVisuTCPEstimated, SIGNAL(clicked()), this, SLOT(updateVisuOptions()));
    connect(UI->checkBoxVisuFingerEstimated, SIGNAL(clicked()), this, SLOT(updateVisuOptions()));
    connect(UI->checkBoxVisuHandEstimated, SIGNAL(clicked()), this, SLOT(updateVisuOptions()));
    connect(UI->checkBoxBlobs3D, SIGNAL(clicked()), this, SLOT(updateVisuOptions()));

    connect(UI->pushButtonProcessAllBlobs, SIGNAL(clicked()), this, SLOT(processAllBlobs()));
   // setup hand tracking
    connect(UI->pushButtonSearchFingertips, SIGNAL(clicked()), this, SLOT(searchFingerTips()));
    connect(UI->horizontalSliderSegThreshold, SIGNAL(valueChanged(int)), this, SLOT(segThresholdChanged(int)));
}


void  SimoxHandEyeCalibrationWindow::closeEvent(QCloseEvent *event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void  SimoxHandEyeCalibrationWindow::quit()
{
    std::cout << " SimoxHandEyeCalibrationWindow: Closing" << std::endl;
    this->close();
    isClosed = true;
    SoQt::exitMainLoop();
}


int  SimoxHandEyeCalibrationWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}

void SimoxHandEyeCalibrationWindow::firstRun()
{

    // setup info box
    updateInfo();

    // update visualization
    updateVisuOptions();

    if (com)
    {
        float t = com->getThreshold();
        t = t / 255.0f * UI->horizontalSliderSegThreshold->maximum();
        UI->horizontalSliderSegThreshold->setValue(t);
    }
    updateSegmentationInfo();
}

void  SimoxHandEyeCalibrationWindow::ping()
{
    pingValue++;
    if (pingValue>5)
    {
        pingValue = 0;
        //UI->checkBoxConnected->setChecked(true);
    }
}

bool SimoxHandEyeCalibrationWindow::wasClosed()
{
    return isClosed;
}

void SimoxHandEyeCalibrationWindow::executeArmConfig()
{
    if (handPoses.size()==0)
    {
        VR_WARNING << "No hand poses, aborting..." << endl;
        return;
    }
    int nr = UI->comboBoxArmConfig->currentIndex();
    if (nr<0 || nr>=(int)handPoses.size())
    {
        VR_WARNING << "Invalid selection?! " << nr << ", aborting..." << endl;
        return;
    }
    if (!com)
    {
        VR_ERROR << "No com" << endl;
        return;
    }
    if (!com->moveArm(handPoses[nr].cartPos_root))
    {
        VR_WARNING << "Could not move arm to requested pose" << endl;
    }
}

void SimoxHandEyeCalibrationWindow::selectPreshape( int i )
{
    if (preshapes.size()==0)
    {
        VR_WARNING << "No preshapes, aborting..." << endl;
        return;
    }
    int nr = UI->comboBoxPreshape->currentIndex();
    if (nr<0 || nr>=(int)preshapes.size())
    {
        VR_WARNING << "Invalid selection?! " << nr << ", aborting..." << endl;
        return;
    }
    if (!com)
    {
        VR_ERROR << "No com" << endl;
        return;
    }
    if (!com->moveFingerJoints(preshapes[nr].fingerJV))
    {
        VR_WARNING << "Could not move hand to requested preshape" << endl;
    }
}
void SimoxHandEyeCalibrationWindow::selectArmConfig( int i )
{
    // nothing to do...
}

void SimoxHandEyeCalibrationWindow::updateVisuOptions()
{
    Bottle cmd;
    cmd.addString("show");
    cmd.addString("robot");
    if (UI->checkBoxVisuRobotModel->isChecked())
        cmd.addString("on");
    else
        cmd.addString("off");
    sendToHandTracker(cmd);

    cmd.clear();
    cmd.addString("show");
    cmd.addString("TCPcoordsystem");
    cmd.addString("model");
    if (UI->checkBoxVisuTCPCoordSystem->isChecked())
        cmd.addString("on");
    else
        cmd.addString("off");
    sendToHandTracker(cmd);

    cmd.clear();
    cmd.addString("show");
    cmd.addString("positions");
    cmd.addString("model");
    if (UI->checkBoxVisuFingerCoordSystems->isChecked())
        cmd.addString("on");
    else
        cmd.addString("off");
    sendToHandTracker(cmd);

    // show cam system
    cmd.clear();
    cmd.addString("show");
    cmd.addString("coordsystem");
    cmd.addString("EyeRightCam");
    if (UI->checkBoxVisuFingerCoordSystems->isChecked())
        cmd.addString("on");
    else
        cmd.addString("off");
    sendToHandTracker(cmd);
    cmd.clear();
    cmd.addString("show");
    cmd.addString("coordsystem");
    cmd.addString("EyeLeftCam");
    if (UI->checkBoxVisuFingerCoordSystems->isChecked())
        cmd.addString("on");
    else
        cmd.addString("off");
    sendToHandTracker(cmd);


    cmd.clear();
    cmd.addString("show");
    cmd.addString("TCPcoordsystem");
    cmd.addString("tracked");
    if (UI->checkBoxVisuTCPEstimated->isChecked())
        cmd.addString("on");
    else
        cmd.addString("off");
    sendToHandTracker(cmd);

    cmd.clear();
    cmd.addString("show");
    cmd.addString("positions");
    cmd.addString("tracked");
    if (UI->checkBoxVisuFingerEstimated->isChecked())
        cmd.addString("on");
    else
        cmd.addString("off");
    sendToHandTracker(cmd);

    cmd.clear();
    cmd.addString("show");
    cmd.addString("hand");
    cmd.addString("tracked");
    if (UI->checkBoxVisuHandEstimated->isChecked())
        cmd.addString("on");
    else
        cmd.addString("off");
    sendToHandTracker(cmd);

    cmd.clear();
    cmd.addString("show");
    cmd.addString("blobs3D");
    if (UI->checkBoxBlobs3D->isChecked())
        cmd.addString("on");
    else
        cmd.addString("off");
    sendToHandTracker(cmd);
}


bool SimoxHandEyeCalibrationWindow::sendToHandTracker(yarp::os::Bottle &cmd)
{
	yarp::os::Bottle response;
    return com->sendToHandTracker(cmd,response);
}

bool SimoxHandEyeCalibrationWindow::sendToHandTracker(yarp::os::Bottle &cmd, yarp::os::Bottle &response)
{
    return com->sendToHandTracker(cmd,response);
}

void SimoxHandEyeCalibrationWindow::searchFingerTips()
{
    cout << "processing image..." << endl;
    Bottle cmd,response;
    cmd.addString("process");
    cmd.addString("image");
    sendToHandTracker(cmd,response);
    if (response.get(0).asInt() == 1)
    {
    	cout << "response OK" << endl;
    	UI->doubleSpinBoxOffsetX->setValue( response.get(1).asDouble() );
    	UI->doubleSpinBoxOffsetY->setValue( response.get(2).asDouble() );
    	UI->doubleSpinBoxOffsetZ->setValue( response.get(3).asDouble() );
	} else
    	cout << "response FAILED" << endl;
}

void SimoxHandEyeCalibrationWindow::processAllBlobs()
{
	cout << "processing image (search all blobs)..." << endl;
	Bottle cmd;
	cmd.addString("process");
	cmd.addString("blobs");
	sendToHandTracker(cmd);
}

void SimoxHandEyeCalibrationWindow::segThresholdChanged( int i )
{
    float t = (float)UI->horizontalSliderSegThreshold->value() / (float)UI->horizontalSliderSegThreshold->maximum();
    if (t<0)
        t = 0;
    if (t>1.0f)
        t = 1.0f;
    t = t*255.0f;
    Bottle cmd;
    cmd.addString("set");
    cmd.addString("threshold");
    cmd.addDouble((double)t);
    sendToHandTracker(cmd);
    updateSegmentationInfo();
}
void SimoxHandEyeCalibrationWindow::updateWindow()
{
    // reduce frequency of updates

    double delay = Time::now() - lastWindowUpdateS;
    if (delay > timeUpdateWindowS)
    {
        //cout << "updateStart" << endl;
        //cout << "s" << flush;
        lastWindowUpdateS = Time::now();
        if (com)
        {
            //cout << "update1" << endl;
            //cout << "1" << flush;
            std::vector<float> tcpModel = com->getTCPPosition(true);
            //cout << "update2" << endl;
            //cout << "2" << flush;
            updateInfoTcpPose(true, tcpModel);
        }
        //cout << "update3" << endl;
        //cout << "3" << flush;

		
        //cout << "updateEnd" << endl;
        //cout << "e" << flush;
    }

}

void SimoxHandEyeCalibrationWindow::moveHead()
{
	if (UI->radioButtonLookHome->isChecked())
	{
		com->lookToHomePos();
	} else if (UI->radioButtonLookTCP->isChecked())
	{
		com->lookToTCP();
	} else if (UI->radioButtonLookTable->isChecked())
	{
		com->lookToTable();
	}
}

void SimoxHandEyeCalibrationWindow::updateSegmentationInfo()
{
    float t = (float)UI->horizontalSliderSegThreshold->value() / (float)UI->horizontalSliderSegThreshold->maximum();
    if (t<0)
        t = 0;
    if (t>1.0f)
        t = 1.0f;
    t = t*255.0f;
    QString s;
    s = "Segmentation Threshold: ";
    s += QString::number(t);
    UI->labelSegThreshold->setText(s);

}
QString SimoxHandEyeCalibrationWindow::createUIString(const std::string &start,std::vector<float> &vec)
{
    QString p1 = start.c_str();
    for (size_t i=0;i<vec.size();i++)
    {
        p1 += QString::number( (double)vec[i],'f',3);
        if (i != vec.size()-1)
            p1 += ", ";
    }
    return p1;
}

void SimoxHandEyeCalibrationWindow::updateInfoJoints(std::vector<float> jointsArm, std::vector<float> jointsHand, std::vector<float> jointsHip, std::vector<float> jointsHead)
{
    UI->labelJointsArm->setText(createUIString("Joints (arm): ",jointsArm));
    UI->labelJointsHand->setText(createUIString("Joints (hand): ",jointsHand));
    UI->labelJointsHip->setText(createUIString("Joints (hip): ",jointsHip));
    UI->labelJointsHead->setText(createUIString("Joints (head): ",jointsHead));
}

void SimoxHandEyeCalibrationWindow::updateInfoTcpPose(bool model, std::vector<float> pose7D)
{
    if (model)
    {
        UI->labelTCPPosModel->setText(createUIString("TCP Position (model): ",pose7D));
    } else
    {
        UI->labelTCPPosVision->setText(createUIString("TCP Position (vision): ",pose7D));
    }
}

void SimoxHandEyeCalibrationWindow::updateInfo()
{
    std::string rootName = com->getRootName();
    std::string tcpName = com->getTcpName();
    std::string rnsName = com->getRNSName();
    QString rootS = "Root: ";
    rootS += rootName.c_str();
    QString rnsS = "Kinematic Chain (RNS): ";
    rnsS += rnsName.c_str();
    QString tcpS = "TCP: ";
    tcpS += tcpName.c_str();
    UI->labelRoot->setText(rootS);
    UI->labelRNS->setText(rnsS);
    UI->labelTCP->setText(tcpS);
    std::vector<float> tcpModel = com->getTCPPosition(true);
    updateInfoTcpPose(true, tcpModel);
    std::vector<float> tcpTracked = com->getTCPPosition(false);
    updateInfoTcpPose(false, tcpTracked);	
}


