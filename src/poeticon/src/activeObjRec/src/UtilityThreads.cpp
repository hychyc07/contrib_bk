#include "iCub/UtilityThreads.h"
#include "iCub/ObjRecModule.h"
#include "iCub/Util.h"

#include <opencv2/highgui/highgui.hpp>

const double CreateObjectThread::BOX_SIZE = 0.05;
const double CreateObjectThread::COLOR_GREEN[] = {0,1,0}; // green
const double CreateObjectThread::COLOR_RED[]   = {1,0,0}; // red
const double CreateObjectThread::COLOR_BLUE[]  = {0,0,1}; // blue
const double CreateObjectThread::COLOR_YELLOW[]= {1,1,0}; // yellow
const double CreateObjectThread::COLOR_WHITE[] = {1,1,1}; // white

void OpenHandThread::gotoStartPos()
{
    // move arm to start position, don't move fingers
    for (int i = 0; i < NUM_ARM_JOINTS; i++)
    {
        Util::safePositionMove(i, startPosition[i], armCtrl, limArm);
    }

    Util::safePositionMove(0, 0, torsoCtrl, limTorso);
    Util::safePositionMove(1, 0, torsoCtrl, limTorso);
    Util::safePositionMove(2, 0, torsoCtrl, limTorso);
}

void OpenHandThread::openHand()
{
    // move fingers
    for (int i = NUM_ARM_JOINTS; i < NUM_JOINTS; i++)
    {
        Util::safePositionMove(i, startPosition[i], armCtrl, limArm);
    }
}

void OpenHandThread::run()
{
    bool armDone = 0; 
    bool torsoDone = 0;
    armCtrl->checkMotionDone(&armDone);
    torsoCtrl->checkMotionDone(&torsoDone);
    if (armDone && torsoDone)
    {
        module->sendActionDoneMsg("hand opened");
        stop();
    }
} 


bool CalibGazeThread::threadInit()
{
    viewWindowOffsetY = rf->check("viewWindowOffsetY", Value(.0), "distance from image center").asInt();
    viewWindowOffsetX = rf->check("viewWindowOffsetX", Value(.0), "distance from image center").asInt();

    double gazeOffsetX = rf->check("gazeOffsetX", Value(0.0), "offset towards fingers").asDouble();
    double gazeOffsetY = rf->check("gazeOffsetY", Value(0.0), "offset orthogonal to palm").asDouble();
    double gazeOffsetZ = rf->check("gazeOffsetZ", Value(0.0), "offset in opposite direction of thumb").asDouble();

    sliderPosGazeOffsetX = 20+gazeOffsetX*100;
    sliderPosGazeOffsetY = 20+gazeOffsetY*100;
    sliderPosGazeOffsetZ = 20+gazeOffsetZ*100;

    //cv::startWindowThread();

    return true;
}

void CalibGazeThread::threadRelease()
{
    cv::destroyAllWindows();
}

void CalibGazeThread::run()
{
    std::string wndName = "Gaze calibration";
    cv::namedWindow(wndName, 0);
    cv::createTrackbar("GazeOffsetX", wndName, &sliderPosGazeOffsetX, 40); 
    cv::createTrackbar("GazeOffsetY", wndName, &sliderPosGazeOffsetY, 40); 
    cv::createTrackbar("GazeOffsetZ", wndName, &sliderPosGazeOffsetZ, 40); 
    cv::waitKey(5); 

    double gazeOffsetX = double(sliderPosGazeOffsetX-20) / 100.0;
    double gazeOffsetY = double(sliderPosGazeOffsetY-20) / 100.0;
    double gazeOffsetZ = double(sliderPosGazeOffsetZ-20) / 100.0;

    module->handGazeControl->setGazeOffsets(gazeOffsetX, gazeOffsetY, gazeOffsetZ);

    std::cout << "gazeOffsets: "
        << gazeOffsetX << " "
        << gazeOffsetY << " "
        << gazeOffsetZ << std::endl;

    cv::Mat showWindow = module->getCameraImage();
    if (showWindow.empty())
        return;

    // draw crosshair
    int cx = showWindow.cols/2 + viewWindowOffsetX;
    int cy = showWindow.rows/2 + viewWindowOffsetY;
    cv::Scalar color = cv::Scalar(0, 255, 255);
    cv::line(showWindow, 
            cv::Point(cx, cy - 5), 
            cv::Point(cx, cy + 5), 
            color, 2);
    cv::line(showWindow, 
            cv::Point(cx - 5, cy), 
            cv::Point(cx + 5, cy), 
            color, 2);

    cv::imshow(wndName, showWindow);
    cv::waitKey(5);

} 

bool HandControlThread::threadInit()
{
    // gui settings
    sliderPosElevation = 0;
    sliderPosRotation = 0;
    sliderPosWAngDist = 2;
    sliderPosWEuclDist = 1;
    sliderPosWMoveDist = 0;
    sliderPosHandPosX = 30;
    sliderPosHandPosY = 0 ;
    sliderPosHandPosZ = 20;
    sliderPosKNN = 100;

    cv::startWindowThread();

    return true;
}

void HandControlThread::threadRelease()
{
    module->handGazeControl->stop(); 
    cv::destroyWindow("Hand Control");
}

void HandControlThread::run()
{
    cv::namedWindow("Hand Control", 0);
    cv::createTrackbar("Elevation", "Hand Control", &sliderPosElevation, 90); 
    cv::createTrackbar("Rotation", "Hand Control", &sliderPosRotation, 360); 
    cv::createTrackbar("Hand x", "Hand Control", &sliderPosHandPosX, 100); 
    cv::createTrackbar("Hand y", "Hand Control", &sliderPosHandPosY, 100); 
    cv::createTrackbar("Hand z", "Hand Control", &sliderPosHandPosZ, 100); 
    cv::createTrackbar("w Ang Dist", "Hand Control", &sliderPosWAngDist, 10); 
    cv::createTrackbar("w Eucl Dist", "Hand Control", &sliderPosWEuclDist, 10); 
    cv::createTrackbar("w Move Dist", "Hand Control", &sliderPosWMoveDist, 10); 
    cv::createTrackbar("kNN", "Hand Control", &sliderPosKNN, 500); 
    cv::waitKey(5); 

    // weights
    double wAng = sliderPosWAngDist;
    double wEucl = sliderPosWEuclDist;
    double wMove = sliderPosWMoveDist;

    module->handGazeControl->setWeights(wAng, wEucl, wMove);

    // k nearest neighbors
    module->handGazeControl->setK(sliderPosKNN);

    // target hand position
    Vector optimalHandX(3);
    optimalHandX[0] = -sliderPosHandPosX / 100.0; 
    optimalHandX[1] = sliderPosHandPosY / 100.0; 
    optimalHandX[2] = sliderPosHandPosZ / 100.0; 
    //std::cout << "setting hand target pos to " << optimalHandX.toString() << std::endl;
    module->handGazeControl->setHandTargetPos(optimalHandX);

    double ed = sliderPosElevation;
    double rd = sliderPosRotation;

    module->handGazeControl->lookAtViewpoint(ed, rd);

    double crnt_e, crnt_r;
    module->getHandGazeAngles(crnt_e,crnt_r);
    double err = Util::calcCentralAngle(crnt_e, ed, crnt_r, rd);

    int width = 10;
    std::cout << std::setw(width) << std::left << "e/r:" 
        << std::setprecision(1) << std::fixed 
        << crnt_e << "°\t" << crnt_r << "°"
        << "\terr: " << err << "°" << std::endl;

}

void CreateObjectThread::run()
{
    module->sendActionDoneMsg("object created");
    stop();
}

bool CreateObjectThread::createObject()
{
    if (objectName.empty()) // default = red box
        objectName = "redbox";

    std::string textureFile = textureName + ".bmp";

    // make sure there are no objecs already in the scene
    std::cout << "deleting old objects..." << std::endl;
    Bottle& deleteObjs = worldPort.prepare();
    deleteObjs.clear();
    deleteObjs.addString("world");
    deleteObjs.addString("del");
    deleteObjs.addString("all");
    worldPort.write();

    Time::delay(1.0);

    // get position of hand
    Bottle result;
    Bottle query;
    query.clear();
    query.addString("world");
    query.addString("get");
    query.addString("rhand");
    worldPortSync.write(query, result);
    if (result.size() == 0)
    {
        std::cout << "Error: Could not get right hand coordinates!" << std::endl;
        return false;
    }
    double handX = result.get(0).asDouble();
    double handY = result.get(1).asDouble();
    double handZ = result.get(2).asDouble();

    // create object at hand position
    std::cout << "creating new object..." << std::endl;
    //Bottle& output = worldPort->prepare();
    //output.clear();
    Bottle output;
    if (objectName == "redbox" 
            || objectName == "bluebox" 
            || objectName == "yellowbox" 
            || objectName == "whitebox" 
            || objectName == "greenbox")
    {
        // create standard simulator box
        output.addString("world");
        output.addString("mk");
        output.addString("box");
        output.addDouble(BOX_SIZE);
        output.addDouble(BOX_SIZE);
        output.addDouble(BOX_SIZE);
        output.addDouble(handX);
        output.addDouble(handY + BOX_SIZE/2 + 0.04);
        //output.addDouble(handZ + 0.04);
        output.addDouble(handZ + 0.02);
        //output.addDouble(handX + BOX_SIZE/2 + 0.03);
        //output.addDouble(handY + 0.01);
        //output.addDouble(handZ + 0.01);
        if (objectName == "redbox")   { output.addDouble(COLOR_RED[0]);   output.addDouble(COLOR_RED[1]);   output.addDouble(COLOR_RED[2]); }
        if (objectName == "bluebox")  { output.addDouble(COLOR_BLUE[0]);  output.addDouble(COLOR_BLUE[1]);  output.addDouble(COLOR_BLUE[2]); }
        if (objectName == "greenbox") { output.addDouble(COLOR_GREEN[0]); output.addDouble(COLOR_GREEN[1]); output.addDouble(COLOR_GREEN[2]); }
        if (objectName == "yellowbox"){ output.addDouble(COLOR_YELLOW[0]); output.addDouble(COLOR_YELLOW[1]); output.addDouble(COLOR_YELLOW[2]); }
        if (objectName == "whitebox") { output.addDouble(COLOR_WHITE[0]); output.addDouble(COLOR_WHITE[1]); output.addDouble(COLOR_WHITE[2]); }
    }
    else
    {
        // load 3d model
        output.addString("world");
        output.addString("mk");
        output.addString("model");
        output.addString(objectName.c_str());
        output.addString(textureFile.c_str());
        output.addDouble(handX);
        output.addDouble(handY + BOX_SIZE/2 + 0.02);
        output.addDouble(handZ + 0.02);
        output.addInt(0);
    }
    Bottle ret;
    worldPortSync.write(output, ret);
    std::cout << ret.toString() << std::endl;

    // rotate 3d model
    Bottle rotCmd;
    rotCmd.clear();
    rotCmd.addString("world");
    rotCmd.addString("rot");
    rotCmd.addString("model");
    rotCmd.addInt(1);
    rotCmd.addDouble(0);
    rotCmd.addDouble(rotation);
    rotCmd.addDouble(0);
    worldPortSync.write(rotCmd, ret);

    Bottle& grabCmd = worldPort.prepare();
    grabCmd.clear();
    grabCmd.addString("world");
    grabCmd.addString("grab");
    if (objectName == "redbox" || objectName == "bluebox" || 
            objectName == "greenbox" ||
            objectName == "whitebox" ||
            objectName == "yellowbox"
       )
        grabCmd.addString("box");
    else
        grabCmd.addString("model");
    grabCmd.addInt(1);
    grabCmd.addString("right");
    grabCmd.addInt(1);
    worldPort.write();

    std::cout << "done" << std::endl;

    return true;
}
