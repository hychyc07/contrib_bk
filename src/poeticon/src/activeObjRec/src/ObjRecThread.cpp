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
 * @file ObjRecThread.cpp
 * @brief implementation of the ObjRecThread methods.
 */

#include "iCub/ObjRecThread.h"
#include "iCub/ObjRecModule.h"

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
 
using namespace yarp::os;
using namespace yarp::sig;


bool ObjRecThread::threadInit() 
{
    std::cout << "Initializing thread..." << std::endl;

    std::string moduleName = rf->check("name", Value("activeObjRec"), "module name (string)").asString().c_str();

    std::string defaultDataRoot = "./data";

    std::string defaultObjectViewDir = defaultDataRoot+"/objectViewDir";
    objectViewDir = rf->check("objectViewDir", Value(defaultObjectViewDir.c_str()), "Directory for storing object views").asString().c_str();
    std::cout << "Object view directory: " << objectViewDir << std::endl;
        
    std::string defaultResultDir = defaultDataRoot+"/results";
    recogResultDir = rf->check("recogResultDir", Value(defaultResultDir.c_str())).asString().c_str();
    std::cout << "Result directory: " << recogResultDir << std::endl;

    // read parameters for entropy based recognition
    DESIRED_ENTROPY = rf->check("desiredEntropy", Value(0.05), "stop recognition when desired entropy is reached").asDouble();

    settings.minElevation = rf->check("minElevation", Value(0), "minimal elevation angle on view sphere").asInt();
    settings.maxElevation = rf->check("maxElevation", Value(90), "maximal elevation angle on view sphere").asInt();
    settings.minRotation  = rf->check("minRotation", Value(140), "minimal Rotation angle on view sphere").asInt();
    settings.maxRotation  = rf->check("maxRotation", Value(280), "maximal Rotation angle on view sphere").asInt();

    fastPlanning     = rf->check("fastPlanning", Value(1)).asInt();
    waitOnIter       = rf->check("waitOnIter", Value(0)).asInt();

    Nb_Iteration = rf->check("maxIterations", Value(15), "maximum number of recognition iterations").asInt();

    maxRecogTrials = rf->check("recogTrials", Value(1)).asInt();
    recogTrialNo = 0;

    settings.boostingRate = rf->check("boostingRate", Value(0.4)).asDouble();

    settings.particlesPerObject = rf->check("particlesPerObject", Value(100)).asInt();

    // port to send notification when recognition done
    if (! isSimulation)
    {
        // port for sending recognition results
        std::string recogResultPortName = "/" + moduleName + "/";
        recogResultPortName += rf->check("recogResultPort", Value("recogResult:o"), 
                "sends object probabilities").asString().c_str();
        if (! recogResultPort.open(recogResultPortName.c_str())) 
        {
            std::cout << "unable to open port " << recogResultPortName << std::endl;
            return false;
        }
    }

    CV_Assert(settings.minElevation >= 0);
    CV_Assert(settings.maxElevation <= 180);
    CV_Assert(settings.minRotation  >= 0);
    CV_Assert(settings.maxRotation  <= 360);

    CV_Assert(DESIRED_ENTROPY >= 0);

    CV_Assert(! objectViewDir.empty());

    if (! activeRec.init(objectViewDir))
    {
        std::cerr << "Error: Active recognition could not be initialized!" << std::endl;
        return false;
    }
    if (! restartRecognition())
    {
        std::cerr << "Error: Active recognition could not be restarted!" << std::endl;
        return false;
    }

    //if (! isSimulation)
        //RateThread::suspend();


    std::cout << "Done." << std::endl;

    return true;
}



void ObjRecThread::threadRelease() 
{
    recogResultPort.interrupt();
    recogResultPort.close();
}


bool ObjRecThread::restartRecognition()
{
    activeRec.restart();

    std::cout << "doMotionPlanning: " << doMotionPlanning << std::endl;
    activeRec.setMotionPlanning(doMotionPlanning);
    activeRec.setFastPlanning(fastPlanning);
    activeRec.setObjectRotationOffset(objectRotation);

    fs::path p = fs::path(recogResultDir);
    if (settings.doBoosting)
        p /= "results_boosted";
    else
        p /= "results";

    if (doMotionPlanning)
        p /= "planned";
    else
        p /= "random";

    try
    {
        fs::create_directories(p);
    }
    catch(std::exception &e)
    {
        std::cerr << "ERROR - ActiveRec::setResultDir: Could not create output directory " << p.string() << std::endl; 
        std::cerr << "\t ... " << e.what() << std::endl; 
        return false;
    }

    std::cout << "outDir: " <<  p << std::endl;
    std::cout << "trueLabel: " <<  trueLabel << std::endl;
    activeRec.setWriteData(p.string(), trueLabel);

    double e,r;
    module->getHandGazeAngles(e, r);
    last_gaze[0] = e;
    last_gaze[1] = r;
    recogIteration = 0;

    return true;
}


void ObjRecThread::run() 
{
    runRecognition();
    
    //viewSphere.markGaze(currentElevation, currentRotation);
    //viewSphere.show("View Sphere", 5);
}


void ObjRecThread::runRecognition()
{
    cv::Mat img = module->getCameraImage();
    if (img.empty())
    {
        return;
    }

    if (isRecogDone()) // recognition completed?
    {
        // done
        std::vector<std::string> objectNames = activeRec.objectNames();
        cv::Mat objProbs = activeRec.getObjectProbabilities();

        if (objProbs.cols != 1)
        {
            std::cerr << "Warning: Object probabilies expected to be a Nx1 matrix!" << std::endl;
        }

        // print results
        std::cout << std::endl;
        std::cout << "Num iterations: " << recogIteration << std::endl;
        std::cout << "Done." << std::endl;

        recogTrialNo++;

        if (recogTrialNo < maxRecogTrials)
        {
            std::cout << "Trial " << recogTrialNo << " completed. Starting next trial..." << std::endl;
            // restart recognition from random postion
            double targetE = yarp::os::Random::uniform(settings.minElevation, settings.maxElevation); 
            double targetR = yarp::os::Random::uniform(settings.minRotation, settings.maxRotation); 
            module->handGazeControl->lookAtViewpoint(targetE, targetR);
            Time::delay(3.0);

            if (! restartRecognition())
            {
                std::cerr << "Error: Active recognition could not be restarted!" << std::endl;
                stop();
                return;
            }
        } 
        else
        {
            std::cout << "Trial " << recogTrialNo << " completed. Done." << std::endl;
            recogTrialNo = 0;

            // send results
            if (! isSimulation)
            {
                // sort results
                cv::Mat sortedIds;
                cv::sortIdx(objProbs, sortedIds, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING);

                // send results sorted by probability, best object first
                Bottle& msg = recogResultPort.prepare();
                msg.clear();
                for (int k = 0; k <  objectNames.size(); k++)
                {
                    int id = sortedIds.at<int>(k);
                    msg.addString(objectNames[id].c_str());
                    msg.addDouble(objProbs.at<float>(id));
                }
                recogResultPort.write();
            }

            module->sendActionDoneMsg("recognition completed");
            stop();
        }
    }
    else
    {
        // done rotating object?
        if (! module->handGazeControl->targetReached()) 
            return;

        std::cout << "\n+++ Active recognition iteration " << ++recogIteration << " +++" << std::endl;

        // update filter
        double e,r;
        module->getHandGazeAngles(e, r);
        cv::Vec2f gaze(e, r);
        cv::Vec2f transition = gaze - last_gaze;
        last_gaze = gaze;
        activeRec.update(img, transition, gaze);

        cv::Vec2f target;
        if (doMotionPlanning)
        {
            // motion planning
            std::cout << "\tsearching next gaze location..." << std::endl;
            target = activeRec.searchTargetGaze(gaze);
            std::cout << "\tdone." << std::endl;
        }
        else
        {
            // return random position
            double targetE = yarp::os::Random::uniform(settings.minElevation+15, settings.maxElevation); 
            double targetR = yarp::os::Random::uniform(settings.minRotation, settings.maxRotation); 
            std::cout << "target: " << targetE << " " << targetR << std::endl;
            target = cv::Vec2f(targetE, targetR);
        }

        if (target != cv::Vec2f())
        {
            if (waitOnIter)
            {
                cv::waitKey();
            }

            //std::cout << "Going to new pose..." << std::endl;
            module->handGazeControl->lookAtViewpoint(target[0], target[1]);
        }
    }
}
