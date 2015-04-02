/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#include <iCub/RecogVtmThread.h>
#include <iCub/ObjRecModule.h>
#include <iCub/HandPoseUtil.h>
#include <iCub/iKin/iKinFwd.h>

#include <boost/lexical_cast.hpp>
#include <boost/timer.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;


bool RecogVtmThread::threadInit() 
{
    if (! VtmThread::threadInit())
        return false;

    REC_CANDIDATE_THRESHOLD = rf->check("recCandidateThreshold", Value(15.0), "max distance for new object candidate").asDouble();
    REC_CONFIRM_THRESHOLD = rf->check("recConfirmThreshold", Value(15.0), "max distance for confirmation of current object candidate").asDouble();

    maxIterations = rf->check("maxIterationsVtm", Value(15), "maximum number of recognition iterations").asInt();
    maxTrials = rf->check("maxTrialsVtm", Value(6), "maximum number of recognition iterations").asInt();

    std::string vtmDir = rf->check("vtmDir", Value("./data/vtms"), "directory containing Vtms").asString().c_str();
    vtmList = loadVtms(vtmDir);

    trialNo = 0;

    restart();

    std::cout << "threadInit done" << std::endl;

    return true;
}

void RecogVtmThread::restart()
{
    recogState = REC_EXPLORE;
    recogIteration = 0;
    
    // reset recognition result
    objectDissimilarities.clear();
    objectDissimilarities.resize(vtmList.size());

    std::cout << "go to start pos" << std::endl;
    // go to start pos
    for (int joint = 0; joint < 7; ++joint)
    {
        Util::safePositionMove(joint, trajectory[0][joint], armCtrl, limArm);
    }
    //Time::delay(2);

}

void RecogVtmThread::saveResults()
{
    try
    {
        fs::create_directories(vtmResultDir);
    }
    catch(std::exception &e)
    {
        std::cerr << "ERROR - RecogVtmThread::threadInit: Could not create output directory " << vtmResultDir << std::endl; 
        std::cerr << "\t ... " << e.what() << std::endl; 
        return;
    }

    int trialNo = 1;
    fs::path resultPath = fs::path(vtmResultDir) / (objectName + "-" + boost::lexical_cast<std::string>(trialNo) + ".txt");
    while (fs::exists(resultPath))
    {
        trialNo++;
        resultPath = fs::path(vtmResultDir) / (objectName + "-" + boost::lexical_cast<std::string>(trialNo) + ".txt");
    }
    std::cout << "Saving results to " << resultPath << std::endl; 
    std::ofstream f(resultPath.c_str(), std::ios::out);
    if (!f.is_open())
    {
        std::cerr << "Could not open file " << resultPath << std::endl;
        return;
    }
    for (int i = 0; i < objectDissimilarities.size(); i++) 
    {
        for (int j = 0; j < objectDissimilarities[i].size(); j++) 
        {
            f << objectDissimilarities[i][j] << " ";
        }
        f << std::endl;
    }
    f.close();
}

void RecogVtmThread::run()
{
    if (recogIteration >= maxIterations) 
    {
        saveResults();
        std::cout << "Done." << std::endl;

        trialNo++;
        if (trialNo >= maxTrials)
        {
            module->sendActionDoneMsg("recog vtm done");
            stop();
        }
        else
        {
            restart();
        }
    }

    // get current view
    cv::Mat camImg = module->getCameraImage();
    if (camImg.empty())
        return;

    imgCurrentView = camImg;
    showImages();


    switch (recogState)
    {
        case REC_EXPLORE:
            {
                moveArm();

                if (! keyframeExtractor.isKeyframe(camImg))
                {
                    return;
                }

                if (findCandidateKeyframe(camImg))
                {
                    imgTargetView = vtmList[targetVtmNo].getKeyframes().at(targetViewNo);

                    std::cout << "NEW CANDIDATE: " << vtmList[targetVtmNo].objectName() << std::endl;
                    std::cout << "TARGET FOUND: KF = " << targetViewNo << std::endl;
                    Util::safePositionMove(targetState, armCtrl, limArm);
                    recogState = REC_CHECKCANDIDATE;
                    std::cout << "GOTO TO POS" << std::endl;
                }
            }
        case REC_CHECKCANDIDATE:
            {
                // wait till target position is reached
                if (! checkArmMotionDone())
                    return;

                recogIteration++;

                // compare obtained view with expected view
                cv::Mat ftActual = Vtm::extractFeature(camImg);
                cv::Mat ftExpected = vtmList[targetVtmNo].getFeatures()[targetViewNo];
                double dist = Vtm::compareKeyframes(ftActual, ftExpected);

                std::cout << "POS REACHED: DIST = " << dist << std::endl;
                if (dist < REC_CONFIRM_THRESHOLD)
                {
                    std::cout << "***SUCCEEDED***" << std::endl;

                    // update results
                    objectDissimilarities[targetVtmNo].push_back(dist);

                    printDissimilarities();


                    // select new kf to ckeck
                    if (findCandidateKeyframe(camImg, targetVtmNo))
                    {
                        imgTargetView = vtmList[targetVtmNo].getKeyframes().at(targetViewNo);

                        std::cout << "NEW TARGET KF: " << targetViewNo << std::endl;
                        Util::safePositionMove(targetState, armCtrl, limArm);
                        recogState = REC_CHECKCANDIDATE;
                        std::cout << "GOTO TO POS" << std::endl;
                    }
                    else
                    {
                        std::cout << "***FAILED***" << std::endl;
                        imgTargetView = cv::Mat::zeros(imgTargetView.rows, imgTargetView.cols, CV_8UC3);

                        std::cout << "NO KF FOUND -> EXPLORE" << std::endl;
                        recogState = REC_EXPLORE;
                    }
                }
                else
                {
                    std::cout << "FAILED -> EXPLORE" << std::endl;
                    recogState = REC_EXPLORE;
                }

            }
            break;
    }
}


void RecogVtmThread::printDissimilarities() const
{
    std::cout << std::endl << "====================================" << std::endl;
    std::cout << "Object dissimilarities:" << std::endl;
    for (int obj = 0; obj < objectDissimilarities.size(); obj++)
    {
        std::cout << vtmList[obj].objectName() << ": \t";
        for (int res = 0; res < objectDissimilarities[obj].size(); res++)
        {
            std::cout << objectDissimilarities[obj][res] << " "; 
        }
        std::cout << std::endl;
    }
    std::cout << "====================================" << std::endl << std::endl;
}


bool RecogVtmThread::findCandidateKeyframe(const cv::Mat &currentKeyframe, int candVtmNo)
{
    int matchingViewNo;
    double minDist = 1000;
    cv::Mat featCurrentKf = Vtm::extractFeature(currentKeyframe);

    // find matching keyframes in VTM
    int vtmStartNo = 0;
    int vtmEndNo = vtmList.size();
    if (candVtmNo >= 0)
    {
        vtmStartNo = candVtmNo;
        vtmEndNo = candVtmNo + 1;
    }
    for (int vtmNo = vtmStartNo; vtmNo < vtmEndNo; ++vtmNo)
    {
        int kfNo = 0;
        std::vector<cv::Mat>::const_iterator featIter;
        for (featIter = vtmList[vtmNo].getFeatures().begin(); 
                featIter != vtmList[vtmNo].getFeatures().end(); ++featIter, ++kfNo)
        {
            double dist = Vtm::compareKeyframes(featCurrentKf, *featIter);
            if (dist < minDist)
            {
                minDist = dist;
                matchingViewNo = kfNo;
                targetVtmNo = vtmNo;
            }
        }
    }

    //std::cout << "MATCHING KF: NO = " << matchingViewNo << ", DIST = " << minDist << std::endl;
    std::cout << "BEST CANDIDATE : " << vtmList[targetVtmNo].objectName() 
              << ", KF = " << matchingViewNo << ", DIST = " << minDist << std::endl;

    boost::timer t;
    // check if the best match is good enough
    if (minDist > REC_CANDIDATE_THRESHOLD)
    {
        std::cout << "No candidate kf found." << std::endl;
        return false;
    }


    imgMatchedView = vtmList[targetVtmNo].getKeyframes().at(matchingViewNo);
    Vector eyePos = module->getEyePosition();
    // get current position in joint space
    Vector currentState(16);
    if (! encArm->getEncoders(currentState.data()))
    {
        fprintf(stderr, "Could not get arm data\n"); 
        return false;
    }
    Vector minLim(NUM_ARM_JOINTS), maxLim(NUM_ARM_JOINTS);
    for (int jointNo = 0; jointNo < NUM_ARM_JOINTS; ++jointNo) 
    {
        double min, max;
        limArm->getLimits(jointNo, &min, &max);
        minLim[jointNo] = min; 
        maxLim[jointNo] = max;
    }

    // select transition
    std::vector<std::pair<Vector, double> > possibleTargets;
    const TransitionMap transitions = vtmList[targetVtmNo].getTransitions();
    TransitionMap::const_iterator tIter;
    int cnt = 0;
    for (tIter = transitions.begin(); tIter != transitions.end(); ++tIter)
    {
        if (tIter->first.first == matchingViewNo)
        {
            //std::cout << "checking target kf no: " << tIter->first.second << std::endl;

            // dont "go" to current view no
            if (tIter->first.second == matchingViewNo)
                continue;

            Vector t = tIter->second;

            // compute position after executing transition
            targetState = currentState;
            for (int jointNo = 0; jointNo < NUM_ARM_JOINTS; ++jointNo) 
            {
                targetState[jointNo] = currentState[jointNo] + t[jointNo];
            }

            //std::cout << "x: " << currentState.toString() << std::endl;
            //std::cout << "t: " << t.toString() << std::endl;
            //std::cout << "xd: " << targetState.toString() << std::endl;

            // check if transition can be executed
            bool reachablePosition = true;
            for (int jointNo = 0; jointNo < NUM_ARM_JOINTS; ++jointNo)
            {
                if (targetState[jointNo] < minLim[jointNo] || targetState[jointNo] > maxLim[jointNo])
                {
                    reachablePosition = false;
                    //std::cout << "not reachable" << std::endl;
                }
            }


            // 
            // check if object can be seen (not hidden behind hand)
            //
            
            // get end effector pose at desired configuration
            iCub::iKin::iCubArm iKinArm("right");
            iKinArm.releaseLink(0);
            iKinArm.releaseLink(1);
            iKinArm.releaseLink(2);
            Vector torso = module->getTorsoQ();
            Vector ang(NUM_ARM_JOINTS+3); // 3 joints for torso
            ang[0] = torso[0];
            ang[1] = torso[1];
            ang[2] = torso[2];
            for (int i = 0; i < NUM_ARM_JOINTS; i++)
                ang[i+3] = targetState[i];
            iKinArm.setAng(ang);
            Vector endEffPose = iKinArm.EndEffPose(ang, true);
            Vector x(3);
            Vector o(4);
            for (int i = 0; i < 3; i++)
                x[i] = endEffPose[i];
            for (int i = 0; i < 4; i++)
                o[i] = endEffPose[i+3];

            // calculate viewpoint
            double e, r;
            HandPoseUtil::getGazeAngles(eyePos, x, o, e, r, false);

            // check if viewpoint makes sense
            //std::cout << "e/r:   " << e << "\t" << r << std::endl;
            if (e > 100.0 || r > 300 || r < 120)
                reachablePosition = false;

            if (! reachablePosition)
                continue;

            // good transition found
            //std::cout << "good transition found" << std::endl;
            
            //std::cout << "transition: " << t.toString() << std::endl;
            // compute transition length
            double transitionLength = 0;
            for (int jointNo = 0; jointNo < NUM_ARM_JOINTS; ++jointNo)
            {
                transitionLength += t[jointNo] * t[jointNo];
            }
            transitionLength = sqrt(transitionLength);

            //std::cout << "transition length: " << transitionLength << std::endl;

            // store as possible target
            possibleTargets.push_back(std::make_pair(targetState,  tIter->first.second));

        }
    }

    std::cout << possibleTargets.size() << " possible targets found. " << t.elapsed() << std::endl;

    if (possibleTargets.empty())
        return false; // we havn't seen anything like that before

    // select random target position
    std::cout << "Selecting random target..." << std::endl;
    int idx = rand() % possibleTargets.size();
    //std::cout << "selected no " << idx << std::endl;
    targetState = possibleTargets[idx].first;
    targetViewNo = possibleTargets[idx].second;

    return true; // found candidate to verife
}


std::vector<Vtm> RecogVtmThread::loadVtms(const std::string &dataDir)
{
    std::vector<Vtm> vtms;

    if (! fs::is_directory(dataDir))
    {
        std::cerr << dataDir << " does not exist or is not a valid directory!" << std::endl;
        return vtms;
    }

    std::cout << "Loading object data from " << dataDir << std::endl;

    // iterate through dataDir
    for (fs::directory_iterator itr(dataDir); itr!=fs::directory_iterator(); ++itr)
    {
        if (! fs::is_directory(itr->path()))
        {
            continue; // skip files
        }

        std::string objectName = itr->path().filename().string();
        std::cout << "Found object '" << objectName << "'" << std::endl;

        Vtm vtm;
        vtm.load(dataDir, objectName);
        vtms.push_back(vtm);
    }

}


void RecogVtmThread::showImages() const
{
    if (! imgCurrentView.empty())
        cv::imshow("Current view", imgCurrentView);
    if (! imgMatchedView.empty())
        cv::imshow("Matched view", imgMatchedView);
    if (! imgTargetView.empty())
        cv::imshow("Target view", imgTargetView);
    cv::waitKey(10);
}


void RecogVtmThread::runVtmComparisions()
{
    std::string trainingDir = "./data_sim/vtms";
    std::string testDir     = "./data_sim/vtms_test";

    // load training VTMs
    std::cout << "Loading training set" << std::endl;
    std::vector<Vtm> trainingSet =  loadVtms(trainingDir);
    
    // load test VTMs
    std::cout << "Loading test set" << std::endl;
    std::vector<Vtm> testSet = loadVtms(testDir);

    cv::Mat results(trainingSet.size(), testSet.size(), CV_32F);
    for (int i = 0; i < trainingSet.size(); i++) 
    {
        for (int j = 0; j < testSet.size(); j++)
        {
            std::cout << "cmp " << i << " " << j << std::endl;
            results.at<float>(i,j) = compareVtms(trainingSet[i], testSet[j]);
        }
    }

    // save results
    std::string resultFilename = "vtmresults.txt";
    std::cout << "Saving results to " << resultFilename << std::endl; 
    std::ofstream f(resultFilename.c_str(), std::ios::out);
    if (!f.is_open())
    {
        std::cerr << "Could not open file " << resultFilename << std::endl;
        return;
    }
    for (int i = 0; i < results.rows; i++) 
    {
        for (int j = 0; j < results.cols; j++) 
        {
            f << results.at<float>(i,j) << " ";
        }
        f << std::endl;
    }

    f.close();
}

// compare complete VTMs (offline)
double RecogVtmThread::compareVtms(const Vtm &vtm1, const Vtm &vtm2) const
{
    std::vector<std::pair<int,int> > map;
    double MIN_MATCHSCORE = 30;
    double MAX_TRANSITION_DIFFERENCE = 20;
    double MIN_TRANSITION_LENGTH = 40;
    double bestMatch = std::numeric_limits<double>::infinity();

    std::cout << "finding links..." << std::endl;
    for (int i = 0; i < vtm1.getFeatures().size(); i++)
    {
        for (int j = 0; j < vtm2.getFeatures().size(); j++)
        {
            double featureDist = Vtm::compareKeyframes(vtm1.getFeatures()[i], vtm2.getFeatures()[j]);
            std::cout << "featureDist: " << featureDist << std::endl;
            if (featureDist < MIN_MATCHSCORE && featureDist > 5)
            {
                map.push_back(std::make_pair(i,j));
                std::cout << "link "<< map.size() << ":\t" << i << " " << j << std::endl;
            }
        }
    }

    std::cout << "matching links..." << std::endl;
    for (int i = 0; i < map.size(); i++)
    {
        //const TransitonMap &transitions = vtm1.getTransitions();
        for (int j = 0; j < map.size(); j++)
        {
            int obj1kf1 = map[i].first;
            int obj1kf2 = map[j].first;
            int obj2kf1 = map[i].second;
            int obj2kf2 = map[j].second;

            if ((obj1kf1 == obj1kf2) || (obj2kf1 == obj2kf2))
                continue;

            double transitionLength = vtm1.calcTransitionLength(obj1kf1, obj1kf2);
            //std::cout << "transitionLength " << obj1kf1 << " " << obj1kf2 << ": " << transitionLength << std::endl;
            if (transitionLength >= MIN_TRANSITION_LENGTH)
            {
                double transitionDifference = Vtm::calcTransitionDifference(
                         vtm1.getTransition(obj1kf1,obj1kf2), 
                         vtm2.getTransition(obj2kf1,obj2kf2));

                //std::cout << "transitionDifference: " << transitionDifference << std::endl;
                if (transitionDifference < MAX_TRANSITION_DIFFERENCE)
                {
                    double matchKf1 = Vtm::compareKeyframes(
                            vtm1.getFeatures()[obj1kf1],
                            vtm2.getFeatures()[obj2kf1]);
                    double matchKf2 = Vtm::compareKeyframes(
                            vtm1.getFeatures()[obj1kf2],
                            vtm2.getFeatures()[obj2kf2]);
                    double match = matchKf1 * matchKf2;

                    if (match < bestMatch)
                    {
                        bestMatch = match;
                    }
                }
            }
        }
    }

    return bestMatch;
}
