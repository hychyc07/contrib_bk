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
 * @file HandGazeControl.cpp
 * @brief
 */

#include <exception>
#include <cmath>

#include <opencv2/core/core_c.h>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>
using namespace yarp::math;

#include "iCub/HandGazeControl.h"
#include "iCub/HandPoseUtil.h"
#include "iCub/Util.h"


#define CARTESIAN_CONTROL 1

const double HandGazeControl::NORM_GAZE   = 180;
const double HandGazeControl::NORM_TARGET_X = 1;
const double HandGazeControl::NORM_DELTA_Q  = 100;

//HandGazeControl::~HandGazeControl()
//{
    //delete nnSearch;
//}


bool HandGazeControl::init()
{
    std::cout << "Initializing HandGazeControl..." << std::endl;

    samplesE.clear();
    samplesR.clear();
    samplesHandX.clear();
    samplesEyeX.clear();
    samplesJoints.clear();

    // read recorded data
    std::ifstream file(filenameTrainingData.c_str(), std::ios::in);
    if (file.is_open())
    {
        int cnt = 0;

        while(! file.eof())
        {
            double e, r;
            Vector handX(3);
            Vector handO(4);
            Vector eyeX(3);
            Vector joints(10);

            file >> e >> r;
            file >> handX[0] >> handX[1] >> handX[2];
            file >> eyeX[0] >> eyeX[1] >> eyeX[2];
            for (int i = 0; i < 10; i++)
            {
                file >> joints[i];
            }
            file >> handO[0] >> handO[1] >> handO[2] >> handO[3];

            cnt++;

            if (! file.eof())
            {
                samplesE.push_back(e);
                samplesR.push_back(r);
                samplesHandX.push_back(handX);
                samplesEyeX.push_back(eyeX);
                samplesJoints.push_back(joints);
                samplesHandO.push_back(handO);
            }
        }

        file.close();
    }

    std::cout << samplesE.size() << " samples loaded" << std::endl;

    if (samplesE.empty())
    {
        std::cerr << "!!! No data found for HandGazeControl!" << std::endl;
        std::cerr << "!!! Please run 'kinexp' to collect training data." << std::endl;
        return true;
    }

#if SAMPLE_CLUSTERING
    // perform kmeans clustering
    std::cout << "Perfoming kmeans clustering..." << std::endl;
    int sampleDim = 5;
    int sampleCount = samplesE.size();
    cv::Mat samples(sampleCount, sampleDim, CV_32F);
    for (int i = 0; i < sampleCount; i++)
    {
        double gx, gy;
        Util::gazeAnglesTo2D(samplesE[i], samplesR[i], gx, gy);
        samples.at<float>(i, 0) = gx;
        samples.at<float>(i, 1) = gy;
        samples.at<float>(i, 2) = samplesHandX[i][0];
        samples.at<float>(i, 3) = samplesHandX[i][1];
        samples.at<float>(i, 4) = samplesHandX[i][2];
    }
    int clusterCount = 30000;
    cv::TermCriteria termcrit(cv::TermCriteria::MAX_ITER, 5, -1);
    cv::Mat labels(sampleCount, 1, CV_32S);
    cv::Mat centers(clusterCount, sampleDim, CV_32F);
    double compactness = cv::kmeans(samples, clusterCount, labels, termcrit, 2, cv::KMEANS_PP_CENTERS, &centers);
    std::cout << "Done. Compactness = " << compactness << std::endl;
#endif

#if NNSEARCH
    // create index for approximate nearest neighbor search 
    double gx,gy;
    std::cout << "Creating nearest neighbor index..." << std::endl;
    cv::Mat gazeAngles(samplesE.size(), 5, CV_32F, cv::Scalar(0));
    for (int i = 0; i < gazeAngles.rows; i++) 
    {
        Util::gazeAnglesTo2D(samplesE[i], samplesR[i], gx, gy);
        gazeAngles.at<float>(i,0) = gx;
        gazeAngles.at<float>(i,1) = gy;
        gazeAngles.at<float>(i,2) = samplesHandX[i][0];
        gazeAngles.at<float>(i,3) = samplesHandX[i][1];
        gazeAngles.at<float>(i,4) = samplesHandX[i][2];
    }

    //std::cout << gazeAngles.at<float>(gazeAngles.rows-1, 0) << " "
    //<< gazeAngles.at<float>(gazeAngles.rows-1, 1) << " " 
    //<< gazeAngles.at<float>(gazeAngles.rows-1, 2) << " " 
    //<< gazeAngles.at<float>(gazeAngles.rows-1, 3) << " " 
    //<< gazeAngles.at<float>(gazeAngles.rows-1, 4) << std::endl;
    //nnSearch = new cv::flann::Index(gazeAngles, cv::flann::AutotunedIndexParams());
    //nnSearch = new cv::flann::Index(gazeAngles, cv::flann::KDTreeIndexParams(16));
    nnSearch = new cv::flann::Index(centers, cv::flann::KMeansIndexParams(32, 11, cvflann::CENTERS_RANDOM));
    //nnSearch = new cv::flann::Index(gazeAngles, cv::flann::LinearIndexParams());
    if (! nnSearch)
    {
        std::cerr << "Could not create index for approximate nearest neighbors search!" << std::endl;
        return false;
    }

    std::cout << "veclen " << nnSearch->veclen() << std::endl;
    std::cout << "size " << nnSearch->size() << std::endl;
    std::cout << "Done." << std::endl;
#endif
}

Vector HandGazeControl::getHandPose(const Vector &torsoQ, const Vector &armQ)
{
    iCub::iKin::iCubArm libArm("right");         
    iCub::iKin::iKinChain *chain = libArm.asChain();     

    chain->releaseLink(0);
    chain->releaseLink(1);
    chain->releaseLink(2);

    Vector ang(10);
    for (int i = 0; i < torsoQ.size(); i++)
        ang[i] = torsoQ[i]*M_PI/180;
    for (int i = 0; i < 7; i++)
        ang[i+torsoQ.size()] = armQ[i]*M_PI/180;

    chain->setAng(ang);
    Vector armPose = chain->EndEffPose();
    return armPose;
}

bool HandGazeControl::calcTargetPose(double e, double r, Vector &handX, Vector &handO)
{
    double startTime;
    int dim = samplesJoints.front().size();

    // number of closest nearest neighbors that is used to calculate the target position
    //int kNNToCalcTarget = kNN;//std::max(1, static_cast<int>(kNN*0.1));

    int nArmJoints;
    encArm->getAxes(&nArmJoints);

    Vector torsoQ(3), armQ(nArmJoints);

    encArm->getEncoders(armQ.data());
    encTorso->getEncoders(torsoQ.data());

    Vector curJoints(10);
    for (int jnt = 0; jnt < 3; jnt++) curJoints[jnt] = torsoQ[jnt];
    for (int jnt = 3; jnt < dim; jnt++) curJoints[jnt] = armQ[jnt-3];

#if NNSEARCH
    /*
     * find a big set of nearest neighbors only regarding gaze angles 
     */

    if (! nnSearch)
    {
        std::cerr << "nnSearch is null" << std::endl;
        return false;
    }

    double gx,gy;
    Util::gazeAnglesTo2D(e, r, gx, gy);
    cv::Mat query(1, 5, CV_32F);
    query.at<float>(0,0) = gx;
    query.at<float>(0,1) = gy;
    query.at<float>(0,2) = targetX[0];
    query.at<float>(0,3) = targetX[1];
    query.at<float>(0,4) = targetX[2];

    startTime = yarp::os::Time::now();
    int nnCandidateCount = 1000;
    cv::Mat indexPtrs(1, nnCandidateCount, CV_32S);
    cv::Mat distPtrs(1, nnCandidateCount, CV_32F);
    //std::cout << "running nn search " << nnCandidateCount << std::endl;
    nnSearch->knnSearch(query, indexPtrs, distPtrs, nnCandidateCount, cv::flann::SearchParams(32));
    //std::cout << "SEARCH TIME: " << Time::now() - startTime << "s" << std::endl;
    
    //int maxNumNeighbors = 10000;
    //double searchRadius = 1;
    //cv::Mat indexPtrs(1, maxNumNeighbors, CV_32S, cvScalar(0));
    //cv::Mat distPtrs(1, maxNumNeighbors, CV_32F, cvScalar(0));
    //int nnFound = nnSearch->radiusSearch(query, indexPtrs, distPtrs, searchRadius,    cv::flann::SearchParams(maxNumNeighbors*2));
    //std::cout << "nnFound: " << nnFound << std::endl;

    int *neighborId = indexPtrs.ptr<int>(0);

#endif

    /*
     * Calculate distance to nearest neighbors also 
     * taking into account the cartesian position.
     */

    startTime = yarp::os::Time::now();
#if NNSEARCH
    cv::Mat distances(samplesE.size(), 1, CV_32F, cv::Scalar(1000));
    cv::Mat nearestNeighborIds(samplesE.size(), 1, CV_32S, cv::Scalar(0));
    for (int i = 0; i < kNN; i++)
    {
        int nb = neighborId[i];
        distances.at<float>(nb,0) = compDistance(e, r, curJoints, nb);
        nearestNeighborIds.at<int>(nb,0) = nb;
    }
    cv::sortIdx(distances, nearestNeighborIds, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
    cv::sort(distances, distances, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
#else
    // old
    cv::Mat distances(samplesE.size(), 1, CV_32F, cv::Scalar(0));
    cv::Mat nearestNeighborIds(samplesE.size(), 1, CV_32S, cv::Scalar(0));
    for (int i = 0; i < samplesE.size(); i++)
    {
        distances.at<float>(i,0) = compDistance(e, r, curJoints, i);
        nearestNeighborIds.at<int>(i,0) = i;
    }
    cv::sortIdx(distances, nearestNeighborIds, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
    cv::sort(distances, distances, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
#endif
    
    //std::cout << "CALC DIST TIME: " << Time::now() - startTime << "s" << std::endl;

     /*
      * Calculate weighted average of best n neighbors
      */

    //startTime = yarp::os::Time::now();
    //double totalWeights = 0;
    //Vector avgJoints(dim);
    //avgJoints.zero();
    //for (int i = 0; i < kNN; i++)
    //{
        //int nbId = neighborId[i];
        //double dist = compDistance(e, r, curJoints, nbId);
        //double w = exp(-16*dist);
        ////std::cout << "w: " << w  << "\tdist: " << dist << std::endl;

        ////int nbId = nearestNeighborIds.at<int>(i,0);
        //for (int jnt = 0; jnt < dim; jnt++)
        //{
            //avgJoints[jnt] +=  w * samplesJoints[nbId][jnt];
        //}
        //totalWeights += w; 
    //}
    //std::cout << "WEIGH AVG TIME: " << Time::now() - startTime << "s" << std::endl;
    
    startTime = yarp::os::Time::now();
    double totalWeights = 0;
    Vector avgJoints(dim);
    avgJoints.zero();
    for (int i = 0; i < kNN; i++)
    {
        double w = exp(-16*distances.at<float>(i,0));
        int nbId = nearestNeighborIds.at<int>(i,0);
        for (int jnt = 0; jnt < dim; jnt++)
        {
            avgJoints[jnt] +=  w * samplesJoints[nbId][jnt];
        }
        totalWeights += w; 
    }
    //std::cout << "WEIGH AVG TIME: " << yarp::os::Time::now() - startTime << "s" << std::endl;

    if (totalWeights == 0)
        return false;

    for (int jnt = 0; jnt < dim; jnt++)
    {
        avgJoints[jnt] /=  totalWeights;
    }

    // update arm and torso position
    for (int jnt = 0; jnt < 10; jnt++)
    {
        if (jnt < 3)
            torsoQ[jnt] = avgJoints[jnt];
        else
            armQ[jnt-3] = avgJoints[jnt]; 
    }

    armQ[1] = std::max(armQ[1], 10.0);
 
    // get best pose
    handX = Vector(3);
    handO = Vector(4);
    int id = nearestNeighborIds.at<int>(0,0);
    handX = samplesHandX[id];
    handO = samplesHandO[id];
    Vector eyeX  = samplesEyeX[id];

    //std::cout << "target: " << handX.toString() << " " << handO.toString() << std::endl;
    //std::cout << "teye:   " << eyeX.toString() << std::endl;

    //Vector handPose = getHandPose(torsoQ, armQ);
    //assert(handPose.size() == 7);
    //for (int i=0; i < handX.size(); i++)
        //handX[i] = handPose[i];
    //for (int i=0; i < handO.size(); i++)
        //handO[i] = handPose[i+handX.size()];


    //armCart->getPose(handX, handO);
    //std::cout << "actual: " << handX.toString() << " " << handO.toString() << std::endl;
    
    return true;
}


bool HandGazeControl::targetReached()
{
    //if (isNewTarget) 
        //return false; 
    bool done;
    armCart->checkMotionDone(&done);
    return done;
}

double HandGazeControl::compDistance(double e, double r, const Vector &curJoints, int index)
{
    double gazeDist = Util::calcCentralAngle(e, samplesE[index], r, samplesR[index]); 
    double targetXDist = yarp::math::norm(samplesHandX[index] - targetX);
    double deltaQ = yarp::math::norm(samplesJoints[index] - curJoints);

    return (wGaze * (gazeDist/NORM_GAZE) 
            + wTargetX * (targetXDist/NORM_TARGET_X) 
            + wDeltaQ * (deltaQ/NORM_DELTA_Q)) 
        / (wGaze + wTargetX + wDeltaQ);
}


void HandGazeControl::fixateObject()
{
    if (!armCart || !gazeCtrl) return; 

    // get cartesian hand pose
    Vector handCoord, handOrient;
    armCart->getPose(handCoord, handOrient);
    Vector objectPos = HandPoseUtil::addOffset(handCoord, handOrient, 
            gazeOffsetX, gazeOffsetY, gazeOffsetZ, isLeftHand);

    // look at palm
    if (! gazeCtrl->lookAtFixationPoint(objectPos))
    {
        std::cerr << "Could not look at fixation point!" << std::endl;
    }
}
