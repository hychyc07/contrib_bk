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
 * @file ActiveRec.cpp
 * @brief implementation of the ActiveRec methods.
 */

#include <iomanip>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/timer.hpp>

#include "iCub/ActiveRec.h"
#include "iCub/Util.h"



namespace fs = boost::filesystem;
 
bool ActiveRec::init(const std::string &objectViewDir)
{
    double startTime;

    std::cout << "\nInitializing active recognition..." << std::endl;

    recogIter = 0;

    writeData = false;

    if (! loadObjects(objectViewDir))
    {
        std::cerr << "ERROR - ActiveRec::init: Could not load object data!" << std::endl; 
        return false;
    }

#if TEST_CUDA_SUPPORT
    cv::Mat mat = keyviews[0];
    double s = 0;
    for (int i = 0; i < keyviews[0].rows; i++)
    {
        s += cv::norm(mat.row(0), mat.row(i), cv::NORM_L1); 
    }
    std::cout << s << std::endl;

    std::cout << "Cuda Enabled: " << cv::gpu::getCudaEnabledDeviceCount() << std::endl;

    s = 0;
    cv::gpu::GpuMat gMat(keyviews[0]);
    for (int i = 0; i < keyviews[0].rows; i++)
    {
        s += cv::gpu::norm(gMat.row(0), gMat.row(i), cv::NORM_L1); 
    }
    std::cout << s << std::endl;
#endif

    pfObjectModel.init(objects, settings.minElevation, settings.maxElevation, 
            settings.minRotation, settings.maxRotation);

    //restart();

    std::cout << "Initialization done." << std::endl;

    return true;
}

void ActiveRec::restart()
{
    recogIter = 0;
    recogTimer.restart();

    objProbsKNN1 = cv::Mat::ones(objects.size(), 1, CV_32F);
    objProbsKNN3 = cv::Mat::ones(objects.size(), 1, CV_32F);
    objProbsKNN5 = cv::Mat::ones(objects.size(), 1, CV_32F);

    particleFilter.init(pfObjectModel, settings.particlesPerObject);
}


void ActiveRec::setWriteData(const std::string &dir, const std::string &objectName)
{
    writeData = false;

    if (dir.empty())
    {
        std::cerr << "ERROR - ActiveRec::setWriteData: No output directory specified!" << std::endl; 
        return;
    }

    trueLabel = objectName; 
    if (trueLabel.empty())
    {
        std::cerr << "ERROR - ActiveRec::setWriteData: No object name specified!" << std::endl; 
        return;
    }

    resultDir = dir + "/" + trueLabel + "/";

    try
    {
        fs::create_directory(resultDir);
    }
    catch(std::exception &e)
    {
        std::cerr << "ERROR - ActiveRec::update: Could not create output directory " << resultDir << std::endl; 
        std::cerr << "\t ... " << e.what() << std::endl; 
        return;
    }


    // add dir for new trial
    int maxTrialNo = 0;
    for (fs::directory_iterator itr(resultDir); itr!=fs::directory_iterator(); ++itr)
    {
        std::string trialDir = itr->path().filename().string();

        int trialNo = boost::lexical_cast<int>(trialDir);
        maxTrialNo = std::max(trialNo, maxTrialNo);
    }
    std::string strTrialNo = boost::str(boost::format("%02i") % (maxTrialNo+1));
    
    resultDir += strTrialNo + "/";

    try
    {
        fs::create_directory(resultDir);
    }
    catch(std::exception &e)
    {
        std::cerr << "ERROR - ActiveRec::update: Could not create output directory " << resultDir << std::endl; 
        std::cerr << "\t ... " << e.what() << std::endl; 
        return;
    }

    writeData = true;
}


cv::Mat ActiveRec::knnMatching(const cv::Mat &g, int knn)
{
    if (knn <= 0) 
        knn = 1;

    cv::Mat objProbs(objects.size(), 1, CV_32F, cv::Scalar(0));
    for (size_t k = 0; k < objects.size(); k++)
    {
        objProbs.at<float>(k) = objects[k].matchKNN(g, knn);
    }

    return objProbs;
}



void ActiveRec::update(const cv::Mat &img, const cv::Vec2f &transition, const cv::Vec2f &gaze)
{
    boost::timer timer;
    double startTime;

    recogIter++;

    cv::Mat feature = Object::extractFeature(img);

    cv::Mat boostingFactors;
    if (settings.doBoosting)
    {
        std::cout << "boosting"<<std::endl;
        boostingFactors = calcBoostingFactors(feature);
    }

    std::cout << boostingFactors << std::endl;
    // update object probs
    cv::Mat newObjProbsKNN1 = knnMatching(feature, 1);
    cv::Mat newObjProbsKNN3 = knnMatching(feature, 3);
    cv::Mat newObjProbsKNN5 = knnMatching(feature, 5);

    newObjProbsKNN1 /= cv::sum(newObjProbsKNN1)[0];
    newObjProbsKNN3 /= cv::sum(newObjProbsKNN3)[0];
    newObjProbsKNN5 /= cv::sum(newObjProbsKNN5)[0];
    if (settings.doBoosting)
    {
        newObjProbsKNN1 = newObjProbsKNN1.mul(boostingFactors); 
        cv::Mat boostedNewObjProbsKNN3 = newObjProbsKNN3.mul(boostingFactors); 
        newObjProbsKNN5 = newObjProbsKNN5.mul(boostingFactors); 

        newObjProbsKNN3 = settings.boostingRate*boostedNewObjProbsKNN3 
            + (1-settings.boostingRate)*newObjProbsKNN3;
    }

    objProbsKNN1 = objProbsKNN1.mul(newObjProbsKNN1);
    objProbsKNN3 = objProbsKNN3.mul(newObjProbsKNN3);
    objProbsKNN5 = objProbsKNN5.mul(newObjProbsKNN5);
    // normalize
    objProbsKNN1 /= cv::sum(objProbsKNN1)[0];
    objProbsKNN3 /= cv::sum(objProbsKNN3)[0];
    objProbsKNN5 /= cv::sum(objProbsKNN5)[0];

    // visualize view similarities
    boost::timer t;
    viewSimImgs = createViewSimImages(feature, gaze);
    std::cout << "Num imgs: " << viewSimImgs.size() << std::endl;
    cv::Mat sims = Util::tileImgs(viewSimImgs, 5, 5, objectNames());
    cv::imshow("view sims", sims);
    std::cout << "view sim time: " << t.elapsed() << std::endl;

    // recognition using particle filter 
    particleFilter.filter(feature, transition, boostingFactors);


    // print current results
    cv::Mat objProbs = particleFilter.getObjectProbabilities();
    for (size_t i = 0; i < objects.size(); i++)
    {
        std::cout << std::setw(20) << std::left << objects[i].getName() << ":" 
            << std::fixed << std::setprecision(3)
            << "\t" << objProbs.at<float>(i) 
            << "\t" << objProbsKNN3.at<float>(i) 
            << " (" << newObjProbsKNN3.at<float>(i) << ")"
            << std::fixed << std::setprecision(3)
            << "\t" << (!boostingFactors.empty() ? boostingFactors.at<float>(i) : 0.0)
            << std::endl;
    }

    cv::Mat knnlog;
    cv::log(objProbsKNN3, knnlog);
    double H_KNN3 = -cv::sum(objProbsKNN3.mul(knnlog))[0];
    std::cout << std::setfill ('-') << std::setw (55) << "-" << std::setfill(' ') << std::endl;
    std::cout << std::setw(20) << std::left << "H" << ":" 
            << std::fixed << std::setprecision(2)
            << "\t" << getEntropy() 
            << "\t" << H_KNN3 
            << std::endl;

    // visualize particles
    particleImgs = particleFilter.createParticleImages(false, gaze);
    cv::Mat particleVis = Util::tileImgs(particleImgs, 5, 5, objectNames());
    cv::Mat particleVis_small; 
    cv::resize(particleVis, particleVis_small, cv::Size(), 0.5, 0.5, cv::INTER_CUBIC);
    cv::imshow("particles", particleVis_small);


    //showBestKeyframes();


    cv::Vec2f target;
    // 
    // write data to disk for evaluations
    //
    if (writeData)
    {
        writeIterationData(objProbs, objProbsKNN1, objProbsKNN3, objProbsKNN5, 
                gaze, target, entropyImg, img);
    }

    //std::cout << "\tUpdate time: " << timer.elapsed() << std::endl;
    //cv::waitKey();

}


void ActiveRec::writeIterationData(const cv::Mat &objProbs, 
        const cv::Mat &objProbsKNN1, const cv::Mat &objProbsKNN3, const cv::Mat &objProbsKNN5, 
        const cv::Vec2f &gaze, const cv::Vec2f &target,
        const cv::Mat &entropyImg, const cv::Mat &objImage)
{
    std::string outDir = resultDir;

    // write time
    std::string timeFilename =outDir+"time.txt";
    std::ofstream fTime(timeFilename.c_str(), std::ios::out | std::ios::app);
    fTime << recogTimer.elapsed() << std::endl; 
    fTime.close();
    
    // write gaze angles
    std::string gazeFilename =outDir+"gaze.txt";
    std::ofstream fGaze(gazeFilename.c_str(), std::ios::out | std::ios::app);
    fGaze << gaze[0] << " " <<  gaze[1] << " " << settings.rotationOffset << std::endl; 
    fGaze.close();

    // write estimated viewpoint for each object
    std::vector<cv::Point2f> modes = particleFilter.getModes();
    std::string viewpointFilename = outDir+"viewpoints.txt";
    std::ofstream fVp(viewpointFilename.c_str(), std::ios::out | std::ios::app);
    for (size_t k = 0; k < modes.size(); k++)
    {
        fVp << modes[k].y << " " << modes[k].x << " ";
    }
    fVp << std::endl;
    fVp.close();
    
    // write target position
    std::string targetFilename =outDir+"target.txt";
    std::ofstream ftarget(targetFilename.c_str(), std::ios::out | std::ios::app);
    ftarget << target[0] << " " <<  target[1] << std::endl; 
    ftarget.close();

    // write object names
    std::string objectNameFile = outDir+"objects.txt";
    std::cout << "\twriting to " << objectNameFile << std::endl;
    std::ofstream f(objectNameFile.c_str(), std::ios::out);
    BOOST_FOREACH(const Object &obj, objects)
    {
        f << obj.getName() << std::endl;
    }
    f.close();

    std::string strIterNo = boost::str(boost::format("%02i") % recogIter);

    // write obj probs
    //   Util::writeMatTxt(outDir+strIterNo+"_objprobs.txt", objProbs);
    std::string objProbsFilename =outDir+"objprobs.txt";
    std::ofstream fObjProbs(objProbsFilename.c_str(), std::ios::out | std::ios::app);
    for (int k = 0; k < objProbs.rows; k++)
    {
        fObjProbs << objProbs.at<float>(k) << " "; 
    }
    fObjProbs << std::endl;
    fObjProbs.close();


    // write obj probs KNN
    //std::string objProbsKNN1Filename =outDir+"objprobs-knn-1.txt";
    //std::ofstream fKNN1(objProbsKNN1Filename.c_str(), std::ios::out | std::ios::app);
    //for (int k = 0; k < objProbsKNN1.rows; k++)
    //{
        //fKNN1 << objProbsKNN1.at<float>(k) << " "; 
    //}
    //fKNN1 << std::endl;
    //fKNN1.close();
    std::string objProbsKNN3Filename =outDir+"objprobs-knn.txt";
    std::ofstream fKNN3(objProbsKNN3Filename.c_str(), std::ios::out | std::ios::app);
    for (int k = 0; k < objProbsKNN3.rows; k++)
    {
        fKNN3 << objProbsKNN3.at<float>(k) << " "; 
    }
    fKNN3 << std::endl;
    fKNN3.close();
    //std::string objProbsKNN5Filename =outDir+"objprobs-knn-5.txt";
    //std::ofstream fKNN5(objProbsKNN5Filename.c_str(), std::ios::out | std::ios::app);
    //for (int k = 0; k < objProbsKNN5.rows; k++)
    //{
        //fKNN5 << objProbsKNN5.at<float>(k) << " "; 
    //}
    //fKNN5 << std::endl;
    //fKNN5.close();
    //Util::writeMatTxt(outDir+strIterNo+"_objprobs-knn.txt", objProbsKNN);


    // write particle images
    //for (size_t k = 0; k < particleImgs.size(); k++)
    //{
        //std::string strObjNo = boost::str(boost::format("%02i") % k);
        //std::string particleImgFilename = outDir+strIterNo+"_particles_"+strObjNo+".png";
        //cv::imwrite(particleImgFilename, particleImgs[k]);
    //}

    //// write view similarity images
    //for (size_t k = 0; k < viewSimImgs.size(); k++)
    //{
        //std::string strObjNo = boost::str(boost::format("%02i") % k);
        //std::string viewSimImgFilename = outDir+strIterNo+"_viewsim_"+strObjNo+".png";
        //cv::imwrite(viewSimImgFilename, viewSimImgs[k]);
    //}

    //if (doMotionPlanning)
    //{
        //// write entropy image
        //std::string entropyImgFilename = outDir+strIterNo+"_entropy.png";
        //cv::imwrite(entropyImgFilename, entropyImg);
    //}
    //// write view image
    //std::string objImageFilename = outDir+strIterNo+"_view.png";
    //cv::imwrite(objImageFilename, objImage);
}

ActiveRec::CellType ActiveRec::createCell(double cellE, double cellR, double cellSizeE, 
        double cellSizeR, const cv::Vec2f &currentGaze)
{
    // compute expected entropy for this action
    boost::timer timer;
    cv::Vec2f action = cv::Vec2f(cellE+settings.minElevation,cellR+settings.minRotation) - currentGaze;

    double val;
    if (fastPlanning)
        val = particleFilter.calcExpectedVariance(action);
    else
        //val = currentEntropy - particleFilter.calcExpectedEntropy(action) + 0.0;
        val = 2 - particleFilter.calcExpectedEntropy(action) + 0.0;

    // Temporary fix (Nov 2012):
    // set regions to 0 that there robot cannot reach
    if (cellE <= 0) 
        val = 0;
    if (cellR < 0)
        val = 0;
    //std::cout << "create cell time: " << timer.elapsed() << "\t" << h << std::endl;
    return CellType(cellE, cellR, cellSizeE, cellSizeR, val);
}

cv::Mat ActiveRec::createTargetMap(const cv::Vec2f &currentGaze)
{
    double rangeE = settings.maxElevation - settings.minElevation;
    double rangeR = settings.maxRotation - settings.minRotation;

    cv::Mat utility(rangeE, rangeR, CV_32F, cv::Scalar(0));

    //
    // Grid search
    //

    CellQueue cells;

    // initialize grid search with uniform grid 
    //std::cout << "\t\tinitializing grid search..." << std::endl;
    boost::timer timer;
    int nCellsE = 9;
    int nCellsR = 14;
    double gridSizeE = utility.rows / (double)nCellsE; 
    double gridSizeR = utility.cols / (double)nCellsR;
    for (double cellE = 0+gridSizeE/2; cellE <= utility.rows-gridSizeE/2; cellE += gridSizeE)
    {
        for (double cellR = 0+gridSizeR/2; cellR <= utility.cols-gridSizeR/2; cellR += gridSizeR)
        {
            cells.push_back(createCell(cellE, cellR, gridSizeE, gridSizeR, currentGaze));
        }
    }

    //std::cout << "\t\twhile..." << std::endl;
    while (! cells.empty())
    {
        //std::cout << "\t\t\tsize: " << cells.size() << std::endl;
        int i = 0;
        CellQueue newCells;
        int bestNCells = cells.size() * 0.1;//2;
        cells.sort(ActiveRec::greater_H());
        CellQueue::const_iterator cIter;
        for (cIter = cells.begin(); cIter != cells.end(); ++cIter, ++i)
        {
            const CellType &cell = *cIter;

            if (i >= bestNCells || cell.sizeE <= 1 || cell.sizeR <= 1)
            {
                // write to utility map -> done
                utility(cell.rect()).setTo(cv::Scalar(cell.h));
            }
            else
            {
                // split cell into 4 new cells -> continue
                double cellSizeE = cell.sizeE/2;
                double cellSizeR = cell.sizeR/2;
                newCells.push_back(createCell(cell.e-cellSizeE/2, cell.r-cellSizeR/2, cellSizeE, cellSizeR, currentGaze)); // top left
                newCells.push_back(createCell(cell.e-cellSizeE/2, cell.r+cellSizeR/2, cellSizeE, cellSizeR, currentGaze)); // top right
                newCells.push_back(createCell(cell.e+cellSizeE/2, cell.r-cellSizeR/2, cellSizeE, cellSizeR, currentGaze)); // bottom left
                newCells.push_back(createCell(cell.e+cellSizeE/2, cell.r+cellSizeR/2, cellSizeE, cellSizeR, currentGaze)); // bottom right
            }
        }
        cells = newCells;
    }

    //std::cout << "time grid search: " << timer.elapsed() << std::endl;
    return utility;
}

cv::Vec2f ActiveRec::searchTargetGaze(const cv::Vec2f &currentGaze)
{
    boost::timer timer;
    cv::Mat utility = createTargetMap(currentGaze);

    cv::blur(utility, utility, cv::Size(5,5));
    cv::blur(utility, utility, cv::Size(5,5));
    cv::blur(utility, utility, cv::Size(5,5));
    cv::blur(utility, utility, cv::Size(15,15));
    cv::blur(utility, utility, cv::Size(15,15));

    double minUtility, maxUtility;
    cv::Point locMax;
    cv::minMaxLoc(utility, &minUtility, &maxUtility, 0, &locMax);
    int tE = locMax.y+settings.minElevation;
    int tR = locMax.x+settings.minRotation;

    if (tE == 0) tE = 20;

    std::cout << "\t\tminUtility: " << minUtility << std::endl;
    std::cout << "\t\tmaxUtility: " << maxUtility << std::endl;
    std::cout << "\t\ttarget pos: " << tE << " " << tR << std::endl;

    if (maxUtility < 0.001)
        return cv::Vec2f();

    // make it colorful :)
    cv::Mat dispUtility = Util::GetColorcoded(utility);


    cv::Mat dispUtilityBig;
	cv::resize(dispUtility, dispUtilityBig, cv::Size(), 2, 2, cv::INTER_CUBIC);

    // draw point to mark current position in entropy map
    cv::Point pG(currentGaze[1]-settings.minRotation, currentGaze[0]-settings.minElevation); 
    cv::circle(dispUtilityBig, pG*2, 2, cv::Scalar::all(255), 3);

    // draw cross to mark target position in entropy map
    int l = 2;
    cv::Scalar clCross(255, 255, 255);
    cv::line(dispUtilityBig, 2*(locMax-cv::Point(l,0)), 2*(locMax+cv::Point(l,0)), clCross, 2);
    cv::line(dispUtilityBig, 2*(locMax-cv::Point(0,l)), 2*(locMax+cv::Point(0,l)), clCross, 2);

    cv::imshow("Entropy", dispUtilityBig); 
    cv::waitKey(10);

    // store image for writing to disk
    entropyImg = dispUtilityBig;

    std::cout << "\t\tTime motion planning (s): " << timer.elapsed() << std::endl;

    return cv::Vec2f(tE, tR);
}



bool ActiveRec::loadObjects(const std::string &objectDir)
{
    // search for object directories
    if ( !fs::exists(objectDir) )
    {
        std::cerr << "loadObjects(): Directory '" << objectDir << "' does not exist!" << std::endl;
        return false;
    }

    objects.clear();

    //std::vector<std::string> imagePaths;
    
    fs::path path(objectDir);
    for (fs::directory_iterator itr(path); itr!=fs::directory_iterator(); ++itr)
    {
        if (! fs::is_directory(itr->path()))
            continue; // skip files

        Object object;
        object.load(itr->path().string());
        objects.push_back(object);
    }

    if (objects.empty())
    {
        std::cout << "No object data found. Please record objects first. Run 'explore <obj>' with an object in hand." << std::endl;
        return false;
    }

    if (settings.doBoosting)
    {
        initBoosting();
    }

    return true;
}


void ActiveRec::initBoosting()
{
    // cluster keyframes and save centers
    std::cout << "KMeans clustering..." << std::endl;
    // get one big cv::Mat containing features of all objects
    std::vector<cv::Mat> listFeatures;
    std::vector<int> featureObjId;
    std::vector<std::string> imagePaths;
    for (int k = 0; k < objects.size(); k++)
    {
        cv::Mat features = objects[k].getKeyviewFeatures(); 
        listFeatures.push_back(features);

        std::vector<int> ids;
        ids.resize(features.rows, k);
        featureObjId.insert(featureObjId.end(), ids.begin(), ids.end());
        
        // get paths to image files
        std::vector<std::string> paths = objects[k].getKeyviewPaths();
        for (int i = 0; i < paths.size(); i++)
        {
            imagePaths.push_back(paths[i]);
        }
    }
    cv::Mat features = Util::vstack(listFeatures); 

    int K = 16;
    //int K = 3;
    cv::Mat bestLabels;
    clusterCenters = cv::Mat(K, features.cols, CV_32F, cv::Scalar(0));
    cv::kmeans(features, K, bestLabels, cv::TermCriteria(), 5, cv::KMEANS_RANDOM_CENTERS, clusterCenters); 

    // write cluster assignments to file for debugging
    std::ofstream clusterFile("clusters.txt", std::ios::out);
    for (int i=0; i<bestLabels.rows; i++)
    {
        int clusterId = bestLabels.at<int>(i);
        clusterFile << clusterId << " " << imagePaths[i] << std::endl;
    }
    ////////////

    cv::Mat clusterHists(K, objects.size(), CV_32F, cv::Scalar(0));
    p_co = cv::Mat(objects.size(), K, CV_32F, cv::Scalar(0));

    p_c = cv::Mat(1, K, CV_32F, cv::Scalar(0));

    for (int i=0; i<bestLabels.rows; i++)
    {
        int clusterId = bestLabels.at<int>(i);
        int objId = featureObjId[i];
        clusterHists.at<float>(clusterId, objId) += 1.0/objects[objId].getKeyviewCount();
        p_c.at<float>(clusterId) += 1.0;
        p_co.at<float>(objId, clusterId) += 1.0; 
    }
    p_c /= cv::sum(p_c)[0];

    p_o = cv::Mat(1, objects.size(), CV_32F, cv::Scalar(1));
    for (int objId=0; objId<objects.size(); objId++)
    {
        //p_o.at<float>(objId) = keyviews[objId].rows;
        p_co.row(objId) /= cv::sum(p_co.row(objId))[0];
        //std::cout << p_co.row(objId) << std::endl;
    }
    p_o /= cv::sum(p_o)[0];
    //std::cout << p_o << std::endl;
    //std::cout << p_c << std::endl;


    // normalize cluster distributions
    //for (int c=0; c<K; c++)
    //{
        //clusterHists.row(c) /= cv::sum(clusterHists.row(c))[0];
        //std::cout << clusterHists.row(c) << std::endl;
    //}

}

cv::Mat ActiveRec::calcBoostingFactors(const cv::Mat &observation)
{
        double min_dist = std::numeric_limits<double>::max();
        int bestClusterId = 0;

        // weight particles according to distance to cluster centers
        cv::Mat dists(1, clusterCenters.rows, CV_32F, cv::Scalar(0));
        for (int i=0; i<clusterCenters.rows; i++)
        {
            double d = cv::norm(observation, clusterCenters.row(i), cv::NORM_L2);
            dists.at<float>(i) = d;
            if (d < min_dist)
            {
                min_dist = d;
                bestClusterId = i;
            }
        }
        dists /= 20;
        dists = dists.mul(dists);
        //std::cout << "center dists: " << dists << std::endl;
        cv::exp(-dists, dists);
        //dists /= cv::sum(dists)[0];
        //std::cout << "center probs: " << dists << std::endl;

        cv::Mat p_oc(objectCount(), 1, CV_32F, cv::Scalar(0));
        for (int cId = 0; cId < dists.cols; cId++)
        {
            for (int objId = 0; objId < objectCount(); objId++)
            {
                p_oc.at<float>(objId) += dists.at<float>(cId) 
                    * (p_co.at<float>(objId, cId) * p_o.at<float>(objId)) / std::sqrt(p_c.at<float>(cId));
            }
        }
        
        return p_oc;
}

std::vector<cv::Mat> ActiveRec::createViewSimImages(const cv::Mat &feature, const cv::Vec2f &gaze)
{
    std::vector<cv::Mat> imgs;
    BOOST_FOREACH(const Object &o, objects)
    {
        imgs.push_back(o.createViewSimImage(feature,gaze));
    }
    return imgs;
}


#if 0
void ActiveRec::showBestKeyframes()
{
    // show best view for each object
    std::vector<int> bestParticleId;
    std::vector<float> bestParticleW;
    bestParticleId.resize(objectCount());
    bestParticleW.resize(objectCount());
    const cv::Mat particles = particleFilter.getParticles();
    const cv::Mat W = particleFilter.getWeights();
    for (int i = 0; i < particles.rows; i++)
    {
        int k = particles.at<float>(i,0);
        if (W.at<float>(i) > bestParticleW[k])
        {
            bestParticleW[k] = W.at<float>(i);
            bestParticleId[k] = i;
        }
    }
    for (size_t k = 0; k < bestParticleId.size(); k++)
    {
        float e = particles.at<float>(bestParticleId[k],1);
        float r = particles.at<float>(bestParticleId[k],2);
        cv::Mat dists(keyviewPos[k].rows, 1, CV_32F);
        double ei, ri;
        for (int i = 0; i < keyviews[k].rows; i++)
        {
            ei = keyviewPos[k].at<float>(i,0);
            ri = keyviewPos[k].at<float>(i,1);
            dists.at<float>(i) = Util::calcCentralAngle(e, ei, r, ri);
        }
        cv::Mat sortedIds;
        cv::sortIdx(dists, sortedIds, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING); 
        int id = sortedIds.at<int>(0);
        std::string strObjNo = boost::lexical_cast<std::string>(k);
        cv::imshow("obj "+strObjNo +"best particle", keyviewImage[k][id]);
    }
}
#endif



