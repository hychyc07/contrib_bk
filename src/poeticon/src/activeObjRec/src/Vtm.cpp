// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file Vtm.cpp
 * @brief implementation of the Vtm methods.
 */

#include <iostream>
#include <fstream>
#include <time.h>
#include <stdlib.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/filesystem.hpp>

#include <iCub/Vtm.h>
#include <iCub/FeatureExtractor.h>

#define USE_SHAPE_VTM 1

namespace fs = boost::filesystem;

void calculateTransition(const Vector& currentState, const Vector& lastState, Vector& transition)
{
    if (currentState.size() != lastState.size())
    {
        std::cerr << "calculateTransition: Both input vectors must be of the same size." << std::endl;
        return;
    }

    transition.resize(currentState.size());
    for (int i = 0; i < currentState.size(); ++i)
    {
        transition[i] = currentState[i] - lastState[i];
    }
}

void invertTransition(const Vector& transition, Vector& invTransition)
{
    invTransition.resize(transition.size());
    for (int i = 0; i < transition.size(); ++i)
    {
        invTransition[i] = -transition[i];
    }
}
    

void Vtm::clear()
{
    keyframes.clear();
    features.clear();
    jointStates.clear();
    transitions.clear();
}


cv::Mat Vtm::extractFeature(const cv::Mat& img)
{
#if USE_SHAPE_VTM
    cv::Mat featureShape = FeatureExtractor::extract(img, FeatureExtractor::PHOG);
    cv::Mat featureColor = FeatureExtractor::extract(img, FeatureExtractor::COLOR);
    cv::Mat feature(1, featureShape.cols+featureColor.cols, CV_32F, cv::Scalar(0));
    cv::Mat dstShape = feature.colRange(0, featureShape.cols);
    cv::Mat dstColor = feature.colRange(featureShape.cols, feature.cols);
    featureShape *= 0.1;
    featureShape.copyTo(dstShape);
    featureColor.copyTo(dstColor);
#else
    cv::Mat feature = FeatureExtractor::extract(img, FeatureExtractor::COLOR);
#endif
    return feature;
}

void Vtm::addKeyframe(const cv::Mat& keyframe, const Vector& joints)
{
    int currKeyframeNo = keyframes.size();
    
    keyframes.push_back(keyframe.clone());

    cv::Mat feature = extractFeature(keyframe);

    //cv::Mat feature = FeatureExtractor::extract(keyframe, FeatureExtractor::COLOR);
    features.push_back(feature.clone());

    // update transition map
    if (currKeyframeNo == 0)
    {
        Vector emptyTransition;
        emptyTransition.resize(joints.size(), 0);
        transitions[std::make_pair(0, 0)] = emptyTransition;
        jointStates.push_back(joints);
    }
    else
    {
        Vector lastJointState = jointStates[jointStates.size()-1];
        jointStates.push_back(joints);
        
        Vector currentTransition;
        calculateTransition(joints, lastJointState, currentTransition); 
        transitions[std::make_pair(currKeyframeNo-1, currKeyframeNo)] = currentTransition;  

        Vector invertedTransition;
        invertTransition(currentTransition, invertedTransition);
        transitions[std::make_pair(currKeyframeNo, currKeyframeNo-1)] = invertedTransition;

        // calculate the transitions from previous keyframes to the current one.
        // -> the keyframes are linked together: (A -> B) + (B -> C) = (A -> C)
        for (int mapRow = currKeyframeNo-2; mapRow >= 0; mapRow--)
        {
            Vector transition;
            transition.resize(joints.size());
            for (int dof = 0; dof < joints.size(); dof++)
            {
                transition[dof] = transitions[std::make_pair(mapRow+1, currKeyframeNo)][dof]
                                + transitions[std::make_pair(mapRow, mapRow+1)][dof];

            }
            transitions[std::make_pair(mapRow, currKeyframeNo)] = transition;

            Vector invTransition;
            invertTransition(transition, invTransition);
            transitions[std::make_pair(currKeyframeNo, mapRow)] = invTransition;
        }
    }
}

bool Vtm::save(const std::string &path, const std::string &objectName) const
{
    if (keyframes.size() != features.size())
    {
        std::cout << "WARNING: Number of keyframes and number of features do not match!" << std::endl;
        std::cout << "Number of keyframes: " << keyframes.size() << std::endl;
        std::cout << "Number of features: " << features.size() << std::endl;
    }

    fs::path objectDir(path);
    objectDir /= objectName;

    // create directory for object 
    try
    {
        fs::create_directories(objectDir);
    }
    catch(std::exception &e)
    {
        std::cerr << "ERROR - Vtm::save: Could not create output directory " << objectDir << std::endl; 
        std::cerr << "\t ... " << e.what() << std::endl; 
        return false;
    }

    // write vtm info
    std::string nameInfofile = (objectDir / "info.txt").string();
    std::ofstream infofile(nameInfofile.c_str(), std::ios::out);
    if (!infofile.is_open())
    {
        std::cerr << "Vtm.save(): Could not open file '" << nameInfofile << "' for writing." << std::endl;
        return false;
    }
    infofile << keyframes.size() << std::endl; 
    infofile.close();
    std::cout << "wrote vtm info to " << nameInfofile << std::endl;

    // write keyframes
    for (int i = 0; i < keyframes.size(); ++i)
    {
        char filename[256];
        sprintf(filename, "%s/%s/keyframe_%03d.png", path.c_str(), objectName.c_str(), i);
        std::cout << "writing keyframe to " << filename << std::endl;
        cv::imwrite(filename, keyframes[i]); 
    }

    // write features
    for (int i = 0; i < features.size(); ++i)
    {
        char filename[256];
        sprintf(filename, "%s/%s/feature_%03d.xml", path.c_str(), objectName.c_str(), i);
        cv::FileStorage storage(filename, cv::FileStorage::WRITE);
        storage << "lab_fourier" << features[i];
        storage.release();
        std::cout << "wrote feature to " << filename << std::endl;
    }

    // write transitions
    std::string nameTransitionFile = (objectDir / "transitions.txt").string();
    std::ofstream file(nameTransitionFile.c_str(), std::ios::out);
    std::map<std::pair<int, int>, Vector>::const_iterator tIter;
    for (tIter = transitions.begin(); tIter != transitions.end(); ++tIter)
    {
        file << tIter->first.first << " " << tIter->first.second << " ";

        for (int i = 0; i < tIter->second.size(); ++i)
        {
            file << tIter->second[i] << " ";
        }
        file << std::endl;
    }
    file.close();
    std::cout << "wrote transitions to " << nameTransitionFile << std::endl;

    return true;
}

bool Vtm::load(const std::string &path, const std::string& objectName)
{
    // delete old data
    keyframes.clear();
    jointStates.clear();
    transitions.clear();

    this->objName = objectName;

    // get number of keyframes
    int nKeyframes;
    std::string nameInfofile = path + "/" + objectName +"/info.txt";
    //std::cout << "loading " << nameInfofile << "..." << std::endl; 
    std::ifstream infofile(nameInfofile.c_str(), std::ios::in);
    if (! infofile.is_open())
    {
        std::cerr << "Could not open keyframe info file " << nameInfofile << std::endl;
        return false;
    }
    infofile >> nKeyframes;
    infofile.close();

    // read keyframes
    for (int i = 0; i < nKeyframes; ++i)
    {
        char filename[256];
        sprintf(filename, "%s/%s/keyframe_%03d.png", path.c_str(), objectName.c_str(), i);
        //std::cout << "loading " << filename << "..." << std::endl; 
        cv::Mat kf = cv::imread(filename);
        if (kf.empty())
        {
            std::cout << "Keyframe file " << filename << " not found" << std::endl;
            return false;
        }
        keyframes.push_back(kf);
    }

    // read features
    for (int i = 0; i < nKeyframes; ++i)
    {
        char filename[256];
        sprintf(filename, "%s/%s/feature_%03d.xml", path.c_str(), objectName.c_str(), i);
        //std::cout << "featfile " << filename << std::endl;
        //cv::FileStorage fs(filename, cv::FileStorage::READ);
        //if (! fs.isOpened())
        //{
            //std::cerr << "Could not open feature file " << filename << std::endl;
            //return false;
        //}
        //cv::Mat feat;
        //std::cout << "foo" << std::endl;
        //fs["lab_fourier"] >> feat;
        //std::cout << "bar" << std::endl;
        //features.push_back(feat);
        //fs.release();
        
        // have to use c-style routines for loading because new cpp version seems to be broken
        CvFileStorage *storage = cvOpenFileStorage(filename, 0, CV_STORAGE_READ);
        CvMat *mat = (CvMat*)cvReadByName(storage, 0, "lab_fourier");
        //cv::Mat m = mat;
        //cv::imshow("mat", m);
        //cv::waitKey();
        features.push_back(cv::Mat(mat).clone());
        cvReleaseFileStorage(&storage);
        cvReleaseMat(&mat);
    }

    // read transitions
    std::string nameTransitionFile = path + "/" + objectName + "/transitions.txt";
    std::ifstream file(nameTransitionFile.c_str(), std::ios::in);
    if (! file.is_open())
    {
        std::cerr << "Could not open transition file " << nameTransitionFile << std::endl;
        return false;
    }
    while (! file.eof())
    {
        int kfFrom, kfTo;
        file >> kfFrom;
        file >> kfTo;
        int nArmJoints = 16;
        Vector t(nArmJoints);
        for (int joint = 0; joint < nArmJoints; ++joint)
        {
            file >> t[joint];
        }
        transitions[std::make_pair(kfFrom, kfTo)] = t;  
    }
    file.close();
    
    return true;
}

