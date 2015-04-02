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
 * @file Object.cpp
 * @brief implementation of the Object class.
 */

#include <iostream>
#include <vector>
#include <fstream>

#include <boost/lexical_cast.hpp>
#include <boost/timer.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include "iCub/ActiveRec.h"
#include "iCub/Util.h"
#include "iCub/GlobalSettings.h"
#include "iCub/cvmat_serialization.h"


#define USE_COLOR 1
#define PCA 0


cv::Mat Object::extractFeature(const cv::Mat &img)
{
#if USE_COLOR_AND_SHAPE
    cv::Mat featureShape = FeatureExtractor::extract(img, FeatureExtractor::PHOG);
    cv::Mat featureColor = FeatureExtractor::extract(img, FeatureExtractor::COLOR);
    cv::Mat feature(1, featureShape.cols+featureColor.cols, CV_32F, cv::Scalar(0));
    cv::Mat dstShape = feature.colRange(0, featureShape.cols);
    cv::Mat dstColor = feature.colRange(featureShape.cols, feature.cols);
    featureShape *= 0.1;
    featureShape.copyTo(dstShape);
    featureColor.copyTo(dstColor);
#elif USE_COLOR
    cv::Mat feature = FeatureExtractor::extract(img, FeatureExtractor::COLOR);
#elif USE_SHAPE 
    cv::Mat feature = FeatureExtractor::extract(img, FeatureExtractor::PHOG);
#elif USE_FOURIER 
    cv::Mat feature = FeatureExtractor::extract(img, FeatureExtractor::FOURIER);
#endif

#if PCA
    //std::cout << "\treducing dimensionality..." << std::endl;
    //g = g.colRange(0, 6400);
    feature = pca.project(feature);
#endif
    //std::cout << "\tdone." << std::endl;
    
    return feature;
}


void Object::addKeyview(const cv::Mat &image, double e, double r)
{
    keyviewImages.push_back(image);
    keyviewViewpoints.push_back(Viewpoint(e,r));
}


bool Object::save(const std::string &objectDir) const
{
    try
    {
        fs::create_directories(objectDir);
    }
    catch(std::exception &e)
    {
        std::cerr << "ERROR - Object::save: Could not create output directory " << objectDir << std::endl; 
        std::cerr << "\t ... " << e.what() << std::endl; 
        return false;
    }

    std::cout << "Saving object to: " << objectDir << std::endl;
    for (int i = 0; i < keyviewImages.size(); i++)
    {
        Viewpoint vp = keyviewViewpoints[i];
        saveKeyview(objectDir, keyviewImages[i], vp[0], vp[1]);
    }
    std::cout << "Done." << std::endl;
}

void Object::saveKeyview(const fs::path &objectDir, const cv::Mat &image, double e, double r) const
{
    // extract features
    cv::Mat ftComposite  = FeatureExtractor::extract(image, FeatureExtractor::COMPOSITE);
    cv::Mat ftFourier    = FeatureExtractor::extract(image, FeatureExtractor::FOURIER);
    cv::Mat ftPhog       = FeatureExtractor::extract(image, FeatureExtractor::PHOG);
    cv::Mat ftColor      = FeatureExtractor::extract(image, FeatureExtractor::COLOR);

    // create directory for object 
    fs::create_directory(objectDir);

    // update object info file
    int viewNo = 0;
    std::string nameInfofile = (objectDir / "info.txt").string();
    // read old value
    std::ifstream infofile_in(nameInfofile.c_str(), std::ios::in);
    if (infofile_in.is_open())
    {
        infofile_in >> viewNo;
        infofile_in.close();
    }
    // write new value
    std::ofstream infofile(nameInfofile.c_str(), std::ios::out);
    if (!infofile.is_open())
    {
        std::cerr << "Could not open file '" << nameInfofile << "' for writing." << std::endl;
        return;
    }
    infofile << viewNo+1 << std::endl; 
    infofile.close();

    // write image
    fs::path filenameImg = objectDir / boost::str(boost::format("keyframe_%03d.png") % (viewNo));
    cv::imwrite(filenameImg.string(), image); 

    // write features
    fs::path filenameFourier    = objectDir / boost::str(boost::format("fourier_%03d.xml") % (viewNo));
    fs::path filenamePhog       = objectDir / boost::str(boost::format("phog_%03d.xml") % (viewNo));
    fs::path filenameComposite  = objectDir / boost::str(boost::format("composite_%03d.xml") % (viewNo));
    fs::path filenameColor      = objectDir / boost::str(boost::format("color_%03d.xml") % (viewNo));
    cv::FileStorage storageFourier(filenameFourier.string(), cv::FileStorage::WRITE);
    cv::FileStorage storagePhog(filenamePhog.string(), cv::FileStorage::WRITE);
    cv::FileStorage storageComposite(filenameComposite.string(), cv::FileStorage::WRITE);
    cv::FileStorage storageColor(filenameColor.string(), cv::FileStorage::WRITE);
    storageFourier   << "lab_fourier" << ftFourier;
    storagePhog      << "phog"        << ftPhog;
    storageComposite << "composite"   << ftComposite;
    storageColor     << "color"       << ftColor;
    storageFourier.release();
    storagePhog.release();
    storageComposite.release();
    storageColor.release();

    
    // write robot data
    fs::path filenamePose  = objectDir / boost::str(boost::format("pose_%03d.ov") % (viewNo));
    std::ofstream file(filenamePose.string().c_str(), std::ios::out);
    if (!file.is_open())
    {
        std::cerr << "Could not open file '" << filenamePose.string() << "' for writing." << std::endl;
        return;
    }
    // gaze angles
    file << e << " " << r << " ";
    // hand coordinates
    //file << handX.toString() << " " << eyeX.toString() << " ";
    //// torso joints
    //file << torsoQ.toString() << " " ;
    //// arm joints
    //for (int i = 0; i < NUM_ARM_JOINTS; i++)
    //{
        //file << armQ[i] << " ";
    //}
    file <<  "\n";
    file.close();
}

void Object::extractKeyviewFeatures()
{
    keyviewFeatures = cv::Mat(keyviewImages.size(), 
            FeatureExtractor::featureLength(FEATURE_TYPE), 
            CV_32F, cv::Scalar::all(0));
    for (int i = 0; i< keyviewImages.size(); i++)
    {
        cv::Mat f = extractFeature(keyviewImages[i]);
        cv::Mat dstRow = keyviewFeatures.row(i);
        f.copyTo(dstRow); 
    }
}

cv::Mat Object::loadFeatures(const std::string &objectPath, FeatureExtractor::FeatureType featureType)
{
        // get number of keyframes
        int nKeyframes;
        std::string nameInfofile = objectPath + "/info.txt" ;
        std::ifstream infofile(nameInfofile.c_str(), std::ios::in);
        if (! infofile.is_open())
        {
            std::cerr << "Could not open keyframe info file " << nameInfofile << std::endl;
        }
        infofile >> nKeyframes;
        infofile.close();

        cv::Mat features(nKeyframes, FeatureExtractor::featureLength(featureType), 
                CV_32F, cv::Scalar::all(0));

        // read views
        for (int kfNo = 0; kfNo < nKeyframes; kfNo++)
        {
            std::string filename;
            std::string nodename;
            if (featureType == FeatureExtractor::FOURIER)
            {
                filename = boost::str(boost::format("fourier_%03d.xml") % kfNo);
                nodename = "lab_fourier";
            }
            else if (featureType == FeatureExtractor::PHOG)
            {
                filename = boost::str(boost::format("phog_%03d.xml") % kfNo);
                nodename = "phog";
            }
            else if (featureType == FeatureExtractor::COMPOSITE)
            {
                filename = boost::str(boost::format("composite_%03d.xml") % kfNo);
                nodename = "composite";
            }
            else if (featureType == FeatureExtractor::COLOR)
            {
                filename = boost::str(boost::format("color_%03d.xml") % kfNo);
                nodename = "color";
            }
            std::string pathnameFeature = objectPath + "/" + filename;

            // have to use c-style routines for loading because new cpp version seems to be broken
            // TODO: try with new opencv version
            CvFileStorage *storage = cvOpenFileStorage(pathnameFeature.c_str(), 0, CV_STORAGE_READ);
            if (! storage)
            {
                std::cerr << "Could not open file '" << pathnameFeature << "' for reading." << std::endl;
            }
            CvMat *mat = (CvMat*)cvReadByName(storage, 0, nodename.c_str());
            //viewFeatures.push_back(cv::Mat(mat).clone());
            cv::Mat dstRow = features.row(kfNo);
            cv::Mat(mat).copyTo(dstRow);
            cvReleaseFileStorage(&storage);
            cvReleaseMat(&mat);
        }

        return features;
}


bool Object::load(const std::string &objectPath)
{
    if (! fs::exists(objectPath))
    {
        std::cerr << "Object directory " << objectPath << " not found!" << std::endl;
        return false;
    }

    dataDir = objectPath;

    objectName = fs::path(objectPath).filename().string();
    std::cout << "Found data for object '" << objectName << "'" << std::endl;

    // get number of keyframes
    int nKeyframes;
    std::string nameInfofile = ( fs::path(objectPath) / "info.txt" ).string();
    std::ifstream infofile(nameInfofile.c_str(), std::ios::in);
    if (! infofile.is_open())
    {
        std::cerr << "Could not open keyframe info file " << nameInfofile << std::endl;
        return false;
    }
    infofile >> nKeyframes;
    infofile.close();

#if ALWAYS_LOAD_FEATURES
    // load features
    cv::Mat shapeFeatures = loadFeatures(objectPath, FeatureExtractor::PHOG);
    cv::Mat colorFeatures = loadFeatures(objectPath, FeatureExtractor::COLOR);
    cv::Mat fourierFeatures = loadFeatures(objectPath, FeatureExtractor::FOURIER);

    CV_Assert(colorFeatures.rows == nKeyframes);
    CV_Assert(shapeFeatures.rows == colorFeatures.rows);

#if USE_COLOR_AND_SHAPE
    keyviewFeatures = cv::Mat(nKeyframes, shapeFeatures.cols + colorFeatures.cols, CV_32F, cv::Scalar(0));
    cv::Mat dstShape = keyviewFeatures.colRange(0, shapeFeatures.cols);
    cv::Mat dstColor = keyviewFeatures.colRange(shapeFeatures.cols, keyviewFeatures.cols);
    shapeFeatures *= 0.1;
    shapeFeatures.copyTo(dstShape);
    colorFeatures.copyTo(dstColor);

    //std::cout << "Feature sums" << std::endl;
    //std::cout << cv::sum(shapeFeatures)[0] << std::endl;
    //std::cout << cv::sum(colorFeatures)[0] << std::endl;

#elif USE_COLOR
    keyviewFeatures = colorFeatures;
#elif USE_SHAPE
    keyviewFeatures = shapeFeatures;
#elif USE_FOURIER
    keyviewFeatures = fourierFeatures;
#endif

#endif

    // load images and viewpoint coords
    for (int kfNo = 0; kfNo < nKeyframes; kfNo++)
    {
        // load image
        fs::path filenameImage  = boost::str(boost::format("%s/keyframe_%03d.png") % objectPath % kfNo);
        if (! fs::exists(filenameImage))
        {
            std::cerr << "Could not open file '" << filenameImage.string() << "' for reading." << std::endl;
            return false;
        }
        cv::Mat img = cv::imread(filenameImage.string(), -1); 
        keyviewImages.push_back(img);
        keyviewPaths.push_back(filenameImage.string());

        // load viewpoint
        fs::path filenamePose  = boost::str(boost::format("%s/pose_%03d.ov") % objectPath %  kfNo);
        std::ifstream file(filenamePose.string().c_str(), std::ios::in);
        if (!file.is_open())
        {
            std::cerr << "Could not open file '" << filenamePose.string() << "' for reading." << std::endl;
            return false;
        }
        double e, r;
        Vector handX(3);
        Vector eyeX(3);
        Vector joints(10);
        file >> e >> r;
        //file >> handX[0] >> handX[1] >> handX[2];
        //file >> eyeX[0] >> eyeX[1] >> eyeX[2];
        //for (int i = 0; i < 10; i++)
        //{
        //file >> joints[i];
        //}

        keyviewViewpoints.push_back(Viewpoint(e,r));
    }

#if PCA
    // run pca to reduce dimensionality of feature descriptors
    std::cout << "prepare pca input mat" << std::endl;
    cv::Mat pcaInput = Util::vstack(keyviews);
    //pcaInput = pcaInput.colRange(0,6400);
    int pcaDims = std::min(pcaInput.rows, 200);
    std::cout << "pca dims: " << pcaDims << std::endl;
    boost::timer timer;
    pca(pcaInput, cv::Mat(), CV_PCA_DATA_AS_ROW, pcaDims);
    std::cout << "\treducing dimensionality of keyframe features..." << std::endl;
    for (int k = 0; k < keyviews.size(); k++)
    {
        cv::Mat features = keyviews[k];//.colRange(0, 6400);
        cv::Mat features_reduced(features.rows, pcaDims, features.type());
        for (int v = 0; v < features.rows; v++)
        {
            cv::Mat in = features.row(v);
            cv::Mat out = features_reduced.row(v);
            pca.project(in, out);
            //std::cout << out << std::endl;
        }
        keyviews[k] = features_reduced;
        keyviewsOrig.push_back(features);
    }
    //std::cout << pca.eigenvalues << std::endl;
    std::cout << "PCA time: " << timer.elapsed() << std::endl;
#endif

    extractKeyviewFeatures();

    buildViewMap();
}


void Object::buildViewMap()
{
    // try to load view map
    fs::path vmPath = fs::path(dataDir) / "viewmap.bin";
    std::cout << "Looking for view map file: " << vmPath << std::endl;
    if (fs::exists(vmPath))
    {
        std::ifstream in(vmPath.string().c_str(), std::ios::in | std::ios::binary);
        boost::archive::binary_iarchive ar(in);
        ar & viewMap;
        in.close();
    } 
    else
    {
        // build view map
        std::cout << "Building view map " << objectName << std::endl;
        viewMap.resize(rangeE*rangeR);
        for (int e = 0; e < rangeE; e++)
        {
            for (int r = 0; r < rangeR; r++)
            {
                viewMap[e*rangeR+r] = interpolateView(e+minElevation,r+minRotation);
            }
        }
        // save view map
        std::cout << "Saving to " << vmPath << std::endl;
        std::ofstream out(vmPath.string().c_str(), std::ios::out | std::ios::binary);
        boost::archive::binary_oarchive ar(out);
        ar & viewMap;
        out.close();
    }
}


cv::Mat Object::interpolateView(double e, double r) const
{
    // find k-nearest neighbor views
    //
    // linear search
    cv::Mat dists(keyviewViewpoints.size(), 1, CV_32F);
    double ei, ri;
    for (int i = 0; i < keyviewViewpoints.size(); i++)
    {
        ei = keyviewViewpoints[i][0];
        ri = keyviewViewpoints[i][1];
        dists.at<float>(i) = Util::calcCentralAngle(e, ei, r, ri);
    }
    cv::Mat sortedIds;
    cv::Mat sortedDists;
    cv::sortIdx(dists, sortedIds, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING); 
    cv::sort(dists, sortedDists, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING); 

    int KNN = 10;
    //cv::Mat nnids = sortedDists.rowRange(0,KNN);
    //cv::normalize(nnids, nnids, 1.0, 0.0, cv::NORM_MINMAX);
    double sumD = 0;
    for (int k = 0; k < KNN; k++)
    {
        //int v = sortedIds.at<int>(k);
        sumD += sortedDists.at<float>(k);
    }

    // calc weightes
    //std::cout << "sumD " << sumD << std::endl;
    //std::cout << newW << std::endl;
    double sumWeights = 0;
    double d;
    cv::Mat KF;
    cv::Mat interpolatedView(1, getFeatureLength(), CV_32F, cv::Scalar::all(0));
    // calc weighted avg
    for (int k = 0; k < KNN; k++)
    {
        int v = sortedIds.at<int>(k);
        KF = keyviewFeatures.row(v);
        CV_Assert(KF.cols == getFeatureLength());
        //double d = 1 - (sortedDists.at<float>(k) / sumD);//cv::norm(P,KF,cv::NORM_L2);
        if (sumD > 0 && KNN > 1)
          d =  1 - sortedDists.at<float>(k) / sumD;//sortedDists.at<float>(KNN);
        else 
          d =  1;
        sumWeights += d;
        //std::cout << "d " << d << std::endl;
        interpolatedView += d * KF;
    }
    if (sumWeights > 0)
    {
        interpolatedView /= sumWeights;
    }

    return interpolatedView;
}


double Object::matchKNN(const cv::Mat &feature, int knn) const
{
    CV_Assert(knn > 0);

    cv::Mat dists(keyviewFeatures.rows, 1, CV_32F, cv::Scalar(0));
    for (int i = 0; i < keyviewFeatures.rows; i++)
    {
        double d = cv::norm(feature, keyviewFeatures.row(i), cv::NORM_L2)/(feature.cols);
        //d = std::exp(-12*d);
        d = std::exp(-featureScaling*d);
        if (d != d) // check for NAN
            d = 0;
        dists.at<float>(i) = d;
    }

    //cv::normalize(dists, dists, 0, 1, cv::NORM_MINMAX);
    
    cv::Mat sortedIds;
    cv::sortIdx(dists, sortedIds, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING); 

    double sum = 0;
    for (int n = 0; n < knn; n++)
    {
        int id = sortedIds.at<int>(n);
        sum += dists.at<float>(id);
    }

    return sum/knn;
}


cv::Mat Object::createViewSimImage(const cv::Mat &feature, const cv::Vec2f &gaze) const
{
    cv::Rect roi = cv::Rect(minRotation, minElevation, maxRotation-minRotation, maxElevation-minElevation);

    cv::Mat img(roi.height, roi.width, CV_32F, cv::Scalar::all(0));

    for (int e = 0; e < roi.height; e++)
    {
        for (int r = 0; r < roi.width; r++)
        {
            img.at<float>(e,r) = matchFeature(e+roi.y, r+roi.x , feature);
        }
    }

    img = Util::GetColorcoded(img, 0, 1);

    // mark gaze
    cv::Point p(gaze[1]-roi.x, gaze[0]-roi.y);
    cv::circle(img, p, 3, cv::Scalar(0,0,0), -1);  

    // mark keyframes
    BOOST_FOREACH(const Viewpoint &vp, keyviewViewpoints)
    {
        cv::circle(img, cv::Point(vp[_R_]-roi.x, vp[_E_]-roi.y), 1, cv::Scalar(0,0,0), 1);
    }

    return img;
}

