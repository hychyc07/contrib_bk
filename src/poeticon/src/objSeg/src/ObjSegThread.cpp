// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Bjoern Browatzki
 * email:   bjoern.Browatzki@tuebingen.mpg.de
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
 * @file ObjSegThread.cpp
 * @brief 
 */

#include "maxflow-v3.02/graph.h"
typedef MaxFlow::Graph<double,double,double> GraphType;

#include "iCub/ObjSegThread.h"
#include <fstream>
#include <functional>
#include <boost/filesystem.hpp>
#include <boost/timer.hpp>
#include <stdlib.h>

#include "opencv2/highgui/highgui.hpp"
#include <cv.h>

using namespace yarp::os;
using namespace yarp::sig;
namespace fs = boost::filesystem;


bool ObjSegThread::threadInit()
{
    std::cout << "Initializing thread..." << std::endl;

    // Head 

    if (! robotHead.isValid())
    {
        std::cout << "Cannot connect to robot head" << std::endl;  
        std::cout << "Device not available.  Here are the known devices:" << std::endl;
        std::cout << Drivers::factory().toString() << std::endl;
        return false;
    }

    if (! robotHead.view(encHead))
    {
        std::cout << "Cannot get interface to the head" << std::endl;  
        robotHead.close();
        return false;
    }


    // Torso 

    if (! robotTorso.isValid()) 
    {
        std::cout << "Cannot connect to robot torso" << std::endl;  
        std::cout << "Device not available.  Here are the known devices:" << std::endl;
        std::cout << Drivers::factory().toString() << std::endl;
        return false;
    }

    if (! robotTorso.view(encTorso))
    {
        std::cout << "Cannot get interface to torso" << std::endl;  
        robotTorso.close();
        return false;
    }

    // Arm
    if (! robotArm.isValid()) 
    {
        std::cout << "Cannot connect to robot torso" << std::endl;  
        std::cout << "Device not available.  Here are the known devices:" << std::endl;
        std::cout << Drivers::factory().toString() << std::endl;
        return false;
    }
    if (! robotArm.view(encArm))
    {
        std::cout << "Cannot get interface to torso" << std::endl;  
        robotArm.close();
        return false;
    }

    // Arm cartesian
    if (! armCartDriver.view(armCart))
    {
        std::cout <<  "Cannot get cartesian interface to the arm" << std::endl;  
        armCartDriver.close();
        return false;
    }


    numClusters_bkg  = rf.check("num_clusters_bkg",Value(5)).asInt();
    numClusters_obj  = rf.check("num_clusters_obj",Value(5)).asInt();
    w_bkg  = rf.check("w_bkg",Value(1.5)).asDouble();
    w_obj  = rf.check("w_obj",Value(1.0)).asDouble();
    kernelSizeGmm_bkg    = rf.check("kernelsize_bkg", Value(1)).asInt();
    kernelSizeGmm_obj    = rf.check("kernelsize_obj", Value(1)).asInt();

    blurLevel            = rf.check("blur_level", Value(3)).asInt();
    minSegmentSize       = rf.check("minSegmentSize", Value(20*20)).asInt();
    //resultWindowSize     = rf.check("resultWindowSize", Value(160)).asInt();
    scaleFactor          = rf.check("scaleFactor", Value(1.0)).asDouble();
    display              = rf.check("display", Value(0)).asInt();
    maxSamples           = rf.check("max_samples", Value(1000)).asInt();

    w_edge           = rf.check("w_edge", Value(0.15)).asDouble();

    std::string strCamera  = rf.check("camera", Value("left")).asString().c_str();
    camera  = strCamera == "left" ? CAM_LEFT : CAM_RIGHT;
    
    lowDisparity         = rf.check("low_disparity", Value(150)).asInt();
    highDisparity        = rf.check("high_disparity", Value(150)).asInt();

    // object bounding box in regard to hand reference frame
    min_bbox_x = rf.check("min_bbox_x", Value(0.08)).asDouble(); // m
    min_bbox_y = rf.check("min_bbox_y", Value(0.12)).asDouble(); // m
    min_bbox_z = rf.check("min_bbox_z", Value(0.08)).asDouble(); // m
    max_bbox_x = rf.check("max_bbox_x", Value(0.08)).asDouble(); // m
    max_bbox_y = rf.check("max_bbox_y", Value(0.12)).asDouble(); // m
    max_bbox_z = rf.check("max_bbox_z", Value(0.08)).asDouble(); // m

    // bounding box center in regard to end effector position 
    max_bbox_offset_x = rf.check("max_bbox_offset_x", Value(0.0)).asDouble();
    max_bbox_offset_y = rf.check("max_bbox_offset_y", Value(0.0)).asDouble();
    max_bbox_offset_z = rf.check("max_bbox_offset_z", Value(0.0)).asDouble();
    min_bbox_offset_x = rf.check("min_bbox_offset_x", Value(0.0)).asDouble();
    min_bbox_offset_y = rf.check("min_bbox_offset_y", Value(0.0)).asDouble();
    min_bbox_offset_z = rf.check("min_bbox_offset_z", Value(0.0)).asDouble();

    std::string modality=rf.check("modality",Value("lab")).asString().c_str();
    if(modality=="rgb")
    {
        std::cout << "rgb bkg segmentation" << std::endl;
        mod=MODE_RGB;
    }
    else if(modality=="yuv")
    {
        std::cout << "yuv bkg segmentation" << std::endl;
        mod=MODE_YUV;
    }
    else if (modality=="hsv")
    {
        std::cout << "hsv bkg segmentation" << std::endl;
        mod=MODE_HSV;
    }
    else if (modality=="lab")
    {
        std::cout << "lab bkg segmentation" << std::endl;
        mod=MODE_LAB;
    }
    else 
    {
        std::cout << "Color space not set. Using lab as default." << std::endl;
        mod=MODE_LAB;
    }

    dImg = 0;

    std::cout << "done" << std::endl;

    return true;
}




void ObjSegThread::run() 
{

    // get RGB camera image
    if (camLeftPort->getInputCount() == 0)
        return; 
    ImageOf<PixelBgr> *image = camLeftPort->read(false);
    if (! image)
        return;
    cv::Mat img_rgb = (IplImage*)image->getIplImage();
    if (img_rgb.empty())
        return;
    if (cv::countNonZero(img_rgb.reshape(1,0)) == 0)
        return;

    // get disparity image from stereo module
    cv::Mat disparityImg; 
    if (disparityImgPort->getInputCount() > 0)
    {
        ImageOf<PixelBgr> *newDispImg = disparityImgPort->read(false);
        if (newDispImg)
            dImg = newDispImg;

        if (dImg)
        {
            cv::Mat disparityImg_8UC3 = (IplImage*)dImg->getIplImage(); 
            CV_Assert(disparityImg_8UC3.type() == CV_8UC3);

            std::vector<cv::Mat> channels;
            cv::split(disparityImg_8UC3, channels);
            disparityImg = channels[0];

        }
    }

    boost::timer tframe;
    boost::timer tarm;

    cv::Mat imgBBox = img_rgb.clone();

    // draw arm skeleton
    //std::vector<Vector> armJointX;
    //getArmSkeleton(armJointX);
    //for (int i = 1; i < armJointX.size(); i++)
    //{
        //Vector p; gazeCtrl->get2DPixel(camera, armJointX[i], p);
        //Vector p2; gazeCtrl->get2DPixel(camera, armJointX[i-1], p2);
        //cv::circle(imgBBox, cv::Point(p[0], p[1]), 5, cv::Scalar(255,0,255), 3);
        //cv::line(imgBBox, cv::Point(p[0], p[1]), cv::Point(p2[0], p2[1]), cv::Scalar(255,0,255), 5, CV_AA);
    //}

    // get hand pos
    Vector handCoord, handOrient;
    armCart->getPose(handCoord, handOrient);

    // shift hand pos to center of palm
    Vector minBboxCenter = translatePoint3D(handCoord, handOrient, min_bbox_offset_x, min_bbox_offset_y, min_bbox_offset_z);
    Vector maxBboxCenter = translatePoint3D(handCoord, handOrient, max_bbox_offset_x, max_bbox_offset_y, max_bbox_offset_z);

    // calc maximum bbox (used for cropping)
    std::vector<Vector> bboxMax;
    createBoundingBox3D(maxBboxCenter, handOrient, max_bbox_x, max_bbox_y, max_bbox_z, bboxMax);

    // calc minimum bbox (used for training of object model)
    std::vector<Vector> bboxMin;
    createBoundingBox3D(minBboxCenter, handOrient, min_bbox_x, min_bbox_y, min_bbox_z, bboxMin);

    if (display)
    {
        // show min bbox center pos
        Vector ph;
        gazeCtrl->get2DPixel(camera, minBboxCenter, ph);
        cv::circle(imgBBox, cv::Point(ph[0], ph[1]), 5, CV_RGB(0,0,255), 3);
        // show bbox
        drawObjBoundingBox3D(imgBBox, bboxMin, CV_RGB(0,255,0));
        drawObjBoundingBox3D(imgBBox, bboxMax, CV_RGB(255,0,0));
        cv::imshow("obj bbox", imgBBox);
    }

    //std::cout << "tarm: " << tarm.elapsed() << std::endl;
    boost::timer tprep;

    cv::Mat bboxMask    = createBoundingBoxMask(imgBBox, bboxMax);
    cv::Mat bboxMaskMin = createBoundingBoxMask(imgBBox, bboxMin);

    // get 2d rectangle from 3d bounding box
    // image will be cropped using this rectangle
    cv::Rect objRect = getBoundingRect(img_rgb, bboxMax);
    if (objRect == cv::Rect())
    {
        std::cout << "no object" << std::endl;
        return;
    }
    cv::Rect objRect_small = objRect;
    objRect_small.x *= scaleFactor;
    objRect_small.y *= scaleFactor;
    objRect_small.width *= scaleFactor;
    objRect_small.height *= scaleFactor;

    // downsample images
    cv::Mat img;
    cv::Mat bboxMask_small;
    cv::Mat bboxMaskMin_small;
    cv::Mat disparityImg_small; 
    cv::resize(img_rgb, img, cv::Size(), scaleFactor, scaleFactor, cv::INTER_CUBIC);
    cv::resize(bboxMask, bboxMask_small, cv::Size(), scaleFactor, scaleFactor, cv::INTER_CUBIC);
    cv::resize(bboxMaskMin, bboxMaskMin_small, cv::Size(), scaleFactor, scaleFactor, cv::INTER_CUBIC);
    if (!disparityImg.empty()) cv::resize(disparityImg, disparityImg_small, cv::Size(), scaleFactor, scaleFactor, cv::INTER_CUBIC);
    
    //img = img_rgb;
    //bboxMask_small = bboxMask;
    //bboxMaskMin_small = bboxMaskMin;
    //if (!disparityImg.empty()) disparityImg_small = disparityImg;


    // create sub-images
    int trainingBorderSize = 16;
    cv::Rect bkgRect = objRect_small+cv::Size(trainingBorderSize*2, trainingBorderSize*2) 
        - cv::Point(trainingBorderSize, trainingBorderSize);
    bkgRect.x = std::max(0, std::min(img.cols-1, bkgRect.x));
    bkgRect.y = std::max(0, std::min(img.rows-1, bkgRect.y));
    bkgRect.width  = std::max(0, std::min(bkgRect.width, img.cols-bkgRect.x));
    bkgRect.height = std::max(0, std::min(bkgRect.height, img.rows-bkgRect.y));

    // create image showing background region for debugging
    //cv::Mat rectImg = img.clone();
    //cv::rectangle(rectImg, objRect_small.tl(), objRect_small.br(), cv::Scalar::all(255)); 
    //cv::rectangle(rectImg, bkgRect.tl(), bkgRect.br(), cv::Scalar::all(255)); 

    // do preprocessing (blur, color convert) only on subimage
    boost::timer tpre;
    cv::Rect preprocessRect = bkgRect+cv::Size(16,16)-cv::Point(8,8);
    preprocessRect.x = std::max(0, std::min(img.cols-1, preprocessRect.x));
    preprocessRect.y = std::max(0, std::min(img.rows-1, preprocessRect.y));
    preprocessRect.width  = std::max(0, std::min(preprocessRect.width, img.cols-preprocessRect.x));
    preprocessRect.height = std::max(0, std::min(preprocessRect.height, img.rows-preprocessRect.y));
    cv::Mat filterWindow = img(preprocessRect);
    preprocessImage(filterWindow);
    //cv::Mat borderImg = img.clone();
    //cv::rectangle(borderImg, objRect_small, cv::Scalar::all(0), -1);
    //borderImg = borderImg(bkgRect);

    // get cropped images at object location 
    cv::Mat centerImg = img(objRect_small);
    cv::Mat centerDisp = !disparityImg_small.empty() ? disparityImg_small(objRect_small) : cv::Mat();
    cv::Mat centerBBoxMask = bboxMask_small(objRect_small);
    cv::Mat centerBBoxMinMask = bboxMaskMin_small(objRect_small);

    //std::cout << "tprep: " << tprep.elapsed() << std::endl;

    // 
    // Train object and background model
    //

#if 1
    // find training samples in input image
    cv::Mat gmmBkgInput = findBackgroundPixels(centerImg, centerBBoxMask, centerDisp);
    cv::Mat gmmObjInput = findObjectPixels(centerImg, centerBBoxMinMask, centerDisp);

    std::vector<cv::Mat> allSamples_bkg;
    std::vector<cv::Mat> allSamples_obj;
    boost::timer textract;
    //std::cout << "extracting..." << std::endl;
    extractPatches(gmmBkgInput, 1, allSamples_bkg);
    extractPatches(gmmObjInput, 1, allSamples_obj);

    if (allSamples_bkg.size() > maxSamples)
    {
        std::random_shuffle(allSamples_bkg.begin(), allSamples_bkg.end()); 
        allSamples_bkg.resize(maxSamples);
    }
    if (allSamples_obj.size() > maxSamples)
    {
        std::random_shuffle(allSamples_obj.begin(), allSamples_obj.end()); 
        allSamples_obj.resize(maxSamples);
    }

    // train background and object model
    //std::cout << "extract time: " << textract.elapsed() << " " << std::flush;
    std::cout << "nsamples bkg: " << allSamples_bkg.size() << " " << std::flush;
    std::cout << "nsamples obj: " << allSamples_obj.size() << " " << std::flush;
    if (allSamples_bkg.size() < 50 || allSamples_obj.size() < 50)
    {
        std::cout << "Too few samples. Segmentation cancelled." <<std::endl;
        return;
    }
    boost::timer ttrain;
    trainGmm(allSamples_bkg, gmmBkg, kernelSizeGmm_bkg, numClusters_bkg);
    trainGmm(allSamples_obj, gmmObj, kernelSizeGmm_bkg, numClusters_bkg);
    std::cout << "Ttrain: " << ttrain.elapsed() << " " << std::flush;

    boost::timer timer;

    //
    // Apply trained models
    //

    // get probability map
    cv::Mat prob_bkg = createProbMap(gmmBkg, centerImg);
    cv::Mat prob_obj = createProbMap(gmmObj, centerImg);

    // normalize prob mats
    //cv::normalize(prob_bkg, prob_bkg, 1, 0, cv::NORM_MINMAX);
    //cv::normalize(prob_obj, prob_obj, 1, 0, cv::NORM_MINMAX);

    std::cout << " Tpredict: " << timer.elapsed() << " " << std::flush;

    boost::timer tmask;

    // create inverted bounding box masks
    cv::Mat centerBBoxMaskInv(centerBBoxMask.rows, centerBBoxMask.cols, centerBBoxMask.type(), cv::Scalar(255));
    cv::Mat centerBBoxMinMaskInv(centerBBoxMinMask.rows, centerBBoxMinMask.cols, centerBBoxMinMask.type(), cv::Scalar(255));
    centerBBoxMaskInv.setTo(cv::Scalar(0), centerBBoxMask);
    centerBBoxMinMaskInv.setTo(cv::Scalar(0), centerBBoxMinMask);

    // label area outside of max bbox as background
    prob_bkg.setTo(cv::Scalar(1.0), centerBBoxMaskInv);
    prob_obj.setTo(cv::Scalar(0.0), centerBBoxMaskInv);

    // label pixels based on disparity
    //if (false) // has disparity image
    if (!centerDisp.empty()) // has disparity image
    {
        // create binary masks for high and low disparity
        cv::Mat lowDispMask(centerDisp.size(), CV_32F);
        cv::Mat highDispMask(centerDisp.size(), CV_32F);
        for (int i = 0; i < centerDisp.rows; i++)
        {
            for (int j = 0; j < centerDisp.cols; j++)
            {
                int disp = centerDisp.at<uchar>(i,j);
                lowDispMask.at<float>(i,j) = (disp > 1 && disp < lowDisparity) ? 1 : 0;
                highDispMask.at<float>(i,j) = (disp >= highDisparity) ? 1 : 0;
            }
        }
        double w_dispLow  = 0.2;
        double w_dispHigh = 0.4;
        // label low disparity pixels as background
        //cv::erode(lowDispMask,lowDispMask,cv::Mat(),cv::Point(-1,-1), 2);
        //cv::imshow("low disp", lowDispMask);
        //cv::Mat lowdisp;
        //lowDispMask.convertTo(lowdisp, CV_32F);
        //scaleAdd(lowdisp, w_dispLow, prob_bkg, prob_bkg); 
        ////prob_bkg.setTo(cv::Scalar(255), lowDispMask);
        ////prob_obj.setTo(cv::Scalar(0),   lowDispMask);
        
        // label high disparity pixels within min bbox as object
        highDispMask.setTo(cv::Scalar(0), centerBBoxMinMaskInv);
        //prob_bkg.setTo(cv::Scalar(0),   highDispMask);
        //cv::Mat highdisp;
        //highDispMask.convertTo(highdisp, CV_32F);
        scaleAdd(highDispMask, w_dispHigh, prob_obj, prob_obj); 
    }


    // use Graph Cut to merge probability maps into object (1) and background (0) segmentation mask
    boost::timer tgc;

    cv::Mat gcLabels = getGraphCutLabelling(prob_bkg, prob_obj);
    std::cout << "Tgc: " << tgc.elapsed() << " " << std::flush;


    cv::Mat objMask;
    gcLabels.convertTo(objMask, CV_8U);
#endif
    //cv::Mat objMask = centerBBoxMinMask; // don't segment, use bbox
    // remove clutter etc 
    //boost::timer tclutter;
    postprocessImage(objMask);

    // mask object in original (full size) image
    cv::Mat centerImg_big = img_rgb(objRect);
    cv::resize(objMask, objMask, centerImg_big.size());

    //if no object was found return min bounding box
    if (cv::countNonZero(objMask) < minSegmentSize)
        objMask = centerBBoxMinMask;

    // apply created object mask to cropped RGB image
    cv::Mat objImgOut;
    centerImg_big.copyTo(objImgOut, objMask);

    //std::cout << "tmask: " << tmask.elapsed() << std::endl;
    if (display)
    {
        if (!disparityImg.empty()) cv::imshow("disparity image", disparityImg);
        //cv::imshow("graph cut result", gcLabels);
        //cv::imshow("centerImg", centerImg);
        cv::imshow("pr bkg", prob_bkg);
        cv::imshow("prob_obj", prob_obj);
        cv::imshow("object", objImgOut);
        cv::waitKey(5);
    }

    // send result
    if (objImgPort->getOutputCount() > 0)
    {
        sendSegmentedImage(objImgOut);
    }

    std::cout << "Ttotal " << tframe.elapsed() << std::endl;
}


void ObjSegThread::threadRelease() 
{
    objImgPort->interrupt();
    objImgPort->close();
}


void ObjSegThread::sendSegmentedImage(const cv::Mat &img)
{
    if (img.empty())
        return;

    IplImage iplTmp = img;
    ImageOf<PixelBgr> *sendImg = new ImageOf<PixelBgr>;
    sendImg->resize(iplTmp.width, iplTmp.height);
    cvCopyImage(&iplTmp, (IplImage*)sendImg->getIplImage());
    sendMutex.wait();
    objImgPort->prepare() = *sendImg;
    objImgPort->write();
    sendMutex.post();
    delete sendImg;
}


void ObjSegThread::extractPatches(const cv::Mat &img, int patchSize, std::vector<cv::Mat> &patches) const
{
    int step = 2;
    for(int row=0; row<img.rows; row+=step)
    {
        //uchar *ptr=(uchar*) img.ptr(row);
        for(int col=0; col<img.cols; col+=step)
        {
            cv::Mat patch(patchSize, patchSize, img.type());
            std::vector<cv::Mat>::const_reverse_iterator p;
            bool newPatch = true;

            uchar *patch_ptr= (uchar*) patch.ptr(0);
            //uchar *patch_ptr= patch (uchar*) img.ptr(row)[3*col];

            patch_ptr[0] = img.ptr(row)[3*col];
            patch_ptr[1] = img.ptr(row)[3*col+1];
            patch_ptr[2] = img.ptr(row)[3*col+2];

            // skip black pixels
            if (patch_ptr[0] == 0 && 
                    patch_ptr[1] == 0 &&
                    patch_ptr[2] == 0 )
            {
                continue;
            }

            if (newPatch)
            {
                patches.push_back(patch);
            }
        }
    }
}


double ObjSegThread::probCluster(const cv::Mat &sample, const cv::Mat &mu, const cv::Mat &inv_sigma) const
{
    cv::Mat dif = sample.row(0) - mu;
    cv::Mat mult = (dif * inv_sigma) * dif.t();
    double p = std::exp(-0.5 * mult.at<double>(0,0) * 0.1 );
    //std::cout << "p: " << p << std::endl;
    //p /= std::sqrt(std::pow(2.0*3.1415, (double)sample.cols/2.0) * std::abs(det_sigma));
    return p;
}

float ObjSegThread::predict(const ObjSegThread::Gmm &gmm, const cv::Mat &sample, cv::Mat &probs) const
{
    double best_prob = 0;
    int best_k = 0;

    int numClusters = gmm.nClusters();
    cv::Mat means = gmm.getMeans();
    cv::Mat w = gmm.getWeights();
    float* w_ptr = (float*)w.ptr(0);

    probs = cv::Mat(1, numClusters, CV_32F);
    float* prob_ptr = new float[numClusters]; //= (float*)probs.ptr(0);

    //CV_Assert(means.type() == sample.type());

    float p = 0;
    float sumW = 0;
    int i,k;
    Gmm::DTYPE *pdif, *p1, *p2, *pmult;
    Gmm::DTYPE sum, prob_k;

    cv::Mat dif = cv::Mat(sample.size(), sample.type());
    pdif =(Gmm::DTYPE*)(dif.data);
    p1 =(Gmm::DTYPE*)(sample.ptr(0));

    cv::Mat mult(dif.rows, 3, CV_32F);
    pmult =(Gmm::DTYPE*)mult.ptr(0);

    const std::vector<cv::Mat> *invCovs = &gmm.getInvCovs();
    
    int invcovRows = (*invCovs)[0].rows; 
    int invcovCols = (*invCovs)[0].cols; 

    for (k = 0; k < numClusters; k++)
    {
        p2 =(Gmm::DTYPE*)means.ptr(k);
        for (i = 0; i < sample.cols; i++)
            pdif[i] = p1[i] - p2[i];
        matmul(pdif, (float*)(*invCovs)[k].ptr(0), pmult, dif.rows, dif.cols, invcovRows, invcovCols);
        sum = 0;

        for (i = 0; i < dif.cols; i++)
        {
            sum += pmult[i] * pdif[i];
        }
        ////prob_k = std::exp(-0.5 * sum * 0.1 );
        prob_k = std::exp(-0.5 * sum * 0.1 );
        //p =1;
        if (prob_k > best_prob)
        {
            best_k = k;
            best_prob = prob_k;
        }
        p += prob_k * w_ptr[k];
        sumW += w_ptr[k];
    }

    p /= sumW;
    delete prob_ptr;

    return p;
    //return best_k;
}


cv::Mat ObjSegThread::createProbMap(const Gmm &gmm, const cv::Mat &img) const
{
    if (gmm.nClusters() == 0 || img.empty())
        return cv::Mat();

    cv::Mat probMat(img.size(),CV_32F, cv::Scalar::all(0));

    boost::timer timer;

    int kernelSize = gmm.getKernelSize();
    int step = kernelSize;

    int u,v;
    cv::Mat pr;
    int comp;
    double prob;
    cv::Mat sample(1, kernelSize*kernelSize*3, CV_32F);
    for(int row=0; row<img.rows; row+=step)
    {
        uchar *ptr = (uchar*) img.ptr(row);
        float *ptr_probMat = (float*) probMat.ptr(row);
        for(int col=0; col<img.cols; col+=step)
        {
            // get local image patch
            float *sample_ptr= (float*) sample.ptr(0);
            sample_ptr[0] = ptr[3*col+0];
            sample_ptr[1] = ptr[3*col+1];
            sample_ptr[2] = ptr[3*col+2];

            prob = predict(gmm, sample, pr);
            ptr_probMat[col] = std::max(0.0,std::min(prob,255.0));
        }
    }
    return probMat;
}


void ObjSegThread::Gmm::fromCvEM(const CvEM &em, int ksize)
{
    cv::Mat means_64F = em.get_means();
    //if (means_64F.at<double>(0,0) != means_64F.at<double>(0,0))
    //    return;
    means_64F.convertTo(means, CV_32F);

    //std::cout << means_64F << std::endl;

    cv::Mat weights_64F = em.get_weights();
    weights_64F.convertTo(weights, CV_32F);
    //std::cout << weights_64F << std::endl;
    //std::cout << "weight:" << std::endl;
    //std::cout << weights.rows << " " <<  weights.cols << std::endl;
    //std::cout << weights << std::endl;

    kernelSize = ksize;

    std::vector<cv::Mat> covs;
    em.getCovs(covs);
    inv_covs.clear();
    for (int k=0; k < em.get_nclusters(); k++)
    {
        cv::Mat inv;
        cv::Mat cov = covs[k];
        cv::invert(cov, inv, cv::DECOMP_SVD);
        cv::Mat inv_32F;
        inv.convertTo(inv_32F, CV_32F);
        inv_covs.push_back(inv_32F);
    }
}

void ObjSegThread::Gmm::save(const std::string &filename) const
{
    std::cout << "Saving GMM to file " << filename << std::flush;
    // gmm.save(filename.c_str());  // not yet implemented in opencv :(
    
    std::ofstream file(filename.c_str(), std::ios::binary);
    if (! file.is_open())
    {
        std::cerr << "Could not write to file '" << filename << "'" << std::endl;
        return;
    }

    file.write((char*)&kernelSize, sizeof(int));

    int numClusters = inv_covs.size();
    std::cout << " numClusters: " << numClusters << std::endl;
    file.write((char*)&numClusters, sizeof(int));
    // save means 
    int type = means.type();

    file.write((char*)&(means.rows), sizeof(int));
    file.write((char*)&(means.cols), sizeof(int));
    file.write((char*)&(type), sizeof(int));
    file.write((char*)(means.data), means.channels()*means.rows*means.cols*sizeof(DTYPE));

    type = weights.type();
    file.write((char*)&(weights.rows), sizeof(int));
    file.write((char*)&(weights.cols), sizeof(int));
    file.write((char*)&(type), sizeof(int));
    file.write((char*)(weights.data), weights.rows*weights.cols*sizeof(DTYPE));

    // save covariant inverses
    for (int k = 0; k < numClusters ; k++)
    {
        cv::Mat cov = inv_covs[k];
        int type = cov.type();
        file.write((char*)&(cov.rows), sizeof(int));
        file.write((char*)&(cov.cols), sizeof(int));
        file.write((char*)&(type), sizeof(int));
        file.write((char*)(cov.data), cov.channels()*cov.rows*cov.cols*sizeof(DTYPE));
    }

    std::cout << "\t[OK]" << std::endl;
}



void ObjSegThread::Gmm::load(const std::string &filename)
{
    std::cout << "Loading GMM from file " << filename << std::endl;
    std::ifstream file(filename.c_str(), std::ios::binary);
    if (! file.is_open())
    {
        std::cerr << "\nCould not open file '" << filename << "'" << std::endl;
        return;
    }
    
    int rows;
    int cols;
    int type;

    file.read((char*)&kernelSize, sizeof(int));
    std::cout << "\tKernel size: \t" << kernelSize << std::endl;

    int numClusters;
    file.read((char*)&numClusters, sizeof(int));
    std::cout << "\tNum clusters: \t" << numClusters << std::endl;

    std::cout << "\tLoading means... " <<  std::flush;
    // read means
    file.read((char*)&rows, sizeof(int));
    file.read((char*)&cols, sizeof(int));
    file.read((char*)&type, sizeof(int));
    means.create(rows, cols, type);
    file.read((char*)(means.data), means.channels()*
            means.rows*means.cols*sizeof(DTYPE));
    std::cout << "\t[OK]" << std::endl;

    std::cout << "\tLoading weights... " <<  std::flush;
    // read weights
    file.read((char*)&rows, sizeof(int));
    file.read((char*)&cols, sizeof(int));
    file.read((char*)&type, sizeof(int));
    weights.create(rows, cols, type);
    file.read((char*)(weights.data), weights.rows*weights.cols*sizeof(DTYPE));
    std::cout << "\t[OK]" << std::endl;

    std::cout << "\tLoading covariants... " <<  std::flush;
    // read inverted covariants
    inv_covs.clear();
    for (int k = 0; k < numClusters ; k++)
    {
        file.read((char*)&rows, sizeof(int));
        file.read((char*)&cols, sizeof(int));
        file.read((char*)&type, sizeof(int));
        cv::Mat cov(rows, cols, type);
        file.read((char*)(cov.data), cov.channels()*cov.rows*cov.cols*sizeof(DTYPE));
        inv_covs.push_back(cov);
    }
    std::cout << "\t[OK]" << std::endl;
}



cv::Mat ObjSegThread::removeBorder(const cv::Mat &img, const cv::Vec3b &color)
{   
    CV_Assert(img.type() == CV_8UC3);


    int top = 0;
    int bottom = 0;
    int left = 0;
    int right = 0;

    for (int y = 0; y < img.rows; y++)
    {
        for (int x = 0; x < img.cols; x++)
        {
            if (img.at<cv::Vec3b>(y,x) != color)
            {
                top = y;
                goto BOTTOM_;
            }
        }
    }

BOTTOM_:
    for (int y = img.rows-1; y >= 0; y--)
    {
        for (int x = 0; x < img.cols; x++)
        {
            if (img.at<cv::Vec3b>(y,x) != color)
            {
                bottom = y;
                goto LEFT_;
            }
        }
    }

LEFT_:
    for (int x = 0; x < img.cols; x++)
    {
        for (int y = 0; y < img.rows; y++)
        {
            if (img.at<cv::Vec3b>(y,x) != color)
            {
                left = x;
                goto RIGHT_;
            }
        }
    }

RIGHT_:
    for (int x = img.cols-1; x >= 0 ; x--)
    {
        for (int y = 0; y < img.rows; y++)
        {
            if (img.at<cv::Vec3b>(y,x) != color)
            {
                right = x;
                goto DONE_;
            }
        }
    }

DONE_:

    int w = right-left;
    int h = bottom - top;
    if (w*h == 0)
        return img;

    return cv::Mat(img, cv::Rect(left,top, w, h));

}

void ObjSegThread::getArmSkeleton(std::vector<Vector> &jointCoords)
{
    jointCoords.clear();
    jointCoords.push_back(getArmJointPosition(2)); // shoulder
    jointCoords.push_back(getArmJointPosition(3)); // elbow
    jointCoords.push_back(getArmJointPosition(4)); // wrist
    jointCoords.push_back(getArmJointPosition(7)); // end eff
}

Vector ObjSegThread::getArmJointPosition(int jnt) const
{
    iCub::iKin::iCubArm libArm("right");         
    iCub::iKin::iKinChain *chain = libArm.asChain();     

    chain->releaseLink(0);
    chain->releaseLink(1);
    chain->releaseLink(2);

    int nAxesTorso;
    encTorso->getAxes(&nAxesTorso);
    Vector torsoQ(nAxesTorso);
    encTorso->getEncoders(torsoQ.data());

    int nAxesArm;
    encArm->getAxes(&nAxesArm);
    Vector armQ(nAxesArm);
    encArm->getEncoders(armQ.data());

    Vector ang(3+jnt);
    for (int i = 0; i < torsoQ.size(); i++)
        ang[i] = torsoQ[i]*M_PI/180;
    for (int i = 0; i < jnt; i++)
        ang[i+torsoQ.size()] = armQ[i]*M_PI/180;

    chain->setAng(ang);
    Vector pose = chain->EndEffPose();
    return pose.subVector(0,2);
}

cv::Rect ObjSegThread::getObjRect(const cv::Mat &src, double handDist) const
{
    //std::cout << "handDist: " << handDist << std::endl;
    double maxDist = 0.35;
    double minDist = 0.15;

    int minW = 100;  
    int maxW = 240;  
    int dW = maxW - minW;

    double s = (handDist-minDist) / (maxDist-minDist);
    int scaledWidth = minW + (1-s)*dW;
    scaledWidth = std::max(minW, std::min(maxW, scaledWidth));

    int width = scaledWidth;

    int cx = src.cols / 2;
    int cy = src.rows / 2;
    
    cv::Rect objRect(cx-width/2, cy-width/2, width, width); 

    objRect.x = std::max(0, objRect.x);
    objRect.y = std::max(0, objRect.y);
    objRect.width  = std::min(objRect.width, src.cols-objRect.x);
    objRect.height = std::min(objRect.height, src.rows-objRect.y);

    return objRect;
}


void ObjSegThread::preprocessImage(cv::Mat &img) const
{
    // blur the image
    for (int i = 0; i < blurLevel; i++)
    {
        cv::blur(img, img, cv::Size(3,3));
    }

    // convert color
    switch(mod)
    {
        case MODE_RGB:
            break;

        case MODE_YUV:
            //cvCvtColor(preprocessROI,preprocessROI,CV_RGB2YUV);
            break;

        case MODE_HSV:
            cv::cvtColor(img, img, CV_RGB2HSV);
            break;

        case MODE_LAB:
            cv::cvtColor(img, img ,CV_BGR2Lab);
            break;
    }
}
void ObjSegThread::createBoundingBox3D(const Vector &handX, const Vector &handO, 
        double bbox_x, double bbox_y, double bbox_z, std::vector<Vector> &bbox) const
{
    double ox = bbox_x/2.0;
    double oy = bbox_y/2.0;
    double oz = bbox_z/2.0;

    Vector topLeftL     = translatePoint3D(handX, handO, ox, -oy, 0);
    Vector topLeftH     = translatePoint3D(handX, handO, ox, -oy, oz);
    Vector topRightL    = translatePoint3D(handX, handO, ox, oy, 0);
    Vector topRightH    = translatePoint3D(handX, handO, ox, oy, oz);
    Vector bottomRightL = translatePoint3D(handX, handO, -ox, oy, 0);
    Vector bottomRightH = translatePoint3D(handX, handO, -ox, oy, oz);
    Vector bottomLeftL  = translatePoint3D(handX, handO, -ox, -oy, 0);
    Vector bottomLeftH  = translatePoint3D(handX, handO, -ox, -oy, oz);

    bbox.clear();
    bbox.push_back(topLeftL);
    bbox.push_back(topRightL);
    bbox.push_back(bottomRightL);
    bbox.push_back(bottomLeftL);
    bbox.push_back(topLeftH);
    bbox.push_back(topRightH);
    bbox.push_back(bottomRightH);
    bbox.push_back(bottomLeftH);
}

cv::Mat ObjSegThread::createBoundingBoxMask(const cv::Mat &img, const std::vector<Vector> &bbox3D) const
{
    cv::vector<cv::vector<cv::Point> > contours;
    cv::vector<cv::Point> hull;

    calcConvexHull2D(bbox3D, hull);
    contours.push_back(hull);

    cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8U);
    cv::drawContours(mask, contours, 0, cv::Scalar(255), CV_FILLED);
    
    return mask;
}


cv::Rect ObjSegThread::getBoundingRect(const cv::Mat &img, const std::vector<Vector> &bbox3D) const
{
    cv::vector<cv::Point> hull;
    calcConvexHull2D(bbox3D, hull);

    // get dimensions of hull
    int maxX = 0, maxY = 0;
    int minX = img.rows-1, minY = img.cols-1;
    for (int i = 0; i < hull.size(); i++)
    {
        cv::Point p = hull[i];
        minX = std::min(minX, p.x);
        minY = std::min(minY, p.y);
        maxX = std::max(maxX, p.x);
        maxY = std::max(maxY, p.y);
    }

    minX = std::max(minX, 0);
    minY = std::max(minY, 0);
    maxX = std::min(maxX, img.cols-1);
    maxY = std::min(maxY, img.rows-1);
    if (minX >= img.cols || minY >= img.rows || maxX < 0 || maxY < 0)
        return cv::Rect();
    return cv::Rect(minX, minY, maxX-minX, maxY-minY); // x,y,w,h
}

void ObjSegThread::calcConvexHull2D(const std::vector<Vector> &bbox3D, cv::vector<cv::Point> &hull) const
{
    CV_Assert(bbox3D.size() == 8);
    std::vector<cv::Point> contour;
    for (int i =0; i < bbox3D.size(); i++)
    {
        Vector p2d;
        // get 2D u/v coordinates in image
        gazeCtrl->get2DPixel(camera, bbox3D[i], p2d); 
        contour.push_back(cv::Point(p2d[0], p2d[1]));
    }
    cv::convexHull(contour, hull, true, true);
}

void ObjSegThread::drawObjBoundingBox3D(cv::Mat &img, const std::vector<Vector> &bbox, const cv::Scalar &color) const
{
    CV_Assert(gazeCtrl);

    // get 2D u/v coordinates in image
    std::vector<Vector> bbox2D;
    for (int i =0; i < bbox.size(); i++)
    {
        Vector p2d;
        gazeCtrl->get2DPixel(camera, bbox[i], p2d); 
        bbox2D.push_back(p2d); 
    }

    int wfThickness = 1;

    CV_Assert(bbox2D.size() == 8);

    cv::Scalar colorVectices = CV_RGB(0,0,255);

    // draw wireframe 
    for (int i = 1; i < 4; i++)
    {
        cv::Point ptStart(bbox2D[i-1][0], bbox2D[i-1][1]);
        cv::Point ptEnd(bbox2D[i][0], bbox2D[i][1]);
        cv::line(img, ptStart, ptEnd, color, wfThickness, CV_AA);
    }
    cv::Point ptStart(bbox2D[3][0], bbox2D[3][1]);
    cv::Point ptEnd(bbox2D[0][0], bbox2D[0][1]);
    cv::line(img, ptStart, ptEnd, color, wfThickness, CV_AA);
    for (int i = 5; i < 8; i++)
    {
        cv::Point ptStart(bbox2D[i-1][0], bbox2D[i-1][1]);
        cv::Point ptEnd(bbox2D[i][0], bbox2D[i][1]);
        cv::line(img, ptStart, ptEnd, color, wfThickness, CV_AA);
    }
    cv::line(img, 
            cv::Point(bbox2D[7][0], bbox2D[7][1]), 
            cv::Point(bbox2D[4][0], bbox2D[4][1]), 
            color, wfThickness, CV_AA);
    for (int i = 0; i < 4; i++)
    {
        cv::line(img, 
                cv::Point(bbox2D[i][0], bbox2D[i][1]), 
                cv::Point(bbox2D[i+4][0], bbox2D[i+4][1]), 
                color, wfThickness, CV_AA);
    }

    // draw vertices
    for (int i = 0; i < 8; i++)
    {
        cv::Point pos(bbox2D[i][0], bbox2D[i][1]);
        cv::circle(img, pos, 4, colorVectices, -1);
    }
}


cv::Mat ObjSegThread::getGraphCutLabelling(const cv::Mat &prob, const cv::Mat &prob_obj) const
{
    int nNodes = prob.cols*prob.rows;

    GraphType *g = new GraphType(nNodes, nNodes*8); 

    int idx = 0;
    for (int i = 0; i < prob.rows; i++)
    {
        for (int j = 0; j < prob.cols; j++)
        {
            g->add_node(); 

            // set node capacities
            double p_bkg = w_bkg*prob.at<float>(i,j);
            double p_obj = w_obj*prob_obj.at<float>(i,j);
            g->add_tweights(idx, p_bkg, p_obj);

            // link node to neighbor pixels (8x)
            for (int nby = i-1; nby <= i+1; nby++)
            {
                for (int nbx = j-1; nbx <= j+1; nbx++)
                {
                   int x = std::min(prob.cols-1, std::max(nbx, 0));
                   int y = std::min(prob.rows-1, std::max(nby, 0));
                   int nb_idx = y*prob.cols+x;
                   if (nb_idx == idx) continue;
                   g->add_edge(idx, nb_idx, w_edge, w_edge);
                }
            }
            idx++;
        }
    }

    // compute...
    g->maxflow();

    // create image containing segmentation binary mask
    idx = 0;
    cv::Mat seg(prob.rows, prob.cols, CV_8U, cv::Scalar(0));
    for (int i = 0; i < prob.rows; i++)
    {
        for (int j = 0; j < prob.cols; j++)
        {
            seg.at<uchar>(i,j) = g->what_segment(idx, GraphType::SINK) == GraphType::SINK ? 255 : 0;
            idx++;
        }
    }

    delete g;

    return seg;
}

void ObjSegThread::postprocessImage(cv::Mat &objMask) const
{
    // get rid of some clutter
    //cv::erode(objMask,objMask,cv::Mat(),cv::Point(-1,-1), 1);
    //cv::dilate(objMask,objMask,cv::Mat(),cv::Point(-1,-1), 1);
    //cv::erode(objMask,objMask,cv::Mat(),cv::Point(-1,-1), 2);

    //cv::blur(objMask, objMask, cv::Size(3,3));
    return;
    //cv::imshow("dilate", objMask);

    // remove artefacts
    int maxSegmentId = 0;
    int maxSegment = 0;
    std::vector<cv::Point> objectPoints;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(objMask, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE); 
    objMask.setTo(cv::Scalar(0));
    for(int idx = 0; idx < contours.size(); idx++)
    {
        int segmentSize = cv::contourArea(contours[idx]);
        if (segmentSize > maxSegment)
        {
            maxSegmentId = idx; 
            maxSegment = segmentSize; 
        }
        //if (segmentSize > minSegmentSize) 
        //{
            //cv::drawContours(objMask, contours, idx, cv::Scalar::all(0), CV_FILLED, 8);
        //}
        //else // remove segment
        //{
            //objectPoints.insert(objectPoints.end(), contours[idx].begin(), contours[idx].end());
        //}
    } 
    cv::drawContours(objMask, contours, maxSegmentId, cv::Scalar::all(255), CV_FILLED, 8);

    // get rid of some clutter
    //cv::erode(objMask,objMask,cv::Mat(),cv::Point(-1,-1), 1);
    cv::dilate(objMask,objMask,cv::Mat(),cv::Point(-1,-1), 4);
    //cv::erode(objMask,objMask,cv::Mat(),cv::Point(-1,-1), 1);

    //cv::blur(objMask, objMask, cv::Size(3,3));
    //cv::blur(objMask, objMask, cv::Size(3,3));
}


cv::Mat ObjSegThread::findBackgroundPixels(const cv::Mat &img, const cv::Mat &bboxMask, const cv::Mat &disparity) const
{
    CV_Assert(bboxMask.type() == CV_8U);

    cv::Mat bkgMask = cv::Mat::zeros(bboxMask.size(), bboxMask.type()); 
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            bool isInBBox       = bboxMask.at<uchar>(i,j) > 0;
            bool isLowDisparity = false;
            bool isBgr = !isInBBox;
            //if (!disparity.empty())
            //{
                //int disp = disparity.at<uchar>(i,j);
                //isLowDisparity = (disp > 0) && (disp < highDisparity);
                //isBgr = isLowDisparity || !isInBBox;
            //}
            bkgMask.at<uchar>(i,j) = isBgr ? 255 : 0;
        }
    }
    cv::Mat background = cv::Mat::zeros(img.size(), img.type());
    img.copyTo(background, bkgMask);
    return background;
}

cv::Mat ObjSegThread::findObjectPixels(const cv::Mat &img, const cv::Mat &bboxMask, const cv::Mat &disparity) const
{
    CV_Assert(bboxMask.type() == CV_8U);

    cv::Mat objMask = cv::Mat::zeros(bboxMask.size(), bboxMask.type()); 
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            bool isInBBox           = bboxMask.at<uchar>(i,j) > 0;
            bool isObj              = isInBBox;
            // if we have disparity information, it may not be a low disp value
            if (!disparity.empty()) 
            {
                bool isHighDisparity    = false;
                bool isLowDisparity     = false;
                int disp = disparity.at<uchar>(i,j);
                //isHighDisparity = disp > highDisparity;
                isLowDisparity = disp > 0 && disp < lowDisparity;
                isObj = isObj && !isLowDisparity;
            }
            objMask.at<uchar>(i,j) = isObj ? 255 : 0;
        }
    }
    cv::Mat object = cv::Mat::zeros(img.size(), img.type());
    img.copyTo(object, objMask);
    return object;
}

void ObjSegThread::trainGmm(const std::vector<cv::Mat> &samples, Gmm &currentGmm, int kernelSize, int numClusters) const
{
    if (samples.empty())
    {
        std::cout << "No samples for GMM training. Training cancelled." << std::endl;
        return;
    }

    if (samples.size() <= numClusters)
    {
        std::cout << "The number of samples for GMM training must be greater than the number of clusters. Training cancelled." << std::endl;
        return;
    }

    //std::cout << "creating input data for GMM training..." << std::endl;
    //std::cout << "num samples: " << samples.size() << std::endl;
    int sampleDims = samples.front().rows *samples.front().rows * samples.front().channels();
    //std::cout << "sample dims: " << sampleDims << std::endl;
    cv::Mat samples_8U(samples.size(), sampleDims, CV_8U);
    int cnt=0;
    std::vector<cv::Mat>::const_iterator p; 
    for (p = samples.begin(); p != samples.end(); ++p)
    {
        for (int i=0;i < samples_8U.cols;i++)
            samples_8U.row(cnt).data[i] = p->data[i];
        cnt++;
    }
    cv::Mat samples_32F;
    samples_8U.convertTo(samples_32F, CV_32F);

    // train gaussian mixture model
    //std::cout << "training GMM (k=" << numClusters << ")... " << std::flush;
    //std::cout << samples_32F << std::endl;
    static bool firsttime = true;
    int dims = 3;
    CvEMParams params;
    params.nclusters = numClusters;
    params.means = 0;
    params.covs = 0;
    params.weights = 0;
    params.term_crit.max_iter = 20;
    params.term_crit.epsilon  = 0.1;
    params.term_crit.type     = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;
    params.cov_mat_type       = CvEM::COV_MAT_SPHERICAL;
    CvEM em;
    em.train(samples_32F, cv::Mat(), params);
    currentGmm.fromCvEM(em, kernelSize);
}
