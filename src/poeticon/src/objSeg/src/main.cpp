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
 * @file main.cpp
 * @brief main code for your module. This shows up in the list of files.
 */

#include "iCub/ObjSegModule.h" 

using namespace yarp::os;
using namespace yarp::sig;

#if 0

#include <cv.h>
#include <highgui.h>

cv::Mat DrawHistogram(const cv::Mat &hist, float scaleX=1, float scaleY=1)
{
  double histMax = 0;
  cv::minMaxLoc(hist, 0, &histMax, 0, 0);


  scaleX = 3;
     IplImage* imgHist = cvCreateImage(cvSize(256*scaleX, 64*scaleY), 8 ,1);
    cvZero(imgHist);

    for(int i=0;i<hist.cols-1;i++)
    {
        float histValue = hist.at<float>(0,i);
        float nextValue = hist.at<float>(0,i+1);
 
        CvPoint pt1 = cvPoint(256.0*i/(double)hist.cols*scaleX, 64*scaleY);
        CvPoint pt2 = cvPoint(256.0*i/(double)hist.cols*scaleX+256.0/(double)hist.cols*scaleX, 64*scaleY);
        CvPoint pt3 = cvPoint(256.0*i/(double)hist.cols*scaleX+256.0/(double)hist.cols*scaleX, (64-nextValue*64/histMax)*scaleY);
        CvPoint pt4 = cvPoint(256.0*i/(double)hist.cols*scaleX, (64-histValue*64/histMax)*scaleY);
 
        int numPts = 5;
        CvPoint pts[] = {pt1, pt2, pt3, pt4, pt1};
 
        cvFillConvexPoly(imgHist, pts, numPts, cvScalar(255));
    }
    cv::Mat img = imgHist;
    return img;
}
cv::Mat extractColor1D(const cv::Mat &src)
{
    bool display = true;
    int histSize[] = {100};
    float a_ranges[] = { 0, 256 };
    float b_ranges[] = { 0, 256 };
    //float a_ranges[] = { 0, 180 };
    //float b_ranges[] = { 0, 256 };
    const float* ranges[] = {a_ranges, b_ranges};
      
    // we compute the histogram from channels 1 and 2 (a and b)
    int channelA[] = {1};
    int channelB[] = {2};

    // prepare image
    cv::Mat lab;
    cv::cvtColor(src, lab, CV_BGR2Lab);
    //lab = src;
    std::vector<cv::Mat> colorPlanes;
    cv::split(lab, colorPlanes);
  
    // create histogram from a* and b* planes of Lab color space
    cv::Mat histA;
    cv::Mat histB;
    cv::Mat hist(histSize[0]*2, 1, CV_32F, cv::Scalar(0));
    // use source image as mask -> ignore black pixels
    cv::Mat mask;
    cv::Mat ones = cv::Mat::ones(src.size(), CV_8U);
    ones.copyTo(mask, colorPlanes[0]);
    // display color planes
    cv::imshow("L", colorPlanes[0]);
    cv::imshow("A", colorPlanes[1]);
    cv::imshow("B", colorPlanes[2]);
    //std::cout << colorPlanes[1] <<std::endl;
    cv::imshow("Mask", mask*255);
    cv::calcHist(&lab, 1, channelA, mask, 
        histA, 1, histSize, ranges, true); 
    cv::calcHist(&lab, 1, channelB, mask, 
        histB, 1, histSize, ranges, true); 

    // concatenate histograms
    cv::Mat dstA = hist.rowRange(0, histA.rows);
    cv::Mat dstB = hist.rowRange(histA.rows, hist.rows);
    histA.copyTo(dstA);
    histB.copyTo(dstB);

    hist = hist.t();

    int nonzero = cv::countNonZero(colorPlanes[0]);
    if (nonzero == 0)
	{
		nonzero = 1;
	}
    hist = hist * 5  / nonzero ;//* 100.0 / (nonzero);

    //std::cout << hist << std::endl;
    //std::cout << nonzero << std::endl;

    // smooth histogram 
    cv::blur(hist, hist, cv::Size(3,3));
    //cv::blur(hist, hist, cv::Size(3,3));

    // display 2d-histogram
    if (display)
    {
        cv::Mat histImg;
        cv::resize(hist, histImg, cv::Size(), 5, 10, cv::INTER_NEAREST);
        //cv::imshow("colorhist", histImg/cv::norm(histImg, cv::NORM_INF));
        return histImg;///cv::norm(histImg, cv::NORM_INF);
    }

    // return as 1d-vector
    return hist.reshape(0, 1);
}
cv::Mat extractColor(const cv::Mat &src)
{
    bool display = true;
    int histSize[] = {25, 25};
    float a_ranges[] = { 0, 255 };
    float b_ranges[] = { 0, 255 };
    //float a_ranges[] = { 50, 200 };
    //float b_ranges[] = { 50, 200 };
    const float* ranges[] = {a_ranges, b_ranges};
      
    // we compute the histogram from channels 1 and 2 (a and b)
    int channels[] = {0, 2};

    // prepare image
    cv::Mat lab;
    cv::cvtColor(src, lab, CV_BGR2Lab);
    //lab = src;
    std::vector<cv::Mat> colorPlanes;
    cv::split(lab, colorPlanes);
  
    // create histogram from a* and b* planes of Lab color space
    cv::MatND histND(2, histSize, CV_32F, cv::Scalar(0));
    // use source image as mask -> ignore black pixels
    cv::Mat mask;
    cv::Mat ones = cv::Mat::ones(src.size(), CV_8U);
    ones.copyTo(mask, colorPlanes[0]);
    // display color planes
    cv::imshow("L", colorPlanes[0]);
    cv::imshow("A", colorPlanes[1]);
    cv::imshow("B", colorPlanes[2]);
    //cv::imshow("mask", mask*255);
    //cv::waitKey();
    cv::calcHist(&lab, 1, channels, mask, 
        histND, 2, histSize, ranges, true); 

    int nonzero = cv::countNonZero(colorPlanes[0]);
    if (nonzero == 0)
	{
		nonzero = 1;
	}
    cv::Mat hist = histND *10 / nonzero ;//* 100.0 / (nonzero);

    std::cout << hist << std::endl;
    std::cout << nonzero << std::endl;

    // smooth histogram 
    cv::blur(hist, hist, cv::Size(3,3));
    //cv::blur(hist, hist, cv::Size(3,3));

    // display 2d-histogram
    if (display)
    {
        cv::Mat histImg;
        cv::resize(hist, histImg, cv::Size(), 10, 10, cv::INTER_NEAREST);
        //cv::imshow("colorhist", histImg/cv::norm(histImg, cv::NORM_INF));
        return histImg/cv::norm(histImg, cv::NORM_INF);
    }

    // return as 1d-vector
    return hist.reshape(0, 1);
}

int main(int argc, char * argv[]) {


    cv::Mat img1 = cv::imread("../activeObjRec/data/objectviews/redcupdisp/keyframe_028.png");
    //cv::Mat img1 = cv::imread("../activeObjRec/data/objectviews/noseg.png");
    //cv::Mat img1 = cv::imread("../activeObjRec/data/objectviews/green.png");
    //cv::Mat img1 = cv::imread("../activeObjRec/data/objectviews/keyframe_blue.png");
    cv::Mat img2 = cv::imread("../activeObjRec/data/objectviews/normcupdisp4/keyframe_045.png");
    //cv::Mat img3 = cv::imread("../activeObjRec/data/objectviews/redcupdisp/keyframe_012.png");
    cv::Mat img3 = cv::imread("../activeObjRec/data/objectviews/normcupdisp4/keyframe_071.png");
    cv::Mat img4 = cv::imread("../activeObjRec/data/objectviews/normcupdisp4/keyframe_072.png");

    cv::Mat h2 = extractColor1D(img2);
    cv::Mat h3 = extractColor1D(img3);
    std::cout << "img1" << std::endl;
    cv::Mat h1 = extractColor1D(img1);
    cv::Mat h4 = extractColor1D(img4);
    cv::imshow("img 1", img1);
    cv::imshow("img 2", img2);
    cv::imshow("img 3", img3);
    cv::imshow("img 4", img4);
    cv::imshow("hst red", DrawHistogram(h1));
    cv::imshow("hst norm", DrawHistogram(h2));
    cv::imshow("hst rd side", DrawHistogram(h3));
    cv::imshow("hst n side", DrawHistogram(h4));
    cv::waitKey(0);
    return 1;

#endif

int main(int argc, char * argv[]) {

   /* initialize yarp network */ 
   Network yarp;
   /* create your module */
   ObjSegModule module; 

   /* prepare and configure the resource finder */
   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("objSegSim.ini");                        //overridden by --from parameter
   rf.setDefaultContext("../contrib/src/poeticon/src/objSeg/conf"); //overridden by --context parameter
   rf.configure("ICUB_ROOT", argc, argv);
 
   /* run the module: runModule() calls configure first and, if successful, it then runs */
   module.runModule(rf);

   return 0;
}

