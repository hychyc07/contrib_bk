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
 * @file FeatureExtractor.h
 * @brief implementation of the FeatureExtractor methods.
 */

#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>

#include <iCub/FeatureExtractor.h>
#include <iCub/Util.h>

#define COLOR_HIST_1D 1

FeatureExtractor::FeatureType FeatureExtractor::featureNameToType(const std::string &name)
{
    if (boost::iequals(name, "fourier"))
        return FOURIER; 
    else if (boost::iequals(name, "phog"))
        return PHOG;
    else if (boost::iequals(name, "surf"))
        return SURF;
    else if (boost::iequals(name, "composite"))
        return COMPOSITE;
    else if (boost::iequals(name, "color"))
        return COLOR;
    else
        return UNKNOWN_FEATURE;
}

cv::Mat FeatureExtractor::extract(const cv::Mat &src, FeatureType type)
{
    
    switch (type)
    {
        case PHOG:
            return extractPhog(src);
            break;
        case FOURIER:
            return extractFourier(src);
            break;
        case SURF:
            return extractSurf(src);
            break;
        case COMPOSITE:
            return extractComposite(src);
            break;
        case COLOR:
            return extractColor(src);
            break;
        default:
            CV_Assert(false);
    }
}

void showScaled(const std::string &wndName, const cv::Mat& img)
{
    double minVal, maxVal;
    cv::minMaxLoc(img, &minVal, &maxVal);
    cv::Mat subMat(img.size(), img.type(), cv::Scalar(minVal));
    cv::imshow(wndName, cv::Mat(img-subMat)/(maxVal-minVal));
}
void showFirstChannel(const std::string &wndName, const cv::Mat& img)
{
    std::vector<cv::Mat> channels;
    cv::split(img, channels);
    showScaled(wndName, channels[0]);
}

cv::Mat FeatureExtractor::transformImage(const cv::Mat &srcImg) 
{
    cv::Mat srcImgReal(srcImg.size(), CV_32F);
    srcImg.convertTo(srcImgReal, CV_32F);

    std::vector<cv::Mat> inImgs;
    inImgs.push_back(srcImgReal);
    inImgs.push_back(cv::Mat(srcImgReal.size(), CV_32F, cv::Scalar(0)));
    cv::Mat dftInput;
    cv::merge(inImgs, dftInput);
    
    cv::Mat dftOutput;
    cv::dft(dftInput, dftOutput, cv::DFT_REAL_OUTPUT);

    return dftOutput;
}


cv::Mat FeatureExtractor::extractFromSingleChannel(const cv::Mat &img) 
{
    cv::Mat dftOutput = transformImage(img);
    
    // shift image
    cv::Mat dftOutputShifted(dftOutput.size(), dftOutput.type());
    for (int y=0; y < dftOutput.rows; y++)
    {
        for (int x=0; x< dftOutput.cols; x++)
        {
            int yShift = (y+dftOutput.rows/2) % dftOutput.rows;
            int xShift = (x+dftOutput.cols/2)  % dftOutput.cols;
            dftOutputShifted.at<cv::Vec2f>(yShift, xShift) = dftOutput.at<cv::Vec2f>(y, x);
        }
    }

    // select subwindow at center
    int WND_WIDTH = 32;
    int WND_HEIGHT = 32;
    cv::Rect selectionSrc(dftOutput.cols/2 - WND_HEIGHT/2, 
                          dftOutput.rows/2 - WND_HEIGHT/2, 
                          WND_WIDTH, 
                          WND_HEIGHT);
    cv::Mat dftWindow(dftOutputShifted, selectionSrc);

    // compute magnitudes of subwindow`
    std::vector<cv::Mat> channels;
    cv::split(dftWindow, channels);
    cv::Mat fft_re = channels[0];
    cv::Mat fft_im = channels[1];
    cv::Mat magnitude;
    cv::Mat angle;
    cv::cartToPolar(fft_re, fft_im, magnitude, angle);

    magnitude += 0.001;  // avoid log(0) -> inf
    cv::log(magnitude, magnitude);

    return magnitude;
}


cv::Mat FeatureExtractor::extractFourier(const cv::Mat &src) 
{
    cv::Mat lab;
    cv::cvtColor(src, lab, CV_BGR2Lab);

    std::vector<cv::Mat> colorChannels;
    cv::split(lab, colorChannels);

    std::vector<cv::Mat> resMats;
    for (int c = 0; c < 3; ++c)
    {
        cv::Mat m = extractFromSingleChannel(colorChannels[c]);
        resMats.push_back(m.clone());
    }

    cv::Mat feature;
    cv::merge(resMats, feature);

    return feature.reshape(1,1);
}


// returns similarity in the range (0,1]
// 1 denotes maximum similarity
double FeatureExtractor::compareFeatures(const cv::Mat &ft1, const cv::Mat &ft2)
{
    double d = cv::norm(ft1, ft2, cv::NORM_L1);
    double sim = exp(-Util::sqr(d/(ft1.cols*2)));
    //std::cout << sim << std::endl;
    return sim;
}


cv::Mat FeatureExtractor::extractPhog(const cv::Mat &src) 
{
    bool display = false;

    // TODO: move to config
    int m_nLevels = 2;
    int m_nBins = 20;
    double m_cannyLowThreshold = 85.0;
    double m_cannyHighThreshold = 110.0;
    int m_maxAngle = 360;

    // convert image to gray scale
    cv::Mat gray_8U(src.size(), CV_8U);
    cv::cvtColor(src, gray_8U, CV_BGR2GRAY);

    // perform canny edge detection
    cv::Mat edges_8U(gray_8U.size(), CV_8U);
    cv::Canny(gray_8U, edges_8U, m_cannyLowThreshold, m_cannyHighThreshold);

    if (display)
    {
        cv::imshow("Edge image", edges_8U);
    }

    // calculate gradients
    cv::Mat gradientX_32F(gray_8U.size(), CV_32F);
    cv::Mat gradientY_32F(gray_8U.size(), CV_32F);
    cv::Sobel(gray_8U, gradientX_32F, gradientX_32F.type(), 1, 0, 3);
    cv::Sobel(gray_8U, gradientY_32F, gradientY_32F.type(), 0, 1, 3);
    if (display)
    {
        cv::imshow("Gradients X", gradientX_32F/255.0);
        cv::imshow("Gradients Y", gradientY_32F/255.0);
    }

    // calculate gradient angles and magnitudes
    cv::Mat magnitude_32F(gray_8U.size(), CV_32F, cv::Scalar(0.0));
    cv::Mat angles_32F(gradientX_32F.size(),CV_32F);
    bool anglesInDegrees = true;
    cv::cartToPolar(gradientX_32F, gradientY_32F, magnitude_32F, angles_32F, anglesInDegrees);

    /*double minval, maxval;
    cv::minMaxLoc(gradientXY_32F, &minval, &maxval);
    std::cout << minval << std::endl;
    std::cout << maxval << std::endl;
    cv::minMaxLoc(gradientXY, &minval, &maxval);
    std::cout << minval << std::endl;
    std::cout << maxval << std::endl;*/

    if (display)
    {
        cv::imshow("Gradient Magnitudes", magnitude_32F/255.0);
        cv::imshow("Gradient Angles", angles_32F/(360));
    }

    // create matrix containing histogram values for each pixel    
    cv::Mat edgeAngles_32F(angles_32F.size(), CV_32F, cv::Scalar(-1));
    angles_32F.copyTo(edgeAngles_32F, edges_8U);
    float degreesPerBin = static_cast<float>(m_maxAngle) / m_nBins;
    cv::Mat bins_32F = (edgeAngles_32F / degreesPerBin);
    
    // convert to int
    cv::Mat bins_32S(angles_32F.size(), CV_32S);
    bins_32F -= cv::Mat(edgeAngles_32F.size(), CV_32F, cv::Scalar(0.5));
    bins_32F.convertTo(bins_32S, CV_32S);

    if (display)
    {
        cv::imshow("Bin Matrix", bins_32S);
    }

    // create histograms for all pyramid levels
    int nCells = 0;
    for (int l = 0; l < m_nLevels; ++l)
    {
        nCells += (int)pow((float)4, (float)l);
    }
    cv::Mat descriptor(1, m_nBins*nCells, CV_32F, cv::Scalar(0.0));
    int bin;
    int count = 0;
    int cellNo = 0;
    for (int level = 0; level < m_nLevels; ++level)
    {
        int cellWidth = (int)(bins_32S.cols / pow((float)2, (float)level));
        int cellHeight = (int)(bins_32S.rows / pow((float)2, (float)level));
        int cellLeft = 0;

        while (cellLeft + cellWidth <= bins_32S.cols)
        {       
            int cellTop = 0;
            while (cellTop + cellHeight <= bins_32S.rows)
            {
                for (int i = cellTop; i < cellTop+cellHeight; ++i)
                {
                    for (int j = cellLeft; j < cellLeft+cellWidth; ++j)
                    {
                        bin = bins_32S.at<int>(i, j);
                        if (bin > 0)
                        {
                            descriptor.at<float>(0, bin+cellNo*m_nBins) += magnitude_32F.at<float>(i, j);
                            count++;
                        }
                    }
                }
                cellTop += cellHeight;
                cellNo++;
            }
            cellLeft += cellWidth;
        }
    }

    // normalize
    if (count > 0)
        descriptor /= count;

    if (display)
    {
        cv::Mat descriptorResized;
        cv::resize(descriptor, descriptorResized, cv::Size(), 3, 50, cv::INTER_NEAREST);
        cv::imshow("descriptor", descriptorResized / cv::norm(descriptorResized, cv::NORM_INF));
        cv::waitKey(10);
    }


    return descriptor;
}

cv::Mat FeatureExtractor::extractComposite(const cv::Mat &src)
{
    cv::Mat f1 = extractSurf(src);
    cv::Mat f2 = extractColor(src);

    cv::Mat combined(1, f1.cols+f2.cols, CV_32F, cv::Scalar(0));

    cv::Mat dstF1 = combined.row(0).colRange(0,f1.cols);
    f1.copyTo(dstF1);
    cv::Mat dstF2 = combined.row(0).colRange(f1.cols, combined.cols);
    f2.copyTo(dstF2);

    return combined;
}

cv::Mat FeatureExtractor::extractSurf(const cv::Mat &src) 
{
    //CV_Assert(false);
    return cv::Mat();
#if 0
    boost::timer timer;

    int borderSizeX = 16;
    int borderSizeY = 16;
    int radius      = 16;

    // create grid to sample surf descriptors
    double gridSizeX = (src.cols-borderSizeX*2) / ((double)N_GRID_POINTS_X-1);
    double gridSizeY = (src.rows-borderSizeY*2) / ((double)N_GRID_POINTS_Y-1);
    std::vector<cv::KeyPoint> gridPoints;
    for (int j = 0; j < N_GRID_POINTS_Y; j++)
    {
        for (int i = 0; i < N_GRID_POINTS_X; i++)
        {
            int pX = borderSizeX + i*gridSizeX;
            int pY = borderSizeY + j*gridSizeY;
            gridPoints.push_back(cv::KeyPoint(cv::Point2f(pX,pY), 2*radius));
        }
    }

    bool useProvidedKeypoints = true; // extract descriptors on grid
    double hessianThreshold = 400; // ignored if use provided keypoints
    int nOctaves            = 2;
    int nOctavesLayers      = 4;
    cv::SURF surf(hessianThreshold, nOctaves, nOctavesLayers);

    //std::cout << "descriptorSize: " << sift.descriptorSize() << std::endl;
    CV_Assert(surf.descriptorSize() == DIMS_SURF);

    // convert input image to gray scale
    cv::Mat imgGray;
    cv::cvtColor(src, imgGray, CV_BGR2GRAY);

    // extract surf descriptors on grid locations
    std::vector<float> descriptors;
    surf(imgGray, cv::Mat(), gridPoints, descriptors, useProvidedKeypoints);

    // convert vector to cv::Mat
    cv::Mat featureVector(1, descriptors.size(), CV_32F, cv::Scalar(0));
    float *featureVectorPtr = featureVector.ptr<float>(0);
    for (size_t i = 0; i < descriptors.size(); i++)
    {
        featureVectorPtr[i] = descriptors[i];
    }

        //cv::drawKeypoints(imgGray, gridPoints, imgGray, cv::Scalar(255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        //cv::imshow("keypoints", imgGray);
        //cv::waitKey();


    //std::cout << "surf extraction time: " << timer.elapsed() << "s" << std::endl;

    return featureVector;
#endif
}

#if COLOR_HIST_1D 
cv::Mat FeatureExtractor::extractColor(const cv::Mat &src)
{
    const bool display = false;
    int histSize[] = {N_BINS_COLORPLANE};
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
    if (display)
    {
        cv::imshow("L", colorPlanes[0]);
        cv::imshow("A", colorPlanes[1]);
        cv::imshow("B", colorPlanes[2]);
        cv::imshow("Mask", mask*255);
    }
    cv::calcHist(&lab, 1, channelA, mask, histA, 1, histSize, ranges, true); 
    cv::calcHist(&lab, 1, channelB, mask, histB, 1, histSize, ranges, true); 

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
    hist = hist * 100  / nonzero ;//* 100.0 / (nonzero);

    // smooth histogram 
    cv::blur(hist, hist, cv::Size(3,3));
    //cv::blur(hist, hist, cv::Size(3,3));

    // display 2d-histogram
    if (display)
    {
        cv::Mat histImg;
        cv::resize(hist, histImg, cv::Size(), 5, 10, cv::INTER_NEAREST);
        cv::imshow("colorhist", Util::drawHistogram(histImg));///cv::norm(histImg, cv::NORM_INF)));
        cv::waitKey();
    }

    // return as 1d-vector
    return hist.reshape(0, 1);
}
#else
cv::Mat FeatureExtractor::extractColor(const cv::Mat &src)
{
    bool display = false;
    int histSize[] = {N_BINS_A, N_BINS_B};
    float a_ranges[] = { 0, 255 };
    float b_ranges[] = { 0, 255 };
    const float* ranges[] = {a_ranges, b_ranges};
      
    // we compute the histogram from channels 1 and 2 (a and b)
    int channels[] = {1, 2};

    // prepare image
    cv::Mat lab;
    cv::cvtColor(src, lab, CV_BGR2Lab);
    std::vector<cv::Mat> colorPlanes;
    cv::split(lab, colorPlanes);
  
    // create histogram from a* and b* planes of Lab color space
    cv::MatND histND(2, histSize, CV_32F, cv::Scalar(0));
    // use source image as mask -> ignore black pixels
    cv::Mat mask;
    cv::Mat ones = cv::Mat::ones(src.size(), CV_8U);
    ones.copyTo(mask, colorPlanes[0]);
    // display color planes
    //cv::imshow("A", colorPlanes[1]);
    //cv::imshow("B", colorPlanes[2]);
    //cv::imshow("mask", mask*255);
    //cv::waitKey();
    cv::calcHist(&lab, 1, channels, mask, 
        histND, 2, histSize, ranges, true); 

    int nonzero = cv::countNonZero(colorPlanes[0]);
    if (nonzero == 0)
	{
		nonzero = 1;
	}
    cv::Mat hist = histND * 100.0 / (nonzero);

    // smooth histogram 
    cv::blur(hist, hist, cv::Size(3,3));
    cv::blur(hist, hist, cv::Size(3,3));

    // display 2d-histogram
    if (display)
    {
        cv::Mat histImg;
        cv::resize(hist, histImg, cv::Size(), 10, 10, cv::INTER_NEAREST);
        cv::imshow("colorhist", histImg/cv::norm(histImg, cv::NORM_INF));
        cv::waitKey(0);
    }

    // return as 1d-vector
    return hist.reshape(0, 1);
}
#endif


