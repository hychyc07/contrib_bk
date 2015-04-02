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
 * @file Object.h
 * @brief 
 */

#ifndef __OBJECT_H__
#define __OBJECT_H__

#include <iostream>
#include <opencv2/core/core.hpp>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "iCub/FeatureExtractor.h"


class Object
{
public:
    typedef std::vector<cv::Mat> ViewMap;
    typedef std::vector<cv::Mat> FeatureVector;
    typedef cv::Vec2f Viewpoint;

    static const int _E_ = 0;
    static const int _R_ = 1;

    inline double matchFeature(const Viewpoint &vp, const cv::Mat &feature) const
    {
        return matchFeature(vp[_E_], vp[_R_], feature);
    }
    inline double matchFeature(double e, double r, const cv::Mat &feature) const
    {
        return compareFeatures(getFeature(e,r), feature);
    }

    void addKeyview(const cv::Mat &image, double e, double r);

    bool save(const std::string &dataDir) const;
    bool load(const std::string &objectPath);

    cv::Mat createViewSimImage(const cv::Mat &feature, const cv::Vec2f &gaze) const;

    inline size_t getKeyviewCount() const
    {
        return keyviewViewpoints.size();
    }

    inline std::string getName() const
    {
        return objectName;
    }

    inline const std::vector<std::string>&  getKeyviewPaths() const
    {
        return keyviewPaths;
    }

    inline cv::Mat getKeyviewFeatures()
    {
        if (keyviewFeatures.empty() && !keyviewImages.empty())
            extractKeyviewFeatures();
        return keyviewFeatures;
    }
    inline const std::vector<Viewpoint>& getKeyviewViewpoints() const
    {
        return keyviewViewpoints;
    }

    inline cv::Mat getFeature(const Viewpoint &vp) const
    {
        return getFeature(vp[_E_], vp[_R_]);
    }

    inline cv::Mat getFeature(int e, int r) const
    {
        e -= minElevation;
        r -= minRotation;
        if (e < 0 || r < 0 || e >= rangeE || r >= rangeR)
        {
            return cv::Mat();
        }
        int idx = e*rangeR+r;
        CV_Assert(idx < viewMap.size());
        return viewMap[idx];
    }

    static inline int getFeatureLength()
    {
        return FeatureExtractor::featureLength(FEATURE_TYPE);
    }
    static cv::Mat extractFeature(const cv::Mat &img);
   
    double matchKNN(const cv::Mat &feature, int knn) const;

private:

    static const int minElevation =  0;
    static const int maxElevation = 90;
    static const int minRotation = 140;
    static const int maxRotation = 280;
    static const int rangeR = maxRotation - minRotation;
    static const int rangeE = maxElevation - minElevation;

    static const FeatureExtractor::FeatureType FEATURE_TYPE = FeatureExtractor::COLOR;

    void extractKeyviewFeatures();
    cv::Mat interpolateView(double e, double r) const;
    void buildViewMap();
    cv::Mat loadFeatures(const std::string &objectPath, FeatureExtractor::FeatureType featureType);
    void saveKeyview(const fs::path &objectDir, const cv::Mat &image, double e, double r) const;

    cv::Mat keyviewFeatures;
    std::vector<cv::Mat> keyviewImages;
    std::vector<Viewpoint> keyviewViewpoints;
    std::vector<std::string> keyviewPaths;

    ViewMap viewMap;
    std::string dataDir;

    int id;
    std::string objectName;
    
    static const double featureScaling = 10.0;

    inline double l2(const cv::Mat &f1, const cv::Mat &f2) const
    {
        double d = cv::norm(f1, f2, cv::NORM_L2)/(f1.cols);
        if (d != d)
            return 0;
        double p = std::exp(-featureScaling*d);
        return p; 
    }

    inline double compareFeatures(const cv::Mat &f1, const cv::Mat &f2) const
    {
        //std::cout << "l2: " << l2(f1,f2)<< std::endl;
        return l2(f1,f2);
    }
};

#endif

