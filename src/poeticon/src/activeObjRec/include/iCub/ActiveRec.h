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
 * @file ActiveRec.h
 * @brief 
 */

#ifndef __ACTIVEREC_H__
#define __ACTIVEREC_H__

#include <vector>
#include <functional>
#include <list>

#include <opencv2/core/core.hpp>

#include <boost/timer.hpp>
#include <boost/foreach.hpp>

#include "iCub/ParticleFilter.h"
#include "iCub/GlobalSettings.h"


class ActiveRec
{
public:

    // 
    // Grid search
    //
    struct CellType
    {
        CellType(double e_, double r_, double sizeE_, double sizeR_, double h_)
            : e(e_), r(r_), sizeE(sizeE_), sizeR(sizeR_), h(h_)
        {};
        double h;
        double e;
        double r;
        double sizeE;
        double sizeR;
        inline cv::Rect rect() const
        {
            return cv::Rect(r-sizeR/2, e-sizeE/2, sizeR+0.5, sizeE+0.5);
        }
    };
    struct less_H //: public binary_function<CellType,CellType,bool> 
    {
        bool operator() (const CellType& x, const CellType& y) const {return x.h < y.h;} 
    };
    struct greater_H //: public binary_function<CellType,CellType,bool> 
    {
        bool operator() (const CellType& x, const CellType& y) const {return x.h >= y.h;} 
    };
    typedef std::list<CellType> CellQueue;


    ActiveRec(GlobalSettings &settings_, int maxIter=10, double desEntropy=0.1) :
        Nb_Iteration(maxIter), desiredEntropy(desEntropy), 
        particleFilter(settings_), settings(settings_)
    {
    }

    bool init(const std::string &objectViewDir);
    void restart();
    void update(const cv::Mat &img, const cv::Vec2f &transition, const cv::Vec2f &gaze);
    cv::Vec2f searchTargetGaze(const cv::Vec2f &currentGaze);
    cv::Mat calcBoostingFactors(const cv::Mat &observation);
    cv::Mat extractFeature(const cv::Mat &img) const;

    void setWriteData(const std::string &dir, const std::string &objectName);

    inline double getEntropy()
    {
        return particleFilter.getCurrentEntropy();
    }

    inline double calcEntropy(const cv::Mat m)
    {
        cv::Mat log_m;
        cv::log(m, log_m);
        cv::Mat h = m.mul(log_m);
        return -cv::sum(h)[0];
    }

    inline cv::Mat getObjectProbabilities()
    {
        return particleFilter.getObjectProbabilities();
    }
    inline void setMotionPlanning(bool active)
    {
        doMotionPlanning = active;
    }
    inline void setFastPlanning(bool active)
    {
        fastPlanning = active;
    }
    inline std::vector<std::string> objectNames() const
    {
        std::vector<std::string> names;
        BOOST_FOREACH(const Object obj, objects)
        {
            names.push_back(obj.getName());
        }
        return names;
    }

    inline ParticleFilter& getParticleFilter()
    {
        return particleFilter;
    }

    inline void setObjectRotationOffset(const double rot_deg)
    {
        settings.rotationOffset = rot_deg;
    }

private:
    std::vector<cv::Mat>  clusterDist;
    cv::Mat               clusterCenters;

    int             recogIter;
    double          currentEntropy;

    std::string     trueLabel;
    
    // Params
    bool            doMotionPlanning;
    // Stop critera:
    double          desiredEntropy;
    int             Nb_Iteration;
    
    PFObjectModel   pfObjectModel;
    ParticleFilter  particleFilter;

    cv::PCA         pca;

    cv::Mat         objProbsKNN1;
    cv::Mat         objProbsKNN3;
    cv::Mat         objProbsKNN5;

    boost::timer    recogTimer;
    
    bool            writeData;
    std::string     resultDir;
    std::string     resultDirPlannedMotion;
    std::string     resultDirRandomMotion;

    bool            fastPlanning;

    // Boosting params
    cv::Mat         p_co;
    cv::Mat         p_o;
    cv::Mat         p_c;

    // Loading object data:
    bool loadObjects(const std::string &objectViewDir);

    // Viewpoint Planning:
    cv::Mat createTargetMap(const cv::Vec2f &currentGaze);
    CellType createCell(double cellE, double cellR, double cellSizeE, double cellSizeR, 
            const cv::Vec2f &currentGaze);

    // calculates object probabilies without making use of propriocetion
    cv::Mat knnMatching(const cv::Mat &g, int knn);

    void writeIterationData(const cv::Mat &objProbs, 
            const cv::Mat &objProbsKNN1, 
            const cv::Mat &objProbsKNN3, 
            const cv::Mat &objProbsKNN5, 
        const cv::Vec2f &gaze, const cv::Vec2f &target,
        const cv::Mat &entropyImg,
        const cv::Mat &objImage);

    
    inline int objectCount() { return objects.size(); }

    void initBoosting();

    // Visualization:
    std::vector<cv::Mat> createViewSimImages(const cv::Mat &feature, const cv::Vec2f &gaze);
    std::vector<cv::Mat> viewSimImgs;
    std::vector<cv::Mat> particleImgs;
    cv::Mat entropyImg;
    //void showBestKeyframes();

    GlobalSettings &settings;

    std::vector<Object> objects;
};


#endif

//----- end-of-file --- ( next line intentionally left blank ) ------------------


