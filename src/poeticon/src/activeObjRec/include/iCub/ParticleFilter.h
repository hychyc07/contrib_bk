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
 * @file ParticleFilter.h
 * @brief 
 */

#ifndef __PARTICLE_FILTER_H__
#define __PARTICLE_FILTER_H__

#include <opencv2/core/core.hpp>

#include "iCub/Object.h"

class GlobalSettings;

class PFObjectModel
{
public:

    friend class ParticleFilter;

    PFObjectModel() {} ;
    PFObjectModel(const std::vector<Object> &objects_,
            double minE, double maxE, double minR, double maxR) :
        objects(objects_)
    { 
        init(objects_, minE, maxE, minR, maxR);
    }

    bool init(const std::vector<Object> &objects_, double minE, double maxE, double minR, double maxR)
    {
        minElevation = minE;
        minRotation  = minR;
        maxElevation = maxE;
        maxRotation  = maxR;

        objects = objects_;

        rangeE = maxElevation-minElevation;
        rangeR = maxRotation-minRotation;

        return true;
    };

    cv::Mat drawSample(const cv::Mat &state, const cv::Vec2f &transition) const;
    cv::Mat drawSample(int k) const;
    cv::Mat drawObservation(const cv::Mat &state) const;
    double evalObservation(const cv::Mat &state, const cv::Mat &observation) const;

    inline bool empty() const { return objects.empty(); }
    inline size_t objectCount() const { return objects.size(); }
    
    inline bool outOfBounds(const cv::Mat &particle) const
    {
        int e = particle.at<float>(1);
        int r = particle.at<float>(2);
        return (e < minElevation || r < minRotation || e >= maxElevation || r >= maxRotation);
    }

    inline cv::Mat getView(const cv::Mat &state) const
    {
        int k = state.at<float>(0);
        double e = state.at<float>(1);
        double r = state.at<float>(2);
        return objects[k].getFeature(e,r);
    }

    inline double chi_square(const cv::Mat &f1, const cv::Mat &f2) const
    {
        double s=0;
        for (int i=0; i<f1.cols; i++)
        {
            if (f2.at<float>(i) > 0)
            {
                double x=f1.at<float>(i)-f2.at<float>(i);
                s += (x*x)/f2.at<float>(i);
            }
        }
        return std::exp(-s);
    }

private:
    double minElevation;
    double maxElevation;
    double minRotation;
    double maxRotation;
    int rangeE;
    int rangeR;

    std::vector<Object> objects;
};

class ParticleFilter
{
public:
    ParticleFilter(GlobalSettings &settings_): settings(settings_) {};

    void init(const PFObjectModel &m, int particlesPerObj = 150);
    void filter(const cv::Mat &observation, const cv::Vec2f &transition, const cv::Mat &boostingFactors);

    double calcExpectedEntropy(const cv::Vec2f &action) const;
    double calcExpectedVariance(const cv::Vec2f &action) const;

    // visualizations
    std::vector<cv::Mat> createParticleImages(bool scaled, const cv::Vec2f &gaze) const;
    std::vector<int> getParticleCounts() const;
    std::vector<cv::Point2f> getModes() const;

    inline const cv::Mat getParticles() const { return particles; }
    inline const cv::Mat getWeights() const { return W; }

    inline cv::Mat getObjectProbabilities() const
    {
        return calcObjectProbabilities(W, particles);
    }

    inline double getCurrentEntropy() const
    {
        return calcEntropy(W, particles);
    }

private:

    inline double calcEntropy(const cv::Mat &w, const std::vector<cv::Mat> &particles) const
    {
        cv::Mat p = calcObjectProbabilities(w, particles);
        cv::Mat log_p;
        cv::log(p, log_p);
        cv::Mat h = p.mul(log_p);
        return -cv::sum(h)[0];
    }
    inline cv::Mat calcObjectProbabilities(const cv::Mat &w, const std::vector<cv::Mat> &particles) const
    {
        int k;
        cv::Mat probs(model.objectCount(), 1, CV_32F, cv::Scalar::all(0));
        for (int i = 0; i < particles.size(); i++)
        {
            k = particles[i].at<float>(0);
            probs.at<float>(k) += w.at<float>(i);
        }
        probs /= cv::sum(probs)[0];
        return probs;
    }
    inline double calcEntropy(const cv::Mat &w, const cv::Mat &particles) const
    {
        cv::Mat p = calcObjectProbabilities(w, particles);
        cv::Mat log_p;
        cv::log(p, log_p);
        cv::Mat h = p.mul(log_p);
        return -cv::sum(h)[0];
    }
    inline cv::Mat calcObjectProbabilities(const cv::Mat &w, const cv::Mat &particles) const
    {
        int k;
        cv::Mat probs(model.objectCount(), 1, CV_32F, cv::Scalar::all(0));
        for (int i = 0; i < particles.rows; i++)
        {
            k = particles.at<float>(i,0);
            probs.at<float>(k) += w.at<float>(i);
        }
        probs /= cv::sum(probs)[0];
        return probs;
    }

    void resample();

    PFObjectModel model;
    cv::Mat particles;
    cv::Mat W;
    cv::Mat lastW;
    cv::Mat current_w;

    // params
    int particleCount;
    int particlesPerObject;

    GlobalSettings &settings;
};


#endif 

//----- end-of-file --- ( next line intentionally left blank ) ------------------

