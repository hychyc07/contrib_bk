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
 * @file ParticleFilter.cpp
 * @brief implementation of the ParticleFilter methods.
 */

#include <iostream>
#include <vector>

#include <opencv2/highgui/highgui.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/timer.hpp>
#include <boost/foreach.hpp>

#include "iCub/ParticleFilter.h"
#include "iCub/Util.h"
#include "iCub/GlobalSettings.h"
//#include "iCub/cvmat_serialization.h"


#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

const double PI = std::atan(1.0)*4;
const double INV_2PI = 0.5*std::sqrt(2*PI);


cv::Mat PFObjectModel::drawSample(const cv::Mat &state, const cv::Vec2f &transition) const
{
    // noise params
    double u     = 0;
    double sigma = 1;
    double scale = 5;

    cv::Mat sample(state.size(), state.type());

    // col 0 specifies object no
    // we always draw samples from the current object
    sample.at<float>(0) = state.at<float>(0);

    // add transition to state plus gaussian noise
    sample.at<float>(1) = state.at<float>(1) + transition[0] + Random::normal(u,sigma)*scale;
    sample.at<float>(2) = state.at<float>(2) + transition[1] + Random::normal(u,sigma)*scale;

    // handle wrap-arounds
    if (sample.at<float>(1) < 0)
    {
        sample.at<float>(1) *= -1;
        sample.at<float>(2) +=180;
    }
    if (sample.at<float>(2) < 0)    sample.at<float>(2) += 360;
    if (sample.at<float>(2) >= 360) sample.at<float>(2) -= 360;

    return sample;
}


cv::Mat PFObjectModel::drawSample(int k) const
{
    cv::Mat sample(1, 3, CV_32F);

    sample.at<float>(0) = k;
    sample.at<float>(1) = Random::uniform(minElevation, maxElevation+10); 
    sample.at<float>(2) = Random::uniform(minRotation-10, maxRotation+10); 
    //sample.at<float>(1) = Random::uniform(minElevation, maxElevation+45); 
    //sample.at<float>(2) = Random::uniform(minRotation-45, maxRotation+45); 
    //sample.at<float>(2) = Random::uniform(60, 340); 

    return sample;
}

cv::Mat PFObjectModel::drawObservation(const cv::Mat &state) const
{
    cv::Mat state_noise = drawSample(state, cv::Vec2f(0,0));
    return getView(state_noise);
}

// P(Y|X)
double PFObjectModel::evalObservation(const cv::Mat &state, const cv::Mat &observation) const
{
    int k = state.at<float>(0);
    double e = state.at<float>(1);
    double r = state.at<float>(2);
    CV_Assert(objects.size() > k);
    return objects[k].matchFeature(e, r, observation);
}


void ParticleFilter::init(const PFObjectModel &m, int particlesPerObj)
{
    model = m;
    particlesPerObject = particlesPerObj;
    particleCount = model.objectCount() * particlesPerObject;

    particles = cv::Mat::zeros(particleCount, 3, CV_32F); 
    W = cv::Mat::ones(particleCount, 1, CV_32F);
    lastW = W.clone();

    std::cout << "\tnum objects: " << model.objectCount() << std::endl; 
    std::cout << "\tsampling uniform..." << std::flush; 

    // initialize with uniform particle distribution
    for (int k = 0; k < model.objectCount(); k++)
    {
        for (int i = 0; i < particlesPerObject; i++)
        {
            int p = k*particlesPerObject+i;
            cv::Mat tmp = particles.row(p);
            model.drawSample(k).copyTo(tmp);
        }
    }
    std::cout << "\t[OK]" << std::endl; 
}

cv::Mat sampleFromDistribution(const cv::Mat &dist, int nSamples)
{
    if (!(dist.rows == 1 || dist.cols == 1))
        std::cout << dist.rows << " " << dist.cols << std::endl;
    CV_Assert(dist.rows == 1 || dist.cols == 1);
    int length = std::max(dist.cols, dist.rows);
    cv::Mat cumsum(length, 1 , dist.type(), cv::Scalar::all(0));
    cumsum.at<float>(0) = dist.at<float>(0);
    for (int i = 1; i < length; i++)
        cumsum.at<float>(i) = cumsum.at<float>(i-1) + dist.at<float>(i);
    cumsum /= cv::norm(cumsum, cv::NORM_INF);

    cv::Mat samples(nSamples, 1, CV_32F);
    cv::Mat r(nSamples, 1, CV_32F);
    cv::randu(r,0,1);
    for (int i = 0; i < nSamples; i++)
    {
        float ri = r.at<float>(i); // random number [0,1]
        int j = 0;  // will hold the selected dimension of the distribution
        while (j < cumsum.rows && cumsum.at<float>(j) < ri) j++;
        samples.at<float>(i) = j;
    }

    return samples;
}


void ParticleFilter::resample()
{
    // compute cumulative sum
    cv::Mat cumsum(W.size(), W.type(), cv::Scalar::all(0));
    cumsum.at<float>(0) = W.at<float>(0);
    for (int i = 1; i < W.rows; i++)
    {
        //cumsum.at<float>(i) = cumsum.at<float>(i-1) + current_w.at<float>(i)*current_w.at<float>(i);
        cumsum.at<float>(i) = cumsum.at<float>(i-1) + current_w.at<float>(i);
    }
    cumsum /= cv::norm(cumsum, cv::NORM_INF);

    // pick particles proportional to current weights
    cv::Mat newW(W.size(), W.type());
    cv::Mat newParticles(particles.size(), particles.type());
    cv::Mat r(W.size(), W.type());
    cv::randu(r,0,1);
    for (int i = 0; i < W.rows; i++)
    {
        float ri = r.at<float>(i); // random number [0,1]
        int j = 0;  // will hold the id of the new particle
        while (j < cumsum.rows && cumsum.at<float>(j) < ri)
        {
            j++;
        }
        // assign new weight
        newW.at<float>(i) = W.at<float>(j);
        // assign new particle
        cv::Mat dst = newParticles.row(i);
        particles.row(j).copyTo(dst);
    }

    newW.copyTo(W);
    newParticles.copyTo(particles);
}


void ParticleFilter::filter(const cv::Mat &observation, const cv::Vec2f &transition, const cv::Mat &boostingFactors)
{
    if (model.empty())
    {
        std::cerr << "ParticleFilter not initialized. Call init() first.";
        return;
    }

    cv::Mat dst;
    cv::Mat newParticle;

    //std::cout << "\t\tsampling new particles..." << std::endl;

    // sample new particles
    boost::timer timer;
    for (int i = 0; i < particles.rows; i++)
    {
        dst = particles.row(i);
        newParticle = model.drawSample(particles.row(i), transition);
        newParticle.copyTo(dst);
    }
    //std::cout << "new particles sampled in (s): " << timer.elapsed() << std::endl; 
    
    // compute new weights
    timer.restart();
    cv::Mat newW = lastW.clone();
    for (int i = 0; i < particles.rows; i++)
    {
        if (! model.outOfBounds(particles.row(i)))
            newW.at<float>(i) = model.evalObservation(particles.row(i), observation);
        else 
            newW.at<float>(i) = std::numeric_limits<double>::quiet_NaN(); // flag oob viewpoint with NaN
    }
    //std::cout << "new weights computed in (s): " << timer.elapsed() << std::endl; 
    
    double oobWeight = 0.10; // default weight for oob viewpoints

    // scale (valid) weights
    cv::normalize(newW, newW, oobWeight, 1, cv::NORM_MINMAX);

    // replace nans with default weight for oob viewpoints 
    for (int i = 0; i < particles.rows; i++)
    {
        if (std::isnan(newW.at<float>(i)))
            newW.at<float>(i) = oobWeight;
    }

    if (! boostingFactors.empty())
    {
        newW /= cv::sum(newW)[0];
        cv::Mat boostedW(newW.size(), CV_32F);

        for (int i = 0; i < particles.rows; i++)
        {
            int objNo = particles.at<float>(i,0);
            boostedW.at<float>(i) = boostingFactors.at<float>(objNo)*newW.at<float>(i);
        }
        boostedW /= cv::sum(boostedW)[0];
        newW = settings.boostingRate*boostedW + (1-settings.boostingRate)*newW;
        newW /= cv::sum(newW)[0];
    }
    else
    {
        std::cout << "no boosting..." << std::endl;
    }

    //normalize new weights
    //newW /= cv::sum(newW)[0];

    lastW       = newW;
    current_w   = newW;

    newW.copyTo(W);

#define SHOW_PARTICLES_BEFORE_REAMPLING 0
#if SHOW_PARTICLES_BEFORE_REAMPLING
    // show particles before resampling
    std::vector<cv::Mat> particleImgs = createParticleImages(true, cv::Vec2f());
    std::vector<std::string> objectNames;
    cv::Mat particleVis = Util::tileImgs(particleImgs, 5, 5, objectNames);
    cv::imshow("particlesW", particleVis);

    {
    std::vector<cv::Mat> particleImgs = createParticleImages(false, cv::Vec2f());
    std::vector<std::string> objectNames;
    cv::Mat particleVis = Util::tileImgs(particleImgs, 5, 5, objectNames);
    cv::imshow("particles", particleVis);
    //cv::waitKey();
    }
#endif

    // update weights
    //double alpha = 0.2;
    //W = W*(1-alpha) + newW*alpha;
    ////W = W.mul(newW);

    //// normalize weights
    //W /= cv::sum(W)[0];

    //std::cout << "updated weights:\n" << W << std::endl;

    // resample from new distribution
    //cv::Mat w_square = W.mul(W);
    //double neff = 1.0/ cv::sum(w_square)[0];
    ////std::cout << "neff " << neff << std::endl;
    //if (neff < W.rows/2)
        resample(); 

    W = cv::Mat::ones(W.rows, 1, CV_32F);

}


double ParticleFilter::calcExpectedVariance(const cv::Vec2f &action) const
{
    int i;
    cv::Mat p, obs;
    std::vector<cv::Mat> observations_hat;
    std::vector<int> observations_obj;

    // prefer particles with higher weights
    cv::Mat sortedIds;
    //cv::sortIdx(W, sortedIds, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING);

    int nSamples = std::min(200, particles.rows);
    
#define PROB_SAMPLING 1

#if PROB_SAMPLING

    // probabilistic sampling according to current weights
    
    // compute cumulative sum
    cv::Mat cumsum(W.size(), W.type(), cv::Scalar::all(0));
    cumsum.at<float>(0) = W.at<float>(0);
    for (int i = 1; i < W.rows; i++)
    {
        //cumsum.at<float>(i) = cumsum.at<float>(i-1) + W.at<float>(i);
        cumsum.at<float>(i) = cumsum.at<float>(i-1) + W.at<float>(i)*W.at<float>(i);
    }
    cumsum /= cv::norm(cumsum, cv::NORM_INF);
    // sample particles and store expected observations
    cv::Mat r(nSamples, 1, W.type());
    cv::randu(r,0,1);
    for (int i = 0; i < nSamples; i++)
    {
        float ri = r.at<float>(i); // random number [0,1]
        int j = 0;  // will hold the id of the new particle
        while (j < cumsum.rows && cumsum.at<float>(j) < ri)
        {
            j++;
        }
        p = model.drawSample(particles.row(j), action);
        if (model.outOfBounds(p))
            continue;
        obs = model.getView(p);
        if (obs.empty())
            continue;
        observations_hat.push_back(obs);
        observations_obj.push_back(p.at<float>(0));
    }
#else
    // pick best N particles
    for (int s = 0; s < nSamples; s++)
    {
        //int i = sortedIds.at<int>(s);
        //std::cout << i << " ";
        int i = rand()%particles.rows;
        p = model.drawSample(particles.row(i), action);
        if (model.outOfBounds(p))
            continue;
        obs = model.drawObservation(p);
        //obs = model.getView(p);
        if (obs.empty())
            continue;
        observations_hat.push_back(obs);
        observations_obj.push_back(p.at<float>(0));
    }
#endif

    // estimate variance of observations between objects
    double w,d;
    double sum = 0;
    double cnt = 0;
    for (int i = 0; i < observations_hat.size(); i++)
    {
        for (int j = i; j < observations_hat.size(); j++)
        {
            w = 1;
            if (observations_obj[i] == observations_obj[j])
                w = 0.05;
            d = cv::norm(observations_hat[i], observations_hat[j], cv::NORM_L2);
            if (d != d)
                continue;
            sum += w * d;
            cnt += w;
        }
    }

    //std::cout << "num cmps: " << observations_hat.size() * observations_hat.size() << std::endl;

    return cnt > 0 ? sum / cnt : 0;
}

double ParticleFilter::calcExpectedEntropy(const cv::Vec2f &action) const
{
    // use particles with highest weights
    cv::Mat sortedIds;
    cv::sortIdx(current_w, sortedIds, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING); 

    int nSamples = std::min(200, particles.rows);
   
    std::vector<cv::Mat> particles_hat;
    //cv::Mat particles_hat(nSamples, particles.cols, particles.type());

    // sample new particles
    boost::timer timer;
    cv::Mat dst;
    cv::Mat prt;
    for (int i = 0; i < nSamples; i++)
    {
        //int j = sortedIds.at<int>(i);
        int j = rand()%particles.rows;
        prt = model.drawSample(particles.row(j), action);
        if (model.outOfBounds(prt))
            continue;
        particles_hat.push_back(prt);
    }

    // sample observations
    int nObservationSamples = std::min(200, (int)particles_hat.size());
    std::vector<cv::Mat> observations_hat;
    for (int s = 0; s < nObservationSamples; s++)
    {
        //int j = sortedIds.at<int>(s);
        int j = rand()%particles_hat.size();
        cv::Mat p_hat = particles_hat[j];
        if (model.outOfBounds(p_hat))
            continue;
        cv::Mat obs = model.drawObservation(p_hat);
        if (obs.empty())
            continue;
        observations_hat.push_back(obs);
    }

    // compute expected entropy for sampled particles and observations
    //timer.restart();
    cv::Mat newW = cv::Mat::ones(particles_hat.size(), 1, CV_32F);
    //std::cout << "obs:   " << observations_hat.size() << std::endl;
    //std::cout << "parts: " << particles_hat.rows << std::endl;
    //exit(0);
    double h_a = 0;
    for (int s = 0; s < observations_hat.size(); s++)
    {
        // p(Y_hat|X)
        for (int i = 0; i < particles_hat.size(); i++)
        {
            if (! model.outOfBounds(particles_hat[i]))
                newW.at<float>(i) = model.evalObservation(particles_hat[i], observations_hat[s]);
            else 
                newW.at<float>(i) = 0.2;
        }
        newW /= cv::sum(newW)[0];
        h_a += calcEntropy(newW, particles_hat);
    }
    if (! observations_hat.empty())
        h_a /= observations_hat.size();

    return h_a;
}

std::vector<int> ParticleFilter::getParticleCounts() const
{
    std::vector<int> particleCounts;
    particleCounts.resize(model.objectCount());
    for (int i = 0; i < particles.rows; i++)
    {
        int k = particles.at<float>(i, 0);
        particleCounts[k]++;
    }
    return particleCounts;
}

std::vector<cv::Point2f> ParticleFilter::getModes() const
{    
    std::vector<int> particleCounts = getParticleCounts();
    std::vector<cv::Point2f> modes;
    std::vector<float> sum_weights;
    modes.resize(model.objectCount());
    sum_weights.resize(model.objectCount());

    for (int i = 0; i < particles.rows; i++)
    {
        int k = particles.at<float>(i, 0);
        cv::Point2f pt(particles.at<float>(i, 2), particles.at<float>(i, 1));
        modes[k] += current_w.at<float>(i) * pt;
        sum_weights[k] += current_w.at<float>(i);
    }
    for (size_t k = 0; k < modes.size(); k++)
    {
        if (particleCounts[k] > 0)
        {
            //modes[k].x /= particleCounts[k];
            //modes[k].y /= particleCounts[k];
            modes[k].x /= sum_weights[k];
            modes[k].y /= sum_weights[k];
        }
    }
    return modes;
}

std::vector<cv::Mat> ParticleFilter::createParticleImages(bool scaled, const cv::Vec2f &gaze) const
{
    std::vector<cv::Mat> plots;

    cv::Rect roi = cv::Rect(settings.minRotation, settings.minElevation, 
                settings.maxRotation-settings.minRotation, settings.maxElevation-settings.minElevation);

    int offsetY = 30;
    cv::Rect reachableArea = roi;
    reachableArea.y += offsetY;//reachableArea.height;

    // create one plot for each object
    for (int k = 0; k < model.objectCount(); k++)
    {
        //cv::Mat plot(roi.height, roi.width, CV_8UC3, cv::Scalar::all(255));
        cv::Mat plot(roi.y+roi.height*2+offsetY, roi.x+roi.width*2, CV_8UC3, cv::Scalar::all(255));
        // mark reachable area
        cv::rectangle(plot, reachableArea.tl(), reachableArea.br(), cv::Scalar::all(0));

        plots.push_back(plot);
    } 
    
    // set point radius proportional to particle weight 
    double maxRadius = 6; // best particle will be drawn that large
    cv::Mat particleRadius;
    cv::normalize(W, particleRadius, 0, 1, cv::NORM_MINMAX);
    particleRadius *= maxRadius;

    // draw particles
    for (int i = 0; i < particles.rows; i++)
    {
        int k = particles.at<float>(i, 0);
        cv::Point pt(particles.at<float>(i, 2), particles.at<float>(i, 1)+offsetY);
        //cv::Scalar color = cv::Scalar(rand()%255, rand()%255, rand()%255);
        cv::Scalar color = cv::Scalar(255,0,0); // blue
        if (scaled)
            cv::circle(plots[k], pt, particleRadius.at<float>(i), color, -1, CV_AA);  
        else
            cv::circle(plots[k], pt, 1, color, -1, CV_AA);  
    }
    // draw gaze position
    if (gaze != cv::Vec2f())
    {
        for (int pl = 0; pl < plots.size(); pl++)
        {
            cv::Point p(gaze[1], gaze[0]+offsetY);
            cv::circle(plots[pl], p, 3, cv::Scalar(0,0,0), -1);  
            // draw gaze + obj rotation offset (= true view point)
            cv::circle(plots[pl], p+cv::Point(-settings.rotationOffset,0), 3, cv::Scalar(0,255,0), -1, CV_AA);  
        }
    }

    // draw keyview positions
    for (int pl = 0; pl < plots.size(); pl++)
    {
        BOOST_FOREACH(const Object::Viewpoint &vp, model.objects[pl].getKeyviewViewpoints())
        {
            cv::Point pt(vp[Object::_R_], vp[Object::_E_]+offsetY);
            cv::circle(plots[pl], pt, 2, cv::Scalar(0,0,0), 1);
        }
    }

    std::vector<cv::Point2f> modes = getModes();
    std::vector<int> particleCounts = getParticleCounts();
    // draw estimated view point
    for (int k = 0; k < plots.size(); k++)
    {
        // draw mode
        cv::circle(plots[k], modes[k]+cv::Point2f(0,offsetY), 3, cv::Scalar(0,0,255), -1, CV_AA);  

        // show number of paritcles
        cv::putText(plots[k], boost::lexical_cast<std::string>(particleCounts[k]), cv::Point(0,40), 
                cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar::all(0)); 
    }

    return plots;
}



