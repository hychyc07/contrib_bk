// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-    
/*
\author Stephen Hart

Copyright (C) 2010 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.
**/
#ifndef _GAUSSIAN_DISTRIBUTION__H_
#define _GAUSSIAN_DISTRIBUTION__H_

#include <vector>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include "ProbabilityDistribution.h"

class GaussianProbabilityDistribution : public ProbabilityDistribution {

protected:

    std::vector<yarp::sig::Vector *> data;

    void computeMeanFromData();

    void computeCovarFromData();

    yarp::sig::Matrix covar;
    
    yarp::sig::Vector mean;
    
public:  

    GaussianProbabilityDistribution(int count);
    
    ~GaussianProbabilityDistribution();
    
    int addSample(yarp::sig::Vector sample, double val=1.0);
    
    int drawSample(yarp::sig::Vector &sample);
    
    int loadDistribution(std::string str);
  
    int saveDistribution(std::string str);
    
    double getProbability(yarp::sig::Vector sample);

};

#endif
