// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-    
/*
\author Shiraj Sen and Stephen Hart

Copyright (C) 2010 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.
**/

#ifndef __PROBABILITYDISTRIBUTION_H__
#define __PROBABILITYDISTRIBUTION_H__

#include <yarp/sig/Vector.h>
#include <string>

/**
 * ProbabilityDistribution Class : This class is an abstract interface to a PDF.
 **/

class ProbabilityDistribution {
  
 protected:
  
  int dimension_count;

 public:

  ProbabilityDistribution(int count) {
    dimension_count = count;
  }

  /**
   * Destructor
   **/
  ~ProbabilityDistribution() {};
  
    int getDimensionCount() { return dimension_count; }
    
    virtual int addSample(yarp::sig::Vector dimension, double val) = 0;
    
    virtual int drawSample(yarp::sig::Vector &sample) = 0;
    
    virtual int loadDistribution(std::string str) = 0;
  
    virtual int saveDistribution(std::string str) = 0;

    virtual double getProbability(yarp::sig::Vector sample) = 0;

};

#endif
