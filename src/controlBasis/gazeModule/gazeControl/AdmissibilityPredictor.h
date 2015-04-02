// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _ADMISSIBILITY_PREDICTOR__H_
#define _ADMISSIBILITY_PREDICTOR__H_

#include <iostream>
#include <string>

#include <yarp/sig/Vector.h>

#include <controlBasis/GaussianProbabilityDistribution.h>

class AdmissibilityPredictor {

protected:

    int size;
    double threshold;

    ProbabilityDistribution *dist;

public:

    AdmissibilityPredictor(int s=1, double thresh=0.5) :
        size(s),
        threshold(thresh),
        dist(NULL)
    {
    }

    ~AdmissibilityPredictor() { 
        if(dist!=NULL) delete dist; dist=NULL;
    }
    
    double getProbability(yarp::sig::Vector vals) {
        return dist->getProbability(vals);
    }

    bool isAdmissible(yarp::sig::Vector vals) {
        double p = dist->getProbability(vals);
        std::cout << "admissibility predictor got prob=" << p << std::endl; 
        return (p > 0.5);
    }

    void addSample(yarp::sig::Vector sample) {
        dist->addSample(sample, 1.0);
    }

    int loadDistribution(std::string str) {
        return dist->loadDistribution(str);
    }
  
    int saveDistribution(std::string str) {
        return dist->saveDistribution(str);
    }
    
};


class GaussianAdmissibilityPredictor : public AdmissibilityPredictor {
    
protected:
    
    //GaussianAdmissibilityPredictor *gaussian;
    
public:
    
    GaussianAdmissibilityPredictor(int s) :
        AdmissibilityPredictor(s)
        //    gaussian(NULL)
    {
        dist = new GaussianProbabilityDistribution(s);
    }

    ~GaussianAdmissibilityPredictor() {
        if(dist!=NULL) delete dist; dist=NULL;
    }

    //    double predict(yarp::sig::Vector) {}

};


#endif 
