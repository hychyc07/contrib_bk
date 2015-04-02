/* 
 * Copyright (C) 2012 Istituto Italiano di Tecnologia
 * Author: Elena Ceseracciu
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include "iCub/DMP/DmpGPR.h"
#include <iostream>
#include <yarp/math/Math.h>

#include "iCub/DMP/DMPPeriodic.h"
#include "iCub/DMP/DMPPointToPoint.h"

void DmpGPR::reset()
{
    this->init=false;
    for (std::vector<iCub::learningmachine::GPRLearner>::iterator gpIt=gprs.begin(); gpIt!=gprs.end(); ++gpIt)
           gpIt->reset();
    
}

bool DmpGPR::checkConstants(const dmp::DMP* y)
{
    if(!init)
    {
        dmpConstants=y->get_DMP_constants();
        dmpType=y->getType();
        init=true;
        return true;
    }
    else
    {
        return dmpType==y->getType() && y->get_DMP_constants()==this->dmpConstants;
    }
    
}

bool DmpGPR::feedSample(const yarp::sig::Vector &x, const dmp::DMP* y)
{   
    if (!checkConstants(y))
        return false; //should print out message as well?
    if (x.size()!=m_)
        return false; //should print out message as well?

    yarp::sig::Vector dmpParams=y->get_DMP_variables();
    for (size_t j=0; j<dmpParams.length(); ++j)
        gprs.at(j).feedSample(x, yarp::sig::Vector(1, dmpParams(j)));  
    sampleCount++;
    std::cout << sampleCount << std::endl;
    return true;
}

bool DmpGPR::inference()
{
    for (std::vector<iCub::learningmachine::GPRLearner>::iterator gpIt=gprs.begin(); gpIt!=gprs.end(); ++gpIt)
           gpIt->train();
    return true; //should add "bool train" method to GPRLearner ?
}

bool DmpGPR::inference(const std::vector <yarp::sig::Vector> & x, const std::vector<dmp::DMP*>& y, bool optimizeHyperparameters)
{
    if (x.size()!=y.size())
    {
        std::cout<< "vector of targets x and vector of dmp parameters y must have the same size!"<< std::endl;
        return false;        
    }
    // feed samples
    std::vector<yarp::sig::Vector>::const_iterator xIt=x.begin();
    for ( std::vector<dmp::DMP*>::const_iterator yIt=y.begin();xIt<x.end() && yIt<y.end(); ++xIt, ++yIt)
    {
        feedSample(*xIt, *yIt);
    }
   if (optimizeHyperparameters)
   {
       double tol= 0.01;
       int max_iter=100;
       unsigned int verb=0;
       bool useHessian=false;
       for (std::vector<iCub::learningmachine::GPRLearner>::iterator gpIt=gprs.begin(); gpIt!=gprs.end(); ++gpIt)
           gpIt->optimize(tol, max_iter, verb, useHessian); 
   }
   
   return inference();
}


std::vector<dmp::DMP*> DmpGPR::generalize(const std::vector <yarp::sig::Vector> & xs)
{
    std::vector<dmp::DMP*> ys;
    for (std::vector <yarp::sig::Vector>::const_iterator xsIt=xs.begin(); xsIt!=xs.end(); ++xsIt)
    {
        ys.push_back(generalize(*xsIt));
    }
    return ys;
}

dmp::DMP* DmpGPR::generalize(const yarp::sig::Vector &xs)
{
    dmp::DMP* ys_i=NULL;
    if (dmpType =="DMPPointToPoint")
    {
        ys_i=new DMPPointToPoint(dof_, N_, 0.0, 0.0, 0.0);
        ys_i->set_DMP_constants(this->dmpConstants);
    }
    else if(dmpType== "DMPPeriodic") 
    {
            ys_i=new DMPPeriodic(dof_, N_, 0.0, 0.0);
            ys_i->set_DMP_constants(this->dmpConstants);  
    }
    else
            return ys_i;
       
    
    yarp::sig::Vector weightsPred(0);
    for (std::vector<iCub::learningmachine::GPRLearner>::iterator gpIt=gprs.begin(); gpIt!=gprs.end(); ++gpIt)
    {
        iCub::learningmachine::Prediction pred =gpIt->predict(xs);
        weightsPred= yarp::math::cat(weightsPred, pred.getPrediction());      
    }
    
    ys_i->set_DMP_variables(weightsPred);

    return ys_i;
}

DmpGPR::DmpGPR(int N, int dof, int m, double sf2) : gprs((N+1)*dof+1, iCub::learningmachine::GPRLearner(m, 1, sf2))
{
    
    dof_ = dof;
    N_=N;
    m_=m;
    verbose=true;
    init=false;
    sampleCount=0; 
}

DmpGPR::~DmpGPR()
{}


void DmpGPR::print()
{
for(std::vector<iCub::learningmachine::GPRLearner>::iterator gpIt=gprs.begin();gpIt<gprs.end(); ++gpIt) 
    {
        std::cout <<gpIt->toString()<<std::endl;
    }    
}

int DmpGPR::getNumberOfSamples()
{
    return sampleCount;
}