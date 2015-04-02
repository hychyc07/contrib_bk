/* 
 * Copyright (C) 2011 Jožef Stefan Institute, Robotics Brain and Cognitive Sciences, Istituto Italiano di Tecnologia (GSL version)
 * Author: Ales Ude (JSI), Andrej Gams (JSI), Elena Ceseracciu (IIT)
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/
#include <cassert>
#include <stdexcept>
#include <cmath>

#include <iostream>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include "iCub/learningMachine/RLSLearner.h"
#include "iCub/learningMachine/Math.h"
#include "iCub/learningMachine/Serialization.h"

using namespace yarp::math;
using namespace yarp::sig;
using namespace iCub::learningmachine::serialization;
using namespace iCub::learningmachine::math;

#include "iCub/learningMachine/GPR.h"
#include "iCub/learningMachine/CovSEard.h"
#include "iCub/learningMachine/LinearMean.h"


namespace iCub {
namespace learningmachine {
        
GPRLearner::GPRLearner(unsigned int dom, unsigned int cod, double sigma)
{
    this->setName("GPR");
    this->sampleCount = 0;
    // make sure to not use initialization list to constructor of base for
    // domain and codomain size, as it will not use overloaded mutators
    this->setDomainSize(dom);
    // slightly inefficient to use mutators, as we are initializing multiple times
    this->setCoDomainSize(cod);
    this->setSigma(sigma);
    this->covFunction=new CovSEard(dom); //default covariance
    this->meanFunction=new LinearMean(dom); //default mean
}


GPRLearner::GPRLearner(const GPRLearner& other)
  : IFixedSizeLearner(other), sampleCount(other.sampleCount), trainingInputs(other.trainingInputs),
    trainingTargets(other.trainingTargets), B(other.B), W(other.W), sigma(other.sigma), covFunction(other.covFunction->clone()), meanFunction(other.meanFunction->clone()){
}

GPRLearner::~GPRLearner() {
    if (covFunction!=NULL)
        delete covFunction;
    if (meanFunction!=NULL)
        delete meanFunction;
        
}

GPRLearner& GPRLearner::operator=(const GPRLearner& other) {
    if(this == &other) return *this; // handle self initialization

    this->IFixedSizeLearner::operator=(other);
    this->sampleCount = other.sampleCount;

     this->B = other.B;
    this->trainingInputs=other.trainingInputs;
    this->trainingTargets=other.trainingTargets;
    this->W = other.W;
    this->sigma = other.sigma;

    return *this;
}

Mean* GPRLearner::getMeanPtr()
{
    return this->meanFunction;
}
 
Covariance* GPRLearner::getCovariancePtr()
{
    return this->covFunction;
}

void GPRLearner::feedSample(const yarp::sig::Vector& input, const yarp::sig::Vector& output) 
{
    this->IFixedSizeLearner::feedSample(input, output);
    if (trainingInputs.rows()==0)
    {
        trainingInputs.resize(1, this->getDomainSize());
        trainingInputs.setRow(0, input);
    }
    else
        trainingInputs= yarp::math::pile(trainingInputs, input);
    if (trainingTargets.rows()==0)
    {
        trainingTargets.resize(1, this->getCoDomainSize());
        trainingTargets.setRow(0, output);
    }
    else
    {
        trainingTargets = yarp::math::pile(trainingTargets, output);
    }
}

void GPRLearner::train() 
{
    // calculate covariance K
    Matrix K=covFunction->calculateCovariance(trainingInputs)/sigma;
    //add sigma*I (add I because we pre-divided K)
    K+=eye(K.rows());
    //PseudoInverse
    
    Matrix Kinv=pinv(K);
    
    
    Vector m=meanFunction->calculateMean(trainingInputs);
  
    this->B.resize(trainingTargets.rows(), this->getCoDomainSize());
    for (size_t dimCt=0; dimCt<this->getCoDomainSize(); dimCt++) 
        B.setCol(dimCt, trainingTargets.getCol(dimCt) - m); //maybe can extend to coDomain size? subtract m from each row of trainingTargets...
    //W=(Kinv*b);
    W=Kinv/sigma; //keep only Kinv, as we need it to calculate variance of prediction
    
}
// void GPRLearner::optimize()
// {
//     
// }

Prediction GPRLearner::predict(const yarp::sig::Vector& input) {
    if (!this->checkDomainSize(input))
        return Prediction();

    yarp::sig::Matrix inputAsMatrix(1, this->getDomainSize());   // is there a better way to
    inputAsMatrix.setRow(0, input);                     // view a vector as a matrix?
    
    Matrix Kstar=covFunction->calculateCrossCovariance(trainingInputs, inputAsMatrix);
    
    Vector mean_s=meanFunction->calculateMean(inputAsMatrix);
    
    Matrix outputAsMatrix=Kstar*this->W*this->B;
    for (size_t dimCt=0; dimCt<coDomainSize; dimCt++)
        outputAsMatrix.setRow(dimCt, outputAsMatrix.getCol(dimCt)+mean_s); //add mean to each row (however, since we have a single input vector, we are adding only to it)
    
    
    Matrix selfcovariance=covFunction->calculateCovariance(inputAsMatrix);
    
    Matrix std=selfcovariance+Kstar*this->W*Kstar.transposed();
    Vector stdVector(outputAsMatrix.cols(), std(0,0));
    
    return Prediction(outputAsMatrix.getRow(0), stdVector);
}

void GPRLearner::reset() {
    this->sampleCount = 0;

    //this->B = zeros(this->getCoDomainSize(), this->getDomainSize()); //USELESS
    //this->W = zeros(this->getCoDomainSize(), this->getDomainSize()); //USELESS
    this->trainingInputs.resize(0, this->getDomainSize());
    this->trainingTargets.resize(0,this->getCoDomainSize());
}

std::string GPRLearner::getInfo() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getInfo();
    buffer << "Sigma: " << this->getSigma() << " | ";
    buffer << "Sample Count: " << this->sampleCount << std::endl;
    //for(unsigned int i = 0; i < this->machines.size(); i++) {
    //    buffer << "  [" << (i + 1) << "] ";
    //    buffer << "lambda: " << this->machines[i]->getLambda();
    //    buffer << std::endl;
    //}
    return buffer.str();
}

std::string GPRLearner::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getConfigHelp();
    buffer << "  sigma val             Signal noise sigma" << std::endl;
    return buffer.str();
}

void GPRLearner::writeBottle(yarp::os::Bottle& bot) {
    bot << /*this->R << */this->B << this->W << this->sigma << this->sampleCount;
    // make sure to call the superclass's method
    this->IFixedSizeLearner::writeBottle(bot);
}

void GPRLearner::readBottle(yarp::os::Bottle& bot) {
    // make sure to call the superclass's method
    this->IFixedSizeLearner::readBottle(bot);
    bot >> this->sampleCount >> this->sigma >> this->W >> this->B /*>> this->R*/ ;
}

void GPRLearner::setDomainSize(unsigned int size) {
    this->IFixedSizeLearner::setDomainSize(size);
    this->reset();
}

void GPRLearner::setCoDomainSize(unsigned int size) {
    this->IFixedSizeLearner::setCoDomainSize(size);
    this->reset();
}

void GPRLearner::setSigma(double s) {
    if(s > 0.0) {
        this->sigma = s;
        this->reset();
    } else{
        throw std::runtime_error("Signal noise sigma has to be larger than 0");
    }
}

double GPRLearner::getSigma() {
    return this->sigma;
}


bool GPRLearner::configure(yarp::os::Searchable& config) {
    bool success = this->IFixedSizeLearner::configure(config);

    // format: set sigma val
    if(config.find("sigma").isDouble() || config.find("sigma").isInt()) {
        this->setSigma(config.find("sigma").asDouble());
        success = true;
    }

    return success;
}

void GPRLearner::setMeanFunction(Mean* newMeanFunction)
{
    if (newMeanFunction==this->meanFunction)
        return;
    if(this->meanFunction!=NULL)
        delete this->meanFunction;
    meanFunction=newMeanFunction->clone();
}
void GPRLearner::setCovarianceFunction(Covariance* newCovFunction)
{
        if (newCovFunction==this->covFunction)
        return;
    if(this->covFunction!=NULL)
        delete this->covFunction;
    covFunction=newCovFunction->clone();
    
}

} // learningmachine
} // iCub
