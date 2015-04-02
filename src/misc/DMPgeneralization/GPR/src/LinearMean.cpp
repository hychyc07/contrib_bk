/* 
 * Copyright (C) 2012 Istituto Italiano di Tecnologia
 * Author: Elena Ceseracciu
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


#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include "iCub/learningMachine/LinearMean.h"
#include <iostream>
using namespace yarp::sig;
using namespace yarp::math;

namespace iCub {
namespace learningmachine {

LinearMean::LinearMean(LinearMean& original): 
meanLinearHyperparam(original.meanLinearHyperparam), meanOffset(original.meanOffset), domain(original.domain){}//copy constructor

LinearMean& LinearMean::operator= (const LinearMean& original)
{
    if(this == &original) return *this; // handle self initialization

    this->meanLinearHyperparam=original.meanLinearHyperparam;
    this->meanOffset=original.meanOffset;
    this->domain=original.domain;
    return *this;
}

LinearMean* LinearMean::clone()
{
    return new LinearMean(*this);
    
}
    
yarp::sig::Vector LinearMean::calculateMean(const yarp::sig::Matrix &x)
{
  //  std::cout << "calc mean, x size:" << x.rows() << " by " << x.cols() << ", meanHyparam size: " << meanLinearHyperparam.size() << std::endl;// fflush(stdout);
    Vector fmean(x.rows(), meanOffset);
    fmean+=x*meanLinearHyperparam;
    return fmean;
}
yarp::sig::Vector LinearMean::calculateMeanDerivative(const yarp::sig::Matrix &x, size_t i)
{
    if (i>this->domain || i < 0)
        return Vector();
    if (i==this->domain)
        return ones(domain);

    return x.getCol(i);
}


void LinearMean::setMeanOffset(double m)
{
    this->meanOffset=m;
}

double LinearMean::getMeanOffset()
{
    return meanOffset;
}

void LinearMean::setLinearHyperparameters(yarp::sig::Vector newMeanHyp)
{
    this->meanLinearHyperparam=newMeanHyp;
}

yarp::sig::Vector LinearMean::getLinearHyperparameters()
{
    return meanLinearHyperparam;
}

bool LinearMean::setHyperparameters(const yarp::sig::Vector newHyperPar)
{
	if (newHyperPar.size()!=this->getNumberOfHyperparameters())
		return false;
    meanLinearHyperparam=newHyperPar.subVector(0, newHyperPar.size()-2);
    meanOffset=newHyperPar(newHyperPar.size()-1);
    return true;
}
yarp::sig::Vector LinearMean::getHyperparameters()
{
    return cat(meanLinearHyperparam, meanOffset);
}
    
int LinearMean::getNumberOfHyperparameters()
{
    return domain+1;
}
        
        

} // learningmachine
} // iCub
