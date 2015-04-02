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
#ifndef LM_ABSTRACT_MEAN__
#define LM_ABSTRACT_MEAN__

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
namespace iCub {
namespace learningmachine {

class Mean
{
public:
    Mean();
    virtual ~Mean();
    virtual yarp::sig::Vector calculateMean(const yarp::sig::Matrix &x)=0;
    virtual yarp::sig::Vector calculateMeanDerivative(const yarp::sig::Matrix &x, size_t i)=0;
    virtual bool setHyperparameters(const yarp::sig::Vector newHyperPar)=0;
    virtual yarp::sig::Vector getHyperparameters()=0;
    virtual int getNumberOfHyperparameters()=0;
    virtual Mean* clone()=0;
    
};
} // learningmachine
} // iCub
#endif
