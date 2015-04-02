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
#ifndef LM_LINEAR_MEAN__
#define LM_LINEAR_MEAN__

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

#include "iCub/learningMachine/Mean.h"

namespace iCub {
namespace learningmachine {

class LinearMean : public Mean
{
     /**
     * Mean hyperparameters.
     */
    yarp::sig::Vector meanLinearHyperparam;
    
     /**
     * Mean offset.
     */
     double meanOffset;
     
     /**
     * Domain size
     */
    size_t domain;
    
public:
    LinearMean(unsigned int domain, double linHyperpar=0.0, double offset=0.0)
    {
        this->domain=domain;
        meanLinearHyperparam.resize(domain, linHyperpar);
        this->meanOffset=offset;
    };
    ~LinearMean(){};
    
    LinearMean(LinearMean& original); //copy constructor
    LinearMean& operator= (const LinearMean& original); //Assignment operator
    LinearMean* clone();
    
     yarp::sig::Vector calculateMean(const yarp::sig::Matrix &x);
     yarp::sig::Vector calculateMeanDerivative(const yarp::sig::Matrix &x, size_t i);
     
      /**
     * Sets the "prior" mean offset \meanOffset to a specified value. This resets the
     * machine.
     *
     * @param s the desired value.
     */
    void setMeanOffset(double m);

    /**
     * Accessor for the mean offset \meanOffset.
     *
     * @returns the value of the parameter
     */
    double getMeanOffset();
    

    
     /**
     * Sets the hyperparameters for "mean" process computation \meanLinearHyperparam to  specified values. This resets the
     * machine.
     *
     * @param newMeanHyp the desired value.
     */
    void setLinearHyperparameters(yarp::sig::Vector newMeanHyp);

    /**
     * Accessor for the hyperparameters for "mean" process computation \meanLinearHyperparam.
     *
     * @returns the value of the parameter
     */
    yarp::sig::Vector getLinearHyperparameters();
    
    virtual bool setHyperparameters(const yarp::sig::Vector newHyperPar);
    virtual yarp::sig::Vector getHyperparameters();
    virtual int getNumberOfHyperparameters();
    
};
} // learningmachine
} // iCub
#endif
