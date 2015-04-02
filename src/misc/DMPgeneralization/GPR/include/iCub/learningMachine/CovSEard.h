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
#ifndef LM_COVARIANCE_COVSEARD__
#define LM_COVARIANCE_COVSEARD__

#include <string>
#include <vector>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

#include "iCub/learningMachine/Covariance.h"
namespace iCub {
namespace learningmachine {

class CovSEard :public Covariance
{
     /**
     * Covariance hyperparameters (length scale).
     */
    yarp::sig::Vector ell;
    
     /**
     * Signal variance.
     */
    double sigmaSignal;
     /**
     * Domain size
     */
    size_t domain;
    
public:
    CovSEard(unsigned int domain, double lengthScale=1.0, double sigmaSignal=1.0)
    {
        this->domain=domain;
        this->ell.resize(domain, lengthScale);
        this->sigmaSignal=sigmaSignal;
        
    };
    ~CovSEard(){};
    
    CovSEard(CovSEard& original); //copy constructor
    CovSEard& operator= (const CovSEard& original); //Assignment operator
    CovSEard* clone();
    
     /**
     * Calculate covariance matrix evaluated on inputs using a squared exponential kernel.
     * Matrix is n-by-n, with n=x.rows() is the number of datapoints used for training
     *
     * @param x input n-by-D matrix, where n is the number of datapoints and D is their dimension
     */
    yarp::sig::Matrix calculateCovariance(const yarp::sig::Matrix &x);
        
     /**
     * Calculate cross-covariance matrix between inputs x and test points xs, using a squared exponential kernel.
     * Matrix is m-by-n, with  m=xs.rows() is the number of test points, and n=x.rows() is the number of datapoints used for training,
     *
     * @param x input n-by-D matrix, where n is the number of datapoints and D is their dimension
     * @param xs input m-by-D matrix, where m is the number of test points and D is their dimension
     */
    yarp::sig::Matrix calculateCrossCovariance(const yarp::sig::Matrix &x, const yarp::sig::Matrix &xs);
    
    yarp::sig::Matrix getDerivative(const yarp::sig::Matrix &x, size_t i); //NIY //return matrix or vector?
    
     /**
     * Sets the "prior" signal lengthscales \ell to  specified values. This resets the
     * machine.
     *
     * @param newEll the desired value.
     */
    void setLengthScale(yarp::sig::Vector newEll);

    /**
     * Accessor for the signal lengthscales \ell.
     *
     * @returns the value of the parameter
     */
    yarp::sig::Vector getLengthScale();
    
         /**
     * Sets the "prior" signal variance \sigmaSignal to a specified value. This resets the
     * machine.
     *
     * @param s the desired value.
     */
    void setSigmaSignal(double s);

    /**
     * Accessor for the signal variance \sigmaSignal.
     *
     * @returns the value of the parameter
     */
    double getSigmaSignal();
    
    virtual bool setHyperparameters(const yarp::sig::Vector newHyperPar);
    virtual yarp::sig::Vector getHyperparameters();
    virtual int getNumberOfHyperparameters();
    
    
};
} // learningmachine
} // iCub
#endif
