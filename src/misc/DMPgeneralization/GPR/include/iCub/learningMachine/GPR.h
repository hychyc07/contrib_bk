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
#ifndef LM_GENERICGPRLEARNER__
#define LM_GENERICGPRLEARNER__

#include <string>
#include <vector>

#include <yarp/sig/Matrix.h>

#include "iCub/learningMachine/IFixedSizeLearner.h"
#include "iCub/learningMachine/Covariance.h"
#include "iCub/learningMachine/Mean.h"

namespace iCub {
namespace learningmachine {
    
/**
* Learner based on Gaussian Process Regression
*/
class GPRLearner : public IFixedSizeLearner {
private:
    /**
     * Matrix B.
     */
    yarp::sig::Matrix B;

    /**
     * Weight matrix for the linear predictor.
     */
    yarp::sig::Matrix W;

    /**
     * Signal noise.
     */
    double sigma;
        

    Covariance* covFunction;
    Mean* meanFunction;
    yarp::sig::Matrix trainingInputs; //need to store them in memory, to calculate cross-covariance with test samples
                                      // each column is an input vector
    yarp::sig::Matrix trainingTargets; //OR use vector<Vector>?


    /**
     * Number of samples during last training routine
     */
    int sampleCount;

public:
    /**
     * Constructor.
     *
     * @param dom initial domain size
     * @param cod initial codomain size
     * @param sigma initial value for signal noise \sigma
     */
   // GPRLearner(unsigned int dom = 1, unsigned int cod = 1, double sigma = 1.0);
    GPRLearner(unsigned int dom = 1, unsigned int cod = 1, double sigma = 1.0);

    /**
     * Copy constructor.
     */
    GPRLearner(const GPRLearner& other);

    /**
     * Destructor.
     */
    virtual ~GPRLearner();

    /**
     * Assignment operator.
     */
    GPRLearner& operator=(const GPRLearner& other);
    
    virtual void setCovarianceFunction(Covariance* newCovFunction);
    virtual void setMeanFunction(Mean* newMeanFunction);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void feedSample(const yarp::sig::Vector& input, const yarp::sig::Vector& output);

      /*
     * Optimize mean, covariance, sigma (noise) hyperparameters
     */
    virtual void optimize(const double tol, const int max_iter, const unsigned int verbose, bool useHessian);
    
    /*
     * Inherited from IMachineLearner.
     */
    virtual void train();

    /*
     * Inherited from IMachineLearner.
     */
    virtual Prediction predict(const yarp::sig::Vector& input);

    /*
     * Inherited from IMachineLearner.
     */
    void reset();

    /*
     * Inherited from IMachineLearner.
     */
    GPRLearner* clone() {
        return new GPRLearner(*this);
    }

    /*
     * Inherited from IMachineLearner.
     */
    virtual std::string getInfo();

    /*
     * Inherited from IMachineLearner.
     */
    virtual std::string getConfigHelp();

    /*
     * Inherited from IMachineLearner.
     */
    virtual void writeBottle(yarp::os::Bottle& bot);

    /*
     * Inherited from IMachineLearner.
     */
    virtual void readBottle(yarp::os::Bottle& bot);

    /*
     * Inherited from IFixedSizeLearner.
     */
    void setDomainSize(unsigned int size);

    /*
     * Inherited from IFixedSizeLearner.
     */
    void setCoDomainSize(unsigned int size);

    /**
     * Sets the signal noise \sigma to a specified value. This resets the
     * machine.
     *
     * @param s the desired value.
     */
    void setSigma(double s);

    /**
     * Accessor for the signal noise \sigma.
     *
     * @returns the value of the parameter
     */
    double getSigma();

    Covariance* getCovariancePtr();
    
    Mean* getMeanPtr();
    /*
     * Inherited from IConfig.
     */
    virtual bool configure(yarp::os::Searchable& config);

};

} // learningmachine
} // iCub
#endif
