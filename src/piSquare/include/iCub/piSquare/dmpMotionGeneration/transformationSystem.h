/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************/

#ifndef TRANSFORMATION_SYSTEM_H_
#define TRANSFORMATION_SYSTEM_H_

#include <string>
#include <vector>
#include <algorithm>

#define EIGEN2_SUPPORT
#include <Eigen/Eigen>

#include <iCub/piSquare/locallyWeightedRegression/lwr.h>
#include <iCub/piSquare/locallyWeightedRegression/lwrParameters.h>
#include <iCub/piSquare/dmpMotionGeneration/dynamicMovementPrimitive.h>

namespace dmp
{

/**
* \ingroup piSquare
*
* Class representing the second order dynamical system used to represent
* the "time" behaviour of a DMP.
*/

class TransformationSystem
{

public:

    friend class DynamicMovementPrimitive;

	/**
	* class constructor
	*/
    TransformationSystem();

	/**
	* class destructor
	*/
    ~TransformationSystem();

	/**
	* initializes a transformation system, identified by its id, starting from a Parameters object
	* containing the parameters for the Locally Weighted Regression (lwr). Such parameters are
	* passed to a LocallyWeightedRegression object contained into the transformation system. This
	* object contains both the theta parameters used by PI^2 algorithm and the basis functions
	* parameters (i.e. mean and variance).
	* @param dmpId is the dmp identifier
	* @param transId is the transformation system identifier
	* @param lwrParams is the Parameters object containing parameters for lwr
	* @return true if initialization is successful, false otherwise
	*/
    bool initialize(int transId, lwr::Parameters lwrParams);
	
	/**
	* store the target trajectory for lwr as a .txt file (debug)
	* @param fileName is the file path
	* @return true if operation is successful, false otherwise
	*/
	bool writeTrajectoryTargetToFile(std::string fileName);

private:

    ///flag indicating whether the system is initialized or not
    bool initialized_;

    ///the id of the transformation system
    int transId_;

	///internal states (velocity and acceleration)
    double z_;
    double zd_;

    ///external states (position, velocity and acceleration)
    double y_;
    double yd_;
    double ydd_;

    ///targets used during supervised learning
    double t_;
    double td_;
    double tdd_;

    ///the start state
    double y0_;
    double initialY0_;

    ///goal state
    double goal_;
    double initialGoal_;

    ///current values of the nonlinear function
    double f_;
    double ft_;

    ///internal variables that are used to compute the normalized mean squared error during learning
    double mse_;
    double meanFt_;
    unsigned int numMseDataPoints_;

    ///internal variable that is used to store the target function for LWR
    std::vector<double> trajectoryTarget_;

    ///lwr model used to approximate the nonlinear function
    lwr::LocallyWeightedRegression lwrModel_;

	/**
	* resets to zero both internal and external variables
	*/
    void reset();

	/**
	* sets both internal and external states to desired values
	* @param y is the desired external state
	* @param z is the desired internal state
	* @return true if operation is successful, false otherwise
	*/
    void setState(const double y, const double z);

	/**
	* gets external state
	* @return the external state y
	*/
    double getState() const;

	/**
	* sets start value for external state
	* @param y0 is the external state start position
	*/
    void setStart(const double y0);

	/**
	* sets goal value for external state
	* @param goal is the external state goal position
	*/
    void setGoal(const double goal);

	/**
	* gets the goal value
	* @return the goal value for external state
	*/
    double getGoal() const;

	/**
	* sets the initial start state value for external state (for those cases in
	* which the start state value may change during computation)
	* @param initialY0 is the initial start state value
	*/
    void setInitialStart(const double initialY0);

	/**
	* sets the initial goal state value for external state (for those cases in
	* which the goal state value may change during computation)
	* @param initialGoal is the initial goal state value
	*/
    void setInitialGoal(const double initialGoal);

	/**
	* resets to zero the variables used for storing mse information
	*/
    void resetMSE();

	/**
	* gets the mse value
	* @param mse is the output mse value
	* @return true if operation is successful, false otherwise
	*/
    bool getMSE(double& mse);

	/**
	* gets the normalized mse value
	* @param mse is the output normalized mse value
	* @return true if operation is successful, false otherwise
	*/
    bool getNormalizedMSE(double& normalizedMse);

	/**
	* computes mse between f_ and ft_ functions
	*/
    void computeMSE();

};

// inline function follow
inline void TransformationSystem::reset()
{
    z_ = 0.0;
    zd_ = 0.0;
    y_ = 0.0;
    yd_ = 0.0;
    ydd_ = 0.0;
}

inline void TransformationSystem::setState(const double y, const double z)
{
    y_ = y;
    z_ = z;
}

inline double TransformationSystem::getState() const
{
    return y_;
}

inline void TransformationSystem::setStart(const double y0)
{
    y0_ = y0;
}

inline void TransformationSystem::setGoal(const double goal)
{
    goal_ = goal;
}

inline double TransformationSystem::getGoal() const
{
    return goal_;
}

inline void TransformationSystem::setInitialStart(const double initialY0)
{
    initialY0_ = initialY0;
}

inline void TransformationSystem::setInitialGoal(const double initialGoal)
{
    initialGoal_ = initialGoal;
}

inline void TransformationSystem::resetMSE()
{
    mse_ = 0.0;
    meanFt_ = 0.0;
    numMseDataPoints_ = 0;
}

inline bool TransformationSystem::getMSE(double& mse)
{
    if (numMseDataPoints_ == 0)
    {
        printf("ERROR: No data point seen yet, cannot compute mean squared error.\n");
        return false;
    }
    mse = mse_ / static_cast<double> (numMseDataPoints_);
    return true;
}

inline bool TransformationSystem::getNormalizedMSE(double& normalizedMse)
{
    if (numMseDataPoints_ == 0)
    {
        printf("ERROR: No data point seen yet, cannot compute normalized mean squared error.\n");
        return false;
    }

    normalizedMse = (static_cast<double> (1.0) / std::max(pow(meanFt_, 2), 1.0)) * (mse_ / static_cast<double> (numMseDataPoints_));
    return true;
}

inline void TransformationSystem::computeMSE()
{
    mse_ += pow(ft_ - f_, 2);
    meanFt_ = (static_cast<double> (numMseDataPoints_) * meanFt_ + ft_) / static_cast<double> (numMseDataPoints_ + 1);
    numMseDataPoints_++;
}

}

#endif /* TRANSFORMATION_SYSTEM_H_ */
