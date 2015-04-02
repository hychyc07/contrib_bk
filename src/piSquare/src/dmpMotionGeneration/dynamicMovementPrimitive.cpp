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

#include <math.h>
#include <sstream>
#include <errno.h>

#include <boost/foreach.hpp>

#include <iCub/piSquare/dmpMotionGeneration/dynamicMovementPrimitive.h>
#include <iCub/piSquare/dmpMotionGeneration/constants.h>
#include <iCub/piSquare/dmpMotionGeneration/mathHelper.h>


using namespace Eigen;
using namespace yarp::os;

namespace dmp
{

static const int DMP_ID_OFFSET = 10000;
static const double NOT_ASSIGNED = -666.666;

DynamicMovementPrimitive::DynamicMovementPrimitive(ResourceFinder* rf) :
    initialized_(false), rf_(rf), params_(rf)
{
}

DynamicMovementPrimitive::~DynamicMovementPrimitive()
{
}

bool DynamicMovementPrimitive::initialize(int numTransformationSystems)
{
    Parameters dmpParams(rf_);
    if (!dmpParams.initialize())
    {
        printf("ERROR: Could not initialize dmp parameters from resource finder\n");
        initialized_ = false;
        return initialized_;
    }

    lwr::Parameters lwrParams(rf_);
    if (!lwrParams.initialize())
    {
        printf("ERROR: Could not initialize lwr parameters from resource finder\n");
        initialized_ = false;
        return initialized_;
    }
    return initialize(numTransformationSystems, dmpParams, lwrParams);
}

bool DynamicMovementPrimitive::initialize(int numTransformationSystems, Parameters dmpParams, lwr::Parameters lwrParams)
{
    //assign dmp parameters
    params_ = dmpParams;

    //overwrite number of transformation system in dmp_params
    if (numTransformationSystems <= 0)
    {
        printf("ERROR: Number of transformation system %i is not valid\n", numTransformationSystems);
        initialized_ = false;
        return initialized_;
    }
    params_.numTransformationSystems_ = numTransformationSystems;

    //initialized transformation systems using the lwr parameters
    transformationSystems_.resize(params_.numTransformationSystems_);
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        if (!transformationSystems_[i].initialize(i, lwrParams))
        {
            printf("ERROR: Could not initialize transformation system  %i.\n", i);
            initialized_ = false;
            return initialized_;
        }
    }

    // set canonical system to pre-defined state
    resetCanonicalState();

    // allocate some memory (usefull?)
    initialize();

    initialized_ = true;
    return initialized_;
}

void DynamicMovementPrimitive::initialize()
{
	//debugTrajectoryPoint is probably meaningless

    trajectoryTargetFunctionInput_.clear();
    debugTrajectoryPoint_
            = VectorXd::Zero(NUM_DEBUG_CANONICAL_SYSTEM_VALUES + (params_.numTransformationSystems_ * NUM_DEBUG_TRANSFORMATION_SYSTEM_VALUES));
}

bool DynamicMovementPrimitive::reInitializeParams()
{
    return params_.initialize();
}

bool DynamicMovementPrimitive::learnFromThetas(const std::vector<VectorXd>& thetas, const VectorXd &initialStart, const VectorXd &initialGoal,
                                               const double samplingFrequency, const double initialDuration)
{
    if (!initialized_)
    {
        printf("ERROR: DMP is not initialized.\n");
        params_.isLearned_ = false;
        return params_.isLearned_;
    }

    ResourceFinder rf;

    //set y0 to start state of trajectory and set goal to end of the trajectory
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        // set initial start and initial goal
        transformationSystems_[i].setInitialStart(initialStart(i));
        transformationSystems_[i].setInitialGoal(initialGoal(i));
    }

    params_.alphaX_ = -log(params_.canSysCutoff_);
    params_.teachingDuration_ = initialDuration;

    params_.deltaT_ = static_cast<double> (1.0) / samplingFrequency;
    params_.initialDeltaT_ = params_.deltaT_;

    params_.tau_ = params_.teachingDuration_;
    params_.initialTau_ = params_.tau_;

    if (!setThetas(thetas))
    {
        printf("ERROR: Could not set theta parameters.\n");
        params_.isLearned_ = false;
        return params_.isLearned_;
    }

    params_.isLearned_ = true;

    return params_.isLearned_;
}

bool DynamicMovementPrimitive::learnFromMinJerk(const yarp::sig::Vector &start, const yarp::sig::Vector &goal, const double duration, const double deltaT)
{
    if (!initialized_)
    {
        printf("ERROR: DMP motion unit is not initialized, not learning from minimum jerk trajectory.\n");
        params_.isLearned_ = false;
        return params_.isLearned_;
    }

    double samplingFrequency = static_cast<double> (1.0) / deltaT;

	//could be removed
    std::vector<std::string> variableNames;
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        std::stringstream ss;
        ss << i;
        variableNames.push_back(std::string("dummy_") + ss.str());
    }

    dmp::Trajectory minJerkTrajectory(rf_);
    if (!minJerkTrajectory.initialize(variableNames, samplingFrequency))
    {
        printf("ERROR: Could not initialize trajectory.\n");
        params_.isLearned_ = false;

        return params_.isLearned_;
    }

    if (!MathHelper::generateMinJerkTrajectory(start, goal, duration, deltaT, minJerkTrajectory))
    {
         printf("ERROR: Could not generate minimum jerk trajectory.\n");
        params_.isLearned_ = false;

        return params_.isLearned_;
    }
    if (!learnFromTrajectory(minJerkTrajectory))
    {
        printf("ERROR: Could not learn from minimum jerk trajectory.\n");
        params_.isLearned_ = false;

        return params_.isLearned_;
    }

	//just for debug
	/*
	std::vector<Eigen::VectorXd> thetas;
	getThetas(thetas);

	
	printf("THETAS: ");
	std::vector<Eigen::VectorXd>::iterator iter = thetas.begin();
	for(; iter != thetas.end(); ++iter)
	{
		for(int i = 0; i < iter->size(); i++)
		{
			Eigen::VectorXd v = *iter;
			printf("%f ", v(i));
		}
		printf("\n\n");
	}
	printf("\n");
	*/
	
    return (params_.isLearned_ = true);
}

bool DynamicMovementPrimitive::learnFromCustomTrajectory(dmp::Trajectory& trajectory)
{
    if (!initialized_)
    {
        printf("ERROR: DMP motion unit is not initialized, not learning from custom trajectory.\n");
        params_.isLearned_ = false;
        return params_.isLearned_;
    }

    if (!learnFromTrajectory(trajectory))
    {
        printf("ERROR: Could not learn from custom trajectory.\n");
        params_.isLearned_ = false;

        return params_.isLearned_;
    }
	
    return (params_.isLearned_ = true);
}

bool DynamicMovementPrimitive::learnFromTrajectory(const Trajectory &trajectory)
{

    if (!initialized_)
    {
        printf("ERROR: DMP motion unit is not initialized, not learning from trajectory.\n");
        params_.isLearned_ = false;
        return params_.isLearned_;
    }

    int numRows = trajectory.getLength();
    if (numRows < MIN_NUM_DATA_POINTS)
    {
        printf("ERROR: Trajectory has %i rows, but should have at least %i.\n", numRows, MIN_NUM_DATA_POINTS);
        params_.isLearned_ = false;
        return params_.isLearned_;
    }

    double samplingFrequency = trajectory.getSamplingFrequency();
    if (samplingFrequency <= 0)
    {
        printf("ERROR: Sampling frequency %f [Hz] of the trajectory is not valid.\n",samplingFrequency);
        params_.isLearned_ = false;
        return params_.isLearned_;
    }

    //set teaching duration to the duration of the trajectory
    params_.teachingDuration_ = static_cast<double> (numRows) / static_cast<double> (samplingFrequency);

    params_.deltaT_ = static_cast<double> (1.0) / samplingFrequency;
    params_.initialDeltaT_ = params_.deltaT_;
    params_.tau_ = params_.teachingDuration_;
    params_.initialTau_ = params_.tau_;

    //compute alpha_x such that the canonical system drops below the cutoff when the trajectory has finished
	//alpha_x is the time constant for the phase system (second order asimptotically stable system)
    params_.alphaX_ = -log(params_.canSysCutoff_);

    double mseTotal = 0.0;
    double normalizedMseTotal = 0.0;
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        transformationSystems_[i].trajectoryTarget_.clear();
        transformationSystems_[i].resetMSE();
    }
    trajectoryTargetFunctionInput_.clear();

    //reset canonical system
    resetCanonicalState();

    //obtain start and goal position
    VectorXd start = VectorXd::Zero(params_.numTransformationSystems_);
    if (!trajectory.getStartPosition(start))
    {
        printf("ERROR: Could not get the start position of the trajectory\n");
        params_.isLearned_ = false;
        return params_.isLearned_;
    }
    VectorXd goal = VectorXd::Zero(params_.numTransformationSystems_);
    if (!trajectory.getEndPosition(goal))
    {
        printf("ERROR: Could not get the goal position of the trajectory\n");
        params_.isLearned_ = false;
        return params_.isLearned_;
    }

    //set y0 to start state of trajectory and set goal to end of the trajectory
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        //check whether all this is necessary (I don't think so...)
        transformationSystems_[i].reset();

        //set start and goal
        transformationSystems_[i].setStart(start(i));
        transformationSystems_[i].setGoal(goal(i));

        //set current state to start state (position and velocity)
        transformationSystems_[i].setState(start(i), 0.0);
    }

    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        transformationSystems_[i].setInitialStart(transformationSystems_[i].y0_);
        transformationSystems_[i].setInitialGoal(transformationSystems_[i].goal_);
    }

	//for each time step and for each dimension, perform supervised learning of the input trajectory
	//Actually is not a "classical" learning problem...here the problem is how to encode the
	//target trajectory in the dmp by representing it as a second order system modulated with
	//a nonlinear function f.
    for (int rowIndex = 0; rowIndex < numRows; rowIndex++)
    {
        //set transformation target:
		//t_, td_ and tdd_ represent the current position, velocity and acceleration we want
		//to learn throught supervised learning. f_ represents the current values of
		//the nonlinear function used to modulate the dmp behaviour, while ft_ is the target
		//value for such nonlinear function.
		//NOTE: is f_ actually used anywhere?????
        for (int i = 0; i < params_.numTransformationSystems_; i++)
        {
            transformationSystems_[i].t_ = trajectory.getTrajectoryPosition(rowIndex, i);
            transformationSystems_[i].td_ = trajectory.getTrajectoryVelocity(rowIndex, i);
            transformationSystems_[i].tdd_ = trajectory.getTrajectoryAcceleration(rowIndex, i);
            transformationSystems_[i].f_ = 0.0;
            transformationSystems_[i].ft_ = 0.0;
        }

        //fit the target function:
		//it computes the ideal value of f_ (i.e. ft_) which allows to exactly reproduce
		//the trajectory with the dmp
        if (!integrateAndFit())
        {
            printf("ERROR: Could not integrate system and fit the target function\n");
            params_.isLearned_ = false;
            return params_.isLearned_;
        }
    }

	if(!writeVectorToFile(trajectoryTargetFunctionInput_, "data/trajectory_target_function_input_.txt")) return false;
	
	if(!transformationSystems_[0].writeTrajectoryTargetToFile("data/trajectory_target_.txt")) return false;

    if (!learnTransformationTarget())
    {
        printf("ERROR: Could not learn transformation target.\n");
        params_.isLearned_ = false;
        return params_.isLearned_;
    }

    mseTotal = 0.0;
    normalizedMseTotal = 0.0;
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        double mse;
        double normalizedMse;
        if (transformationSystems_[i].getMSE(mse))
        {
            mseTotal += mse;
        }
        if (transformationSystems_[i].getNormalizedMSE(normalizedMse))
        {
            normalizedMseTotal += normalizedMse;
        }
        transformationSystems_[i].resetMSE();
    }

    printf("Successfully learned DMP from trajectory.\n");
    params_.isLearned_ = true;

    return params_.isLearned_;
}

bool DynamicMovementPrimitive::getInitialStart(VectorXd &initialStart)
{
    if (initialStart.size() != params_.numTransformationSystems_)
    {
        return false;
    }
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        initialStart(i) = transformationSystems_[i].initialY0_;
    }

    return true;
}

bool DynamicMovementPrimitive::getInitialGoal(VectorXd &initialGoal)
{
    if (initialGoal.size() != params_.numTransformationSystems_)
    {
        return false;
    }
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        initialGoal(i) = transformationSystems_[i].initialGoal_;
    }

    return true;
}

bool DynamicMovementPrimitive::getGoal(VectorXd &goal)
{
    if (goal.size() != params_.numTransformationSystems_)
    {
        return false;
    }
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        goal(i) = transformationSystems_[i].goal_;
    }

    return true;
}

bool DynamicMovementPrimitive::setup(const double samplingFrequency)
{
	VectorXd start = VectorXd::Zero(params_.numTransformationSystems_);
    VectorXd goal = VectorXd::Zero(params_.numTransformationSystems_);
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        start(i) = transformationSystems_[i].initialY0_;
        goal(i) = transformationSystems_[i].initialGoal_;
    }
    return setup(start, goal, params_.initialTau_, samplingFrequency);
}

bool DynamicMovementPrimitive::setup(const VectorXd &goal, const double movementDuration, const double samplingFrequency)
{
    VectorXd start = VectorXd(params_.numTransformationSystems_);
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        start(i) = transformationSystems_[i].initialY0_;
    }
    if (!setup(start, goal, movementDuration, samplingFrequency))
    {
        params_.isSetup_ = false;
        return params_.isSetup_;
    }

    //start has not been specified, need to be set before the DMP can be propagated
    params_.isStartSet_ = false;
	params_.isSetup_ = true;

    return params_.isSetup_;
}

bool DynamicMovementPrimitive::setup(const VectorXd &start, const VectorXd &goal, const double movementDuration, const double samplingFrequency)
{
    if (!initialized_)
    {
        printf("ERROR: DMP unit is not initialized.\n");
        params_.isSetup_ = false;
        return params_.isSetup_;
    }

    if (!params_.isLearned_)
    {
        printf("ERROR: DMP unit is not learned.\n");
        params_.isSetup_ = false;
        return params_.isSetup_;
    }

    if (start.size() != params_.numTransformationSystems_)
    {
        printf("ERROR: Cannot set start of the DMP, the size is %i, but should be %i.\n", start.size(), params_.numTransformationSystems_);
        params_.isSetup_ = false;
        return params_.isSetup_;
    }

    if (goal.size() != params_.numTransformationSystems_)
    {
        printf("ERROR: Cannot set goal of the DMP, the size is %i, but should be %i.\n", goal.size(), params_.numTransformationSystems_);
        params_.isSetup_ = false;
        return params_.isSetup_;
    }

    //reset canonical system
    resetCanonicalState();

    if (movementDuration > 0)
    {
        params_.tau_ = movementDuration;

        if (samplingFrequency <= 0)
        {
            printf("ERROR: Sampling frequency %f [Hz] of the trajectory is not valid.\n", samplingFrequency);
            params_.isSetup_ = false;
            return params_.isSetup_;
        }
        params_.deltaT_ = static_cast<double> (1.0) / static_cast<double> (samplingFrequency);
    }
    else
    {
        params_.tau_ = params_.initialTau_;
        params_.deltaT_ = params_.initialDeltaT_;
    }

    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        //set internal variables to zero
        transformationSystems_[i].reset();

        // set start and goal
        transformationSystems_[i].setStart(start(i));
        transformationSystems_[i].setGoal(goal(i));

        //set current state to start state (position and velocity)
        transformationSystems_[i].setState(start(i), 0.0);
    }

    params_.numSamples_ = 0;
    params_.isStartSet_ = true;
    params_.isSetup_ = true;

    return params_.isSetup_;
}


bool DynamicMovementPrimitive::getCurrentPosition(VectorXd &currentDesiredPosition, const int startIndex, const int endIndex)
{
    if ((!params_.isSetup_) || (startIndex < 0) || (endIndex > params_.numTransformationSystems_) || (endIndex <= startIndex))
    {
        return false;
    }
    if (currentDesiredPosition.size() != endIndex - startIndex)
    {
        printf("ERROR: Provided vector has wrong size (%i), required size is (%i). Cannot get current position.\n", currentDesiredPosition.size(), endIndex-startIndex);
        return false;
    }
    for (int i = startIndex; i < endIndex; i++)
    {
        currentDesiredPosition(i) = transformationSystems_[i].y_;
    }

    return true;
}

bool DynamicMovementPrimitive::changeGoal(const VectorXd &goal, const int startIndex, const int endIndex)
{
    if ((!params_.isSetup_) || (startIndex < 0) || (endIndex > params_.numTransformationSystems_) || (endIndex <= startIndex))
    {
        return false;
    }
    if (goal.size() != endIndex - startIndex)
    {
        printf("ERROR: Provided vector has wrong size (%i), required size is (%i). Cannot change goal position.\n", goal.size(), endIndex-startIndex);
        return false;
    }
    for (int i = startIndex; i < endIndex; i++)
    {
        transformationSystems_[i].setGoal(goal(i - startIndex));
    }

    return true;
}

bool DynamicMovementPrimitive::changeGoal(const double newGoal, const int index)
{
    if ((!params_.isSetup_) || (index < 0) || (index > params_.numTransformationSystems_))
    {
        return false;
    }
    transformationSystems_[index].setGoal(newGoal);

    return true;
}

bool DynamicMovementPrimitive::changeStart(const VectorXd &start)
{
    if (!params_.isSetup_)
    {
        printf("ERROR: DMP is not setup\n");
        return false;
    }
    if (start.size() != params_.numTransformationSystems_)
    {
        printf("ERROR: Start vector has wrong size (%i), it should be %i.\n", start.size(), params_.numTransformationSystems_);
        return false;
    }
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        transformationSystems_[i].setStart(start(i));

        // set current state to start state (position and velocity)
        transformationSystems_[i].setState(start(i), 0.0);
    }
    params_.isStartSet_ = true;

    return true;
}

bool DynamicMovementPrimitive::getThetas(std::vector<VectorXd>& thetas)
{
	if(!initialized_) return false;

    thetas.clear();
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        int numRfs;
        if (!transformationSystems_[i].lwrModel_.getNumRFS(numRfs))
        {
            printf("ERROR: Could not get number of receptive fields.\n");
            return false;
        }

        VectorXd thetaVector = VectorXd::Zero(numRfs);
        if (!transformationSystems_[i].lwrModel_.getThetas(thetaVector))
        {
            printf("ERROR: Could not retrieve thetas from transformation system %i.\n",i);
            return false;
        }
        thetas.push_back(thetaVector);
    }
    return true;
}

bool DynamicMovementPrimitive::setThetas(const std::vector<VectorXd>& thetas)
{
    if(!initialized_) return false;
    if(!static_cast<int>(thetas.size()) == params_.numTransformationSystems_) return false;
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        if (!transformationSystems_[i].lwrModel_.setThetas(thetas[i]))
        {
            printf("ERROR: Could not set thetas of transformation system %i.\n",i);
            return false;
        }
    }

    return true;
}

bool DynamicMovementPrimitive::getWidthsAndCenters(std::vector<Eigen::VectorXd>& widths, std::vector<Eigen::VectorXd>& centers)
{
    if(!initialized_) return false;

    widths.clear();
    centers.clear();

    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        int numRfs;
        if (!transformationSystems_[i].lwrModel_.getNumRFS(numRfs))
        {
            printf("ERROR: Could not get number of receptive fields.\n");
            return false;
        }

        VectorXd centersVector = VectorXd::Zero(numRfs);
        VectorXd widthsVector = VectorXd::Zero(numRfs);
        if (!transformationSystems_[i].lwrModel_.getWidthsAndCenters(widthsVector , centersVector))
        {
            printf("ERROR: Could not retrieve thetas from transformation system %i.\n",i);
            return false;
        }
        widths.push_back(widthsVector);
        centers.push_back(centersVector);
    }
    return true;
}

bool DynamicMovementPrimitive::getWidthsAndCenters(const int transSystemIndex, VectorXd& widths, VectorXd& centers)
{
    if(!initialized_) return false;

    int numRfs;

	if(getNumRFS(transSystemIndex, numRfs)) return false;
    if(!widths.size() == numRfs) return false;
    if(!centers.size() == numRfs) return false;

    if (!transformationSystems_[transSystemIndex].lwrModel_.getWidthsAndCenters(widths, centers))
    {
        printf("ERROR: Could not get widths and centers of transformation system %i.\n", transSystemIndex);
        return false;
    }

    return true;
}

bool DynamicMovementPrimitive::getBasisFunctions(const int numTimeSteps, std::vector<MatrixXd>& basisFunctions)
{
    if(!initialized_) return false;

    basisFunctions.clear();
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        int numRfs;
        if (!getNumRFS(i, numRfs))
        {
            return false;
        }

        MatrixXd basisFunctionMatrix = MatrixXd::Zero(numTimeSteps, numRfs);
        VectorXd xInputVector = VectorXd::Zero(numTimeSteps);
        double dx = static_cast<double> (1.0) / static_cast<double> (numTimeSteps - 1);
        xInputVector(0) = 0.0;
        for (int j = 1; j < numTimeSteps; j++)
        {
            xInputVector(j) = xInputVector(j - 1) + dx;
        }
        if (!transformationSystems_[i].lwrModel_.generateBasisFunctionMatrix(xInputVector, basisFunctionMatrix))
        {
            printf("ERROR: LWR basis function generation failed!\n");
            return false;
        }
        basisFunctions.push_back(basisFunctionMatrix);
    }

    return true;
}

bool DynamicMovementPrimitive::getCanonicalSystem(const int numTimeSteps, VectorXd& canSystemVector)
{
    if(!canSystemVector.size() == numTimeSteps) return false;
    if(numTimeSteps <= 0) return false;

    double dt = params_.tau_ / static_cast<double> (numTimeSteps - 1);
    double time = 0;

    canSystemVector(0) = 1;
    for (int j = 1; j < numTimeSteps; j++)
    {
        integrateCanonicalSystem(canSystemVector(j), time);
        time += dt;
    }

    return true;
}

bool DynamicMovementPrimitive::getNumRFS(const int transId, int& numRfs)
{
    if(!initialized_) return false;

    if ((transId < 0) || (transId >= params_.numTransformationSystems_))
    {
        printf("ERROR: Could not get number of receptive fields, the transformation system id (%i) is invalid.\n", transId);
        return false;
    }

    return transformationSystems_[transId].lwrModel_.getNumRFS(numRfs);
}

bool DynamicMovementPrimitive::getNumRFS(std::vector<int>& numRfs)
{
    numRfs.clear();
    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        int tmpNumRfs;
        if (!getNumRFS(i, tmpNumRfs))
        {
            return false;
        }
        numRfs.push_back(tmpNumRfs);
    }

    return true;
}

bool DynamicMovementPrimitive::setDuration(const double movementDuration, const int samplingFrequency)
{
    if (!params_.isSetup_)
    {
        printf("ERROR: DMP need to be setup first.\n");
        return false;
    }

    if (samplingFrequency <= 0)
    {
       printf("ERROR: Sampling frequency %i [Hz] is not valid.\n", samplingFrequency);
        return false;
    }

	//is this condition so restrictive???
    if (movementDuration <= 0.5)
    {
        printf("ERROR: Movement duration (%f) is too small.\n", movementDuration);
        return false;
    }

    params_.tau_ = movementDuration;
    params_.deltaT_ = static_cast<double> (1.0) / static_cast<double> (samplingFrequency);
    return true;
}

bool DynamicMovementPrimitive::propagateFull(Trajectory& trajectory, const double samplingDuration, const int numSamples)
{


    if ((!params_.isLearned_) || (!params_.isSetup_) || (!params_.isStartSet_))
    {
        if(!params_.isLearned_) printf("ERROR: DMP is not learned from demonstration.\n");
        if(!params_.isSetup_) printf("ERROR: DMP with is not setup. Need to set start, goal, and duration first.\n");
        return false;
    }

    if(trajectory.getMaxDimension() < params_.numTransformationSystems_ * POS_VEL_ACC) return false;
    if(trajectory.getMaxLength() <= numSamples) return false;

    double specialSamplingFrequency = static_cast<double> (numSamples) / (samplingDuration);
    if (!trajectory.setSamplingFrequency(specialSamplingFrequency))
    {
        printf("ERROR: Could not set sampling frequency.\n");
        return false;
    }

    VectorXd desiredCoordinates = VectorXd::Zero(params_.numTransformationSystems_ * POS_VEL_ACC);
    bool movementFinished = false;
    while (!movementFinished)
    {
        if (!propagateStep(desiredCoordinates, movementFinished, samplingDuration, numSamples))
        {
            printf("ERROR: Could not propagate dmp.\n");
            return false;
        }

        if (!trajectory.add(desiredCoordinates))
        {
            printf("ERROR: Could not add point to trajectory.\n");
            return false;
        }
    }

    return true;
}

bool DynamicMovementPrimitive::propagateStep(VectorXd &desiredCoordinates, bool &movementFinished)
{
    return propagateStep(desiredCoordinates, movementFinished, params_.tau_, static_cast<int> (params_.tau_ / params_.deltaT_));
}

bool DynamicMovementPrimitive::propagateStep(VectorXd &desiredCoordinates, bool &movementFinished, const double samplingDuration, const int numSamples)
{
    if ((!params_.isLearned_) || (!params_.isSetup_) || (!params_.isStartSet_))
    {
        if(!params_.isLearned_) printf("DMP is not learned from demonstration.\n");
        if(!params_.isSetup_) printf("DMP is not setup. Need to set start, goal, and duration first.\n");
        movementFinished = true;
        return false;
    }

    if (desiredCoordinates.size() != params_.numTransformationSystems_ * POS_VEL_ACC)
    {
        printf("ERROR: Number of desired coordinates (%i) is not correct, it should be %i. TODO: REMOVE ME !!\n", desiredCoordinates.size(), params_.numTransformationSystems_ * POS_VEL_ACC);
        movementFinished = true;
        return false;
    }

    if (numSamples <= 0)
    {
        printf("ERROR: Number of samples (%i) is not valid. TODO: REMOVE ME !!\n", numSamples);
    }
    double dtTotal = samplingDuration / static_cast<double> (numSamples);
    double dtThreshold = static_cast<double> (1.0) / DEFAULT_SAMPLING_FREQUENCY;
    int numIteration = (int)ceil(dtTotal / dtThreshold);

    //integrate the system, make sure that all internal variables are set properly
    if (!integrate(dtTotal, numIteration))
    {
        printf("ERROR: Problem while integrating the dynamical system.\n");
        movementFinished = true;
        return false;
    }
    params_.numSamples_++;

    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        desiredCoordinates(i * POS_VEL_ACC + _POS_ - 1) = transformationSystems_[i].y_;
        desiredCoordinates(i * POS_VEL_ACC + _VEL_ - 1) = transformationSystems_[i].yd_;
        desiredCoordinates(i * POS_VEL_ACC + _ACC_ - 1) = transformationSystems_[i].ydd_;
    }

    //check whether movement has finished...
    if (params_.numSamples_ >= numSamples)
    {
        params_.isSetup_ = false;
        params_.isStartSet_ = false;
        movementFinished = true;
        return true;
    }
    else
    {
        movementFinished = false;
    }
    return true;
}

bool DynamicMovementPrimitive::integrateAndFit()
{
	/**
	* trajectoryTargetFunctionInput_ represents the input for the locally weighted regression
	* (lwr) algorithm used to find the values for theta parameters. As a classical regression
	* problem, given a function y(x) to fit by tuning some parameters of the model, the input
	* are some values of the independent variable (x) and the targets are the values of the
	* dependent variable (y) assumed in correspondence of those x values. Since for a dmp the
	* role of the independent variable (time) is played by the canonical system, in particular
	* by the behaviour of x, the input of the lwr algorithm in this case is the behaviour of
	* the canonical system itself, i.e. an exponentially decreasing trend from 1 to 0.
	* trajectoryTarget_ contained in the transformationSystem object represents the sequence
	* of values of the nonlinear function.
	*/

    trajectoryTargetFunctionInput_.push_back(canonicalSystem_.x);

    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {

        transformationSystems_[i].ft_ = ((transformationSystems_[i].tdd_ * pow(params_.tau_, 2) + params_.dGain_ * transformationSystems_[i].td_
                * params_.tau_) / params_.kGain_) - (transformationSystems_[i].goal_ - transformationSystems_[i].t_)
                + (transformationSystems_[i].goal_ - transformationSystems_[i].y0_) * canonicalSystem_.x;

        transformationSystems_[i].ft_ /= canonicalSystem_.x;

        //the nonlinearity is computed by LWR (later)
        transformationSystems_[i].trajectoryTarget_.push_back(transformationSystems_[i].ft_);

        //transformation state derivatives (make use of target knowledge)
        transformationSystems_[i].zd_ = (params_.kGain_ * (transformationSystems_[i].goal_ - transformationSystems_[i].y_) - params_.dGain_
                * transformationSystems_[i].z_ - params_.kGain_ * (transformationSystems_[i].goal_ - transformationSystems_[i].y0_)
                * canonicalSystem_.x + params_.kGain_ * transformationSystems_[i].ft_) * (static_cast<double> (1.0) / params_.tau_);
		
		//integrate all systems, coupling between z and y variables
        transformationSystems_[i].z_ += transformationSystems_[i].zd_ * params_.deltaT_;
        transformationSystems_[i].y_ += transformationSystems_[i].yd_ * params_.deltaT_;
    }

    //canonical system
    integrateCanonicalSystem(canonicalSystem_.x, canonicalSystem_.time);
    canonicalSystem_.time += params_.deltaT_;

    return true;
}

bool DynamicMovementPrimitive::learnTransformationTarget()
{
	/**
	* The input of the regression algorithm (lwr) is trajectory_target_function_input_
	* which represents the indipendent variable behaviour (i.e. canonical system/x) and the target is 
	* trajectory_target_ , contained into the transformation system, which are the values of the nonlinear
	* function f.
	*/

    Eigen::Map<VectorXd> input = VectorXd::Map(&trajectoryTargetFunctionInput_[0], trajectoryTargetFunctionInput_.size());

    for (int i = 0; i < params_.numTransformationSystems_; i++)
    {
        if (trajectoryTargetFunctionInput_.size() != transformationSystems_[i].trajectoryTarget_.size())
        {
            printf("ERROR: Traget trajectory of transformation system %i has different size than input trajectory.\n", i);
            return false;
        }
        Eigen::Map<VectorXd> target = VectorXd::Map(&transformationSystems_[i].trajectoryTarget_[0], transformationSystems_[i].trajectoryTarget_.size());
        if (!transformationSystems_[i].lwrModel_.learnWeights(input, target))
        {
            printf("ERROR: Could not learn weights of transformation system %i.\n", i);
            return false;
        }
    }

    //compute mean squared error
    for (int i = 0; i < params_.numTransformationSystems_; ++i)
    {
        transformationSystems_[i].computeMSE();
    }

    return true;
}

inline bool DynamicMovementPrimitive::integrate(const double dtTotal, const int numIteration)
{
	//what does num_iteration means?

    double dt = dtTotal / static_cast<double> (numIteration);

    for (int n = 0; n < numIteration; n++)
    {

        for (int i = 0; i < params_.numTransformationSystems_; i++)
        {
            //compute nonlinearity using LWR
            double prediction = 0;
            if (!transformationSystems_[i].lwrModel_.predict(canonicalSystem_.x, prediction))
            {
                printf("ERROR: Could not predict output.\n");
                return false;
            }

			//compute all the actual values for the transformation system differential equations.
            transformationSystems_[i].f_ = prediction * canonicalSystem_.x;

            transformationSystems_[i].zd_ = (params_.kGain_ * (transformationSystems_[i].goal_ - transformationSystems_[i].y_) - params_.dGain_
                    * transformationSystems_[i].z_ - params_.kGain_ * (transformationSystems_[i].goal_ - transformationSystems_[i].y0_)
                    * canonicalSystem_.x + params_.kGain_ * transformationSystems_[i].f_) * (static_cast<double> (1.0) / params_.tau_);

            transformationSystems_[i].yd_ = transformationSystems_[i].z_ * (static_cast<double> (1.0) / params_.tau_);
            transformationSystems_[i].ydd_ = transformationSystems_[i].zd_ * (static_cast<double> (1.0) / params_.tau_);

            transformationSystems_[i].z_ += transformationSystems_[i].zd_ * dt; //* params_.delta_t_;
            transformationSystems_[i].y_ += transformationSystems_[i].yd_ * dt; //* params_.delta_t_;

        }

        //canonical system
        integrateCanonicalSystem(canonicalSystem_.x, canonicalSystem_.time);
        canonicalSystem_.time += dt; //+= params_.delta_t_;
    }

    return true;
}

inline void DynamicMovementPrimitive::integrateCanonicalSystem(double& canonicalSystemX, const double canonicalSystemTime) const
{
    canonicalSystemX = exp(-(params_.alphaX_ / params_.tau_) * canonicalSystemTime);
}

bool DynamicMovementPrimitive::isIncreasing(int transformationSystemIndex, bool &isIncreasing)
{
    if ((transformationSystemIndex >= 0) && (transformationSystemIndex < params_.numTransformationSystems_) && params_.isLearned_)
    {
        return (transformationSystems_[transformationSystemIndex].initialY0_ >= transformationSystems_[transformationSystemIndex].initialGoal_);
    }

    return false;
}

double DynamicMovementPrimitive::getProgress() const
{
    double progress;
    if (canonicalSystem_.x < 1.0e-8)
    {
        progress = 1.0;
    }
    else if (params_.alphaX_ > 1.0e-8)
    {
        progress = -log(canonicalSystem_.x) / params_.alphaX_;
    }
    else
    {
        progress = 0.0;
    }

    return progress;
}

bool DynamicMovementPrimitive::writeVectorToFile(std::vector<double> trajectory, std::string fileName)
{
	FILE *fp;
	int count = 0;
    if ((fp = fopen(fileName.c_str(), "w")) == NULL)
    {
        printf("ERROR: Cannot open file: %s\n", fileName.c_str());
        return false;
    }

	std::vector<double>::iterator iter = trajectory.begin();
	for(; iter != trajectory.end(); ++iter)
	{
		fprintf(fp, "%f\n", *iter);
		count++;
	}

	return true;
}

}
