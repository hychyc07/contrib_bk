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


#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <limits.h>
#include <direct.h>

#include <iCub/piSquare/policyImprovementManager/policyImprovementManager.h>

using namespace Eigen;
using namespace pi2;
using namespace library;
using namespace yarp::os;
using namespace yarp::sig;


PolicyImprovementManager::PolicyImprovementManager(ResourceFinder* rf, double movementDuration, Vector& start, Vector& goal, bool averagedCost) :
	rf_(rf),
	initialTrajectory_(rf),
	bestTrajectory_(rf),
	initialized_(false),
	numDimensions_(start.size()),
    movementDuration_(movementDuration),
    samplingFrequency_(-1),
	dt_(-1),
	controlCostWeight_(-1),
	noiseDecay_(-1),
	bestCost_(std::numeric_limits<double>::max()),
	startPositions_(start),
	goalPositions_(goal),
	averagedCost_(averagedCost)
{
}

PolicyImprovementManager::PolicyImprovementManager(ResourceFinder* rf, double movementDuration, int numDim, dmp::Trajectory& trajectory, bool averagedCost) :
	rf_(rf),
	initialTrajectory_(trajectory),
	bestTrajectory_(rf),
	initialized_(false),
	numDimensions_(numDim),
    movementDuration_(movementDuration),
    samplingFrequency_(-1),
	dt_(-1),
	controlCostWeight_(-1),
	noiseDecay_(-1),
	bestCost_(std::numeric_limits<double>::max()),
	averagedCost_(averagedCost)
{
	// because trajectories works on eigen vectors
	VectorXd tmpStart = VectorXd::Zero(numDim);
	VectorXd tmpGoal = VectorXd::Zero(numDim);
	trajectory.getStartPosition(tmpStart);
	trajectory.getEndPosition(tmpGoal);
	dmp::MathHelper::eigenToYarpVector(tmpStart, startPositions_);
	dmp::MathHelper::eigenToYarpVector(tmpGoal, goalPositions_);
}

PolicyImprovementManager::~PolicyImprovementManager()
{
}

bool PolicyImprovementManager::initialize()
{
	//reads all the parameters from .ini file

	std::string parameterName = "num_time_steps";
	if (!rf_->check(parameterName.c_str()))
    {
        printf("ERROR: Parameter %s not found!\n", parameterName.c_str());
		initialized_ = false;
        return initialized_;
    }
	numTimeSteps_ = rf_->find(parameterName.c_str()).asInt();
	
	parameterName = "num_trials";
	if (!rf_->check(parameterName.c_str()))
    {
        printf("ERROR: Parameter %s not found!\n", parameterName.c_str());
        initialized_ = false;
        return initialized_;
    }
	numTrials_ = rf_->find(parameterName.c_str()).asInt();

	parameterName = "num_rollouts";
	if (!rf_->check(parameterName.c_str()))
    {
        printf("ERROR: Parameter %s not found!\n", parameterName.c_str());
        initialized_ = false;
        return initialized_;
    }
	numRollouts_ = rf_->find(parameterName.c_str()).asInt();

	parameterName = "num_reused_rollouts";
	if (!rf_->check(parameterName.c_str()))
    {
        printf("ERROR: Parameter %s not found!\n", parameterName.c_str());
        initialized_ = false;
        return initialized_;
    }
	numReusedRollouts_ = rf_->find(parameterName.c_str()).asInt();

	parameterName = "variance";
	if (!rf_->check(parameterName.c_str()))
    {
        printf("ERROR: Parameter %s not found!\n", parameterName.c_str());
        initialized_ = false;
        return initialized_;
    }

	Bottle variance = rf_->findGroup("variance").tail();
	for(int i = 0; i < variance.size(); i++)
	{
		variance_.push_back(variance.get(i).asDouble());
	}

	parameterName = "delta_t";
	if (!rf_->check(parameterName.c_str()))
    {
        printf("ERROR: Parameter %s not found!\n", parameterName.c_str());
        initialized_ = false;
        return initialized_;
    }
	dt_ = rf_->find(parameterName.c_str()).asDouble();

	parameterName = "control_cost_weight";
	if (!rf_->check(parameterName.c_str()))
    {
        printf("ERROR: Parameter %s not found!\n", parameterName.c_str());
        initialized_ = false;
        return initialized_;
    }
	controlCostWeight_ = rf_->find(parameterName.c_str()).asDouble();

	parameterName = "noise_decay";
	if (!rf_->check(parameterName.c_str()))
    {
        printf("ERROR: Parameter %s not found!\n", parameterName.c_str());
        initialized_ = false;
        return initialized_;
    }
	noiseDecay_ = rf_->find(parameterName.c_str()).asDouble();

	//some computation only possible once parameters were read from file
	samplingFrequency_ = double(movementDuration_) / double(numTimeSteps_);
	costTrend_ = VectorXd::Zero(numTrials_);

	//create directory where output file are stored
	mkdir("data");

	//some checks for consistency
	if(startPositions_.size() != goalPositions_.size())
	{
		printf("ERROR: Start and goal positions vector dimension must agree\n");
        initialized_ = false;
        return initialized_;
	}

	if(numDimensions_ <= 0 || movementDuration_ <= 0 || controlCostWeight_ < 0 || noiseDecay_ <= 0 || samplingFrequency_ <= 0 || dt_ <= 0)
	{
		printf("ERROR: One or more parameters received wrong initialization values\n");
        initialized_ = false;
        return initialized_;
	}

	if(variance_.size() != 1 && variance_.size() != numDimensions_)
	{
		printf("ERROR: the number of dimensions of the variance vector must be equal to either 1 or the number of problem dimensions\n");
		initialized_ = false;
		return initialized_;
	}

    initialized_ = true;

    return initialized_;
}

bool PolicyImprovementManager::runPI2()
{
    if(!initialized_) return false;

    boost::shared_ptr<dmp::DynamicMovementPrimitive> dmp(new dmp::DynamicMovementPrimitive(rf_));

    if(!dmp->initialize(numDimensions_)) return false;
	
	// if an already trajectory was given, then learn from it.
	// Otherwise generate a minimum jerk trajectory from start to goal and learn from it
	if(initialTrajectory_.isInitialized())
	{
		if(!dmp->learnFromCustomTrajectory(initialTrajectory_)) return false;
		if(!dmp->setup(1.0/(movementDuration_ / numTimeSteps_))) return false;
	}
	else
	{
		if(!dmp->learnFromMinJerk(startPositions_, goalPositions_, movementDuration_, dt_)) return false;
		if(!dmp->setup(1.0/dt_)) return false;
	}

    policy_.reset(new DMPPolicy());
    if(!policy_->initialize(dmp)) return false;

    PolicyImprovement policyImprovement;
    if(!policyImprovement.initialize(numRollouts_, numTimeSteps_, numReusedRollouts_, 0, policy_)) return false;

	//data structures initializations
    std::vector<std::vector<Eigen::VectorXd> > rollouts;
    std::vector<Eigen::MatrixXd> parameterUpdates;
    std::vector<Eigen::VectorXd> parameters;
    Eigen::MatrixXd rolloutCosts = MatrixXd::Zero(numRollouts_, numTimeSteps_);
    Eigen::VectorXd rolloutCost = VectorXd::Zero(numTimeSteps_);
    std::vector<double> noiseVector;
    for (int i = 0; i < numDimensions_; ++i)
    {
		if (variance_.size() == 1) noiseVector.push_back(variance_[0]);
		else noiseVector.push_back(variance_[i]);
    }
	double trialCost = 0.0;
    dmp::Trajectory trajectory(rf_);

    //the PI^2 loop
    for (int i = 0; i < numTrials_; ++i)
    {
        if(!policy_->getParameters(parameters)) return false;

        policy_->getDMP(dmp);

		/**
        * gets the noise-less cost:
		* computes the current trajectory from the parameters containted into the dmp, and then
		* assigns a cost to such trajectory just considering "state" information, no control effort.
		* If the noise cannot be turned off, i.e. the system is stochastic, then the cost related to
		* the new parameters is computed as an average value over 5 identical trials.
		*/
		if(averagedCost_)
		{
			trialCost = 0.0;
			for(int i = 0; i < NUM_SAMPLE_TRIALS; i++)
			{
				if(!rollout(parameters, rolloutCost, trajectory)) return false;
				trialCost += rolloutCost.sum();
			}

			trialCost /= NUM_SAMPLE_TRIALS;
		}
		else
		{
			if(!rollout(parameters, rolloutCost, trajectory)) return false;
			trialCost = rolloutCost.sum();
		}
		
		costTrend_(i) = trialCost;
        printf("iteration %d, cost: %lf\n", i+1, trialCost);

		// partial writing to save current results (debug)
		if(!writeCostTrendToFile("data/costTrend.txt")) return false;

		if(i % 10 == 0)
		{
			std::stringstream sstm;
			sstm << "data/TrajectoryTrial" << i << ".txt";
			if(!trajectory.writeTrajectoryToFile(sstm.str())) return false;
		}

		/**
		* if the cost is lower than the current lowest cost, then save the current trial as the
		* best trajectory and update the best cost with the current one.
		*/
		if(trialCost < bestCost_)
		{
			bestCost_ = trialCost;
			bestTrajectory_ = trajectory;
			bestTrajectoryIndex_ = i;
		}

		/**
		* once the noise-less trajectory is generated, some noisy rollouts are generated in order to
		* improve the theta parameters. Such distinction between noisy and noise-less trajectory makes
		* sense when the system is simulated during the improvement phase and physically executed only
		* once between an improvement and the following.
		*/
        if(!policyImprovement.getRollouts(rollouts, noiseVector)) return false;

		/**
		* just like before, a trajectory is computed from the corresponding theta vector, and
		* its cost is evaluated with the same cost functional. The difference is that here we
		* are considering noisy parameters vectors.
		*/
        for (int r=0; r<int(rollouts.size()); ++r)
        {
            rollout(rollouts[r], rolloutCost, trajectory);
            rolloutCosts.row(r) = rolloutCost.transpose();
			
			std::stringstream sstm2;
			sstm2 << "data/TrajectoryTrial" << i+1 << "Rollout" << r << ".txt";
			//trajectory.writeTrajectoryToFile(sstm2.str());
        }

		/**
		* the costs related to the rollouts are used to update the actual theta parameters.
		* NOTE: allCosts, used as a container of setRolloutCosts function, is not used.
		*/
        std::vector<double> allCosts;
        if(!policyImprovement.setRolloutCosts(rolloutCosts, controlCostWeight_, allCosts)) return false;
        if(!policyImprovement.improvePolicy(parameterUpdates)) return false;
        if(!policy_->updateParameters(parameterUpdates)) return false;
        for (int i=0; i<numDimensions_; ++i) noiseVector[i] *= noiseDecay_;
    }

	//store results (cost behaviour over trials and best trajectory)
	if(!writeCostTrendToFile("data/costTrend.txt")) return false;
	if(!bestTrajectory_.writeTrajectoryToFile("data/bestTrajectory.txt")) return false;
	

    return true;
}

bool PolicyImprovementManager::rollout(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, dmp::Trajectory& trajectoryOut)
{
    boost::shared_ptr<dmp::DynamicMovementPrimitive> dmpPtr;
    policy_->getDMP(dmpPtr);

    dmp::Trajectory trajectory(rf_);

    if (!trajectory.initialize(numDimensions_*3, samplingFrequency_, numTimeSteps_+1, numDimensions_*3))
    {
        printf("ERROR: Could not initialize trajectory.\n");
        return false;
    }

    dmp::DynamicMovementPrimitive newDmp = *dmpPtr;
    newDmp.setThetas(parameters);
    newDmp.propagateFull(trajectory, movementDuration_, numTimeSteps_);

	Vector tmpCosts;
    computeCost(trajectory, tmpCosts);

	dmp::MathHelper::yarpToEigenVector(tmpCosts, costs);
    trajectoryOut = trajectory;

    return true;
}

bool PolicyImprovementManager::writeCostTrendToFile(std::string fileName)
{
	FILE *fp;
    if ((fp = fopen(fileName.c_str(), "w")) == NULL)
    {
        printf("ERROR: Cannot open file %s\n", fileName.c_str());
        return false;
    }

	for (int i = 0; i < costTrend_.size(); i++)
    {
		fprintf(fp, "%f\n", costTrend_(i));
    }

	fclose(fp);

	return true;
}