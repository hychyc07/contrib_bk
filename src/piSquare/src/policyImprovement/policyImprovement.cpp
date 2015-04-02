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

#include <time.h>
#include <cfloat>
#include <algorithm>

#define EIGEN2_SUPPORT
#include <Eigen/LU>
#define EIGEN2_SUPPORT
#include <Eigen/Array>

#include <iCub/piSquare/policyImprovement/policyImprovement.h>

using namespace Eigen;
using namespace library;

namespace pi2
{

PolicyImprovement::PolicyImprovement():
    initialized_(false)
{
}

PolicyImprovement::~PolicyImprovement()
{
}

bool PolicyImprovement::initialize(const int numRollouts, const int numTimeSteps, const int numReusedRollouts, const int numExtraRollouts, 
									boost::shared_ptr<Policy> policy, bool useCumulativeCosts)
{
    numTimeSteps_ = numTimeSteps;
    useCumulativeCosts_ = useCumulativeCosts;
    policy_ = policy;

	initialized_ = true;

    if(!policy_->setNumTimeSteps(numTimeSteps_)) initialized_ = false;
    if(!policy_->getControlCosts(controlCosts_)) initialized_ = false;
    if(!policy_->getNumDimensions(numDimensions_)) initialized_ = false;
    if(!policy_->getNumParameters(numParameters_)) initialized_ = false;
    if(!policy_->getBasisFunctions(basisFunctions_)) initialized_ = false;
    if(!policy_->getParameters(parameters_)) initialized_ = false;

    //invert the control costs, initialize noise generators:
    invControlCosts_.clear();
    noiseGenerators_.clear();
    for (int d=0; d<numDimensions_; ++d)
    {
        invControlCosts_.push_back(controlCosts_[d].inverse());

		//A multivariate Gaussian Distribution is a generalization of the one-dimension
		//gaussian distribution: roughtly speaking, along each dimension (or their linear
		//combinations) data are always distributed as a gaussian function. The parameters
		//passed to the constructor represents the mean (here equal to zero) and the variance
		//of the multivariate gaussian. The fact that the variance is equal to the inverse
		//of the control cost matrix derives from theoretical considerations (see the paper).
        MultivariateGaussian mvg(VectorXd::Zero(numParameters_[d]), invControlCosts_[d]);
        noiseGenerators_.push_back(mvg);
    }

    if(!setNumRollouts(numRollouts, numReusedRollouts, numExtraRollouts)) initialized_ = false;
    if(!preAllocateTempVariables()) initialized_ = false;
    if(!preComputeProjectionMatrices()) initialized_ = false;

    return initialized_;
}

bool PolicyImprovement::setNumRollouts(const int numRollouts, const int numReusedRollouts, const int numExtraRollouts)
{
    numRollouts_ = numRollouts;
    numRolloutsReused_ = numReusedRollouts;
    numRolloutsExtra_ = numExtraRollouts;
    numRolloutsGen_ = 0;
    if (numRolloutsReused_ >= numRollouts)
    {
        printf("ERROR: Number of reused rollouts must be strictly less than number of rollouts.\n");
        return false;
    }

    //preallocate memory for a single rollout:
    Rollout rollout;

    rollout.parameters_.clear();
    rollout.noise_.clear();
    rollout.noiseProjected_.clear();
    rollout.parametersNoiseProjected_.clear();
    rollout.controlCosts_.clear();
    rollout.totalCosts_.clear();
    rollout.cumulativeCosts_.clear();
    rollout.probabilities_.clear();
    for (int d=0; d<numDimensions_; ++d)
    {
        rollout.parameters_.push_back(VectorXd::Zero(numParameters_[d]));
        rollout.noise_.push_back(VectorXd::Zero(numParameters_[d]));
        std::vector<VectorXd> tmpProjectedNoise;
        for (int t=0; t<numTimeSteps_; ++t)
        {
            tmpProjectedNoise.push_back(VectorXd::Zero(numParameters_[d]));
        }
        rollout.noiseProjected_.push_back(tmpProjectedNoise);
        rollout.parametersNoiseProjected_.push_back(tmpProjectedNoise);
        rollout.controlCosts_.push_back(VectorXd::Zero(numTimeSteps_));
        rollout.totalCosts_.push_back(VectorXd::Zero(numTimeSteps_));
        rollout.cumulativeCosts_.push_back(VectorXd::Zero(numTimeSteps_));
        rollout.probabilities_.push_back(VectorXd::Zero(numTimeSteps_));
    }
    rollout.stateCosts_ = VectorXd::Zero(numTimeSteps_);

    //duplicate this rollout:
    for (int r=0; r<numRollouts; ++r)
        rollouts_.push_back(rollout);

    for (int r=0; r<numReusedRollouts; ++r)
        reusedRollouts_.push_back(rollout);

    for (int r=0; r<numExtraRollouts; ++r)
        extraRollouts_.push_back(rollout);

    rolloutsReused_ = false;
    rolloutsReusedNext_ = false;
    extraRolloutsAdded_ = false;
    rolloutCostSorter_.reserve(numRollouts_);

    return true;
}

double Rollout::getCost()
{
    double cost = stateCosts_.sum();
    int num_dim = controlCosts_.size();
    for (int d=0; d<num_dim; ++d)
        cost += controlCosts_[d].sum();
    return cost;
}

bool PolicyImprovement::generateRollouts(const std::vector<double>& noiseStddev)
{
	//rolloutParameters_ and rolloutNoise_ are assumed to be already allocated

    if(!initialized_) return false;
    if(!static_cast<int>(noiseStddev.size()) == numDimensions_) return false;

    //save the latest policy parameters:
    if(!copyParametersFromPolicy())
	{
		printf("ERROR while copying parameters from Policy\n");
		return false;
	}

    numRolloutsGen_ = numRollouts_ - numRolloutsReused_;
    if (!rolloutsReusedNext_)
    {
        numRolloutsGen_ = numRollouts_;
        if (numRolloutsReused_ > 0)
        {
            rolloutsReusedNext_ = true;
        }
    }
    else
    {
        //figure out which rollouts to reuse depending on their cost
        rolloutCostSorter_.clear();
        for (int r=0; r<numRollouts_; ++r)
        {
            double cost = rollouts_[r].getCost();
            rolloutCostSorter_.push_back(std::make_pair(cost,r));
        }
        if (extraRolloutsAdded_)
        {
            for (int r=0; r<numRolloutsExtra_; ++r)
            {
                double cost = extraRollouts_[r].getCost();
                rolloutCostSorter_.push_back(std::make_pair(cost,-r-1));
                //index is -ve if rollout is taken from extra_rollouts
            }
            extraRolloutsAdded_ = false;
        }
        std::sort(rolloutCostSorter_.begin(), rolloutCostSorter_.end());

        //use the best ones: (copy them into reused_rollouts)
        for (int r=0; r<numRolloutsReused_; ++r)
        {
            double reuse_index = rolloutCostSorter_[r].second;

            if (reuse_index >=0)
                reusedRollouts_[r] = rollouts_[(unsigned int)reuse_index];
            else
            {
                reusedRollouts_[r] = extraRollouts_[(unsigned int)-reuse_index-1];
            }
        }

        //copy them back from reusedRollouts_ into rollouts_
        for (int r=0; r<numRolloutsReused_; ++r)
        {
            rollouts_[numRolloutsGen_+r] = reusedRollouts_[r];

            //update the noise based on the new parameters, because their noise refers to the past value of thetas (?)
            for (int d=0; d<numDimensions_; ++d)
            {
                rollouts_[numRolloutsGen_+r].noise_[d] = rollouts_[numRolloutsGen_+r].parameters_[d] - parameters_[d];
            }
        }
        rolloutsReused_ = true;
    }

    //generate new rollouts by obtaining samples from a gaussian probability distribution
    for (int d=0; d<numDimensions_; ++d)
    {
        for (int r=0; r<numRolloutsGen_; ++r)
        {
            noiseGenerators_[d].sample(tmpNoise_[d]);
            rollouts_[r].noise_[d] = noiseStddev[d]*tmpNoise_[d];
            rollouts_[r].parameters_[d] = parameters_[d] + rollouts_[r].noise_[d];
        }
    }

    return true;
}

bool PolicyImprovement::getRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts, const std::vector<double>& noiseVariance)
{
    if (!generateRollouts(noiseVariance))
    {
        printf("ERROR: Failed to generate rollouts.\n");
        return false;
    }

    rollouts.clear();
    for (int r=0; r<numRolloutsGen_; ++r)
    {
        rollouts.push_back(rollouts_[r].parameters_);
    }

    computeProjectedNoise();

    return true;
}

bool PolicyImprovement::setRolloutCosts(const Eigen::MatrixXd& costs, const double controlCostWeight, std::vector<double>& rolloutCostsTotal)
{
    if(!initialized_) return false;

    controlCostWeight_ = controlCostWeight;
    computeRolloutControlCosts();

    for (int r=0; r<numRolloutsGen_; ++r)
    {
        rollouts_[r].stateCosts_ = costs.row(r).transpose();
    }

    //set the total costs

    rolloutCostsTotal.resize(numRollouts_);

	// debug
	printf("COSTS FOR CURRENT ROLLOUTS:\n");
    for (int r=0; r<numRollouts_; ++r)
    {
        rolloutCostsTotal[r] = rollouts_[r].getCost();
		printf("%f\n", rolloutCostsTotal[r]);
    }
	printf("\n");

    return true;
}

bool PolicyImprovement::computeProjectedNoise()
{
    for (int r=0; r<numRollouts_; ++r)
    {
        computeProjectedNoise(rollouts_[r]);
    }

    return true;
}

bool PolicyImprovement::computeRolloutControlCosts()
{
    for (int r=0; r<numRollouts_; ++r)
    {
        computeRolloutControlCosts(rollouts_[r]);
    }

    return true;
}

bool PolicyImprovement::computeRolloutCumulativeCosts()
{
    for (int r=0; r<numRollouts_; ++r)
    {
        for (int d=0; d<numDimensions_; ++d)
        {
            rollouts_[r].totalCosts_[d] = rollouts_[r].stateCosts_ + rollouts_[r].controlCosts_[d];
            rollouts_[r].cumulativeCosts_[d] = rollouts_[r].totalCosts_[d];
            if (useCumulativeCosts_)
            {
                for (int t=numTimeSteps_-2; t>=0; --t)
                {
                    rollouts_[r].cumulativeCosts_[d](t) += rollouts_[r].cumulativeCosts_[d](t+1);
                }
            }
        }
    }

    return true;
}

bool PolicyImprovement::computeRolloutProbabilities()
{
    for (int d=0; d<numDimensions_; ++d)
    {
        for (int t=0; t<numTimeSteps_; t++)
        {

            //find min and max cost over all rollouts:
            double minCost = rollouts_[0].cumulativeCosts_[d](t);
            double maxCost = minCost;
            for (int r=1; r<numRollouts_; ++r)
            {
                double c = rollouts_[r].cumulativeCosts_[d](t);
                if (c < minCost)
                    minCost = c;
                if (c > maxCost)
                    maxCost = c;
            }

            double denom = maxCost - minCost;

            //prevent divide by zero:
            if (denom < 1e-8)
                denom = 1e-8;

            double p_sum = 0.0;
            for (int r=0; r<numRollouts_; ++r)
            {
                //the -10.0 here is taken from the paper:
                rollouts_[r].probabilities_[d](t) = exp(-10.0*(rollouts_[r].cumulativeCosts_[d](t) - minCost)/denom);
                p_sum += rollouts_[r].probabilities_[d](t);
            }
            for (int r=0; r<numRollouts_; ++r)
            {
                rollouts_[r].probabilities_[d](t) /= p_sum;
            }

        }

    }

    return true;
}

bool PolicyImprovement::computeParameterUpdates()
{
    for (int d=0; d<numDimensions_; ++d)
    {
        parameterUpdates_[d] = MatrixXd::Zero(numTimeSteps_, numParameters_[d]);
        for (int t=0; t<numTimeSteps_; ++t)
        {
            for (int r=0; r<numRollouts_; ++r)
            {
                parameterUpdates_[d].row(t).transpose() += rollouts_[r].noiseProjected_[d][t] * rollouts_[r].probabilities_[d](t);
            }
        }
    }

    return true;
}

bool PolicyImprovement::improvePolicy(std::vector<Eigen::MatrixXd>& parameterUpdates)
{
	//computeRolloutCumulativeCosts performs what is already done by setRolloutCosts function in the main PI^2 loop.

    if(!initialized_) return false;

    computeRolloutCumulativeCosts();
    computeRolloutProbabilities();
    computeParameterUpdates();
    parameterUpdates = parameterUpdates_;

    return true;
}

bool PolicyImprovement::preAllocateTempVariables()
{
    tmpNoise_.clear();
    tmpParameters_.clear();
    parameterUpdates_.clear();
    for (int d=0; d<numDimensions_; ++d)
    {
        tmpNoise_.push_back(VectorXd::Zero(numParameters_[d]));
        tmpParameters_.push_back(VectorXd::Zero(numParameters_[d]));
        parameterUpdates_.push_back(MatrixXd::Zero(numTimeSteps_, numParameters_[d]));
    }
    tmpMaxCost_ = VectorXd::Zero(numTimeSteps_);
    tmpMinCost_ = VectorXd::Zero(numTimeSteps_);
    tmpSumRolloutProbabilities_ = VectorXd::Zero(numTimeSteps_);

    return true;
}

bool PolicyImprovement::preComputeProjectionMatrices()
{
	//projectionMatrices: for each dimension there is a vector of matrices, one for each time step.

    projectionMatrices_.clear();
    for (int d=0; d<numDimensions_; ++d)
    {
        std::vector<MatrixXd> projectionMatricesForDim;

        VectorXd basisFunction(numParameters_[d]);
        VectorXd invRTimesG(numParameters_[d]);
        for (int t=0; t<numTimeSteps_; ++t)
        {
            basisFunction = basisFunctions_[d].row(t).transpose();
            invRTimesG = invControlCosts_[d] * basisFunction;
            double gTransposeRG = basisFunction.dot(invRTimesG);

            if (gTransposeRG < 1e-6)
            {
                printf("WARNING: Denominator (g_transpose_r_g) is close to 0: %f\n", gTransposeRG);
            }

            //outer product
            projectionMatricesForDim.push_back((invRTimesG / gTransposeRG) * basisFunction.transpose());
        }
        projectionMatrices_.push_back(projectionMatricesForDim);

    }
    return true;
}

bool PolicyImprovement::addExtraRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts, std::vector<Eigen::VectorXd>& rolloutCosts)
{
    if(!int(rollouts.size()) == numRolloutsExtra_) return false;

    //update our parameter values, so that the computed noise is correct:
	if(!copyParametersFromPolicy())
	{
		printf("ERROR while copying parameters from Policy\n");
		return false;
	}

    for (int r=0; r<numRolloutsExtra_; ++r)
    {
        extraRollouts_[r].parameters_ = rollouts[r];
        extraRollouts_[r].stateCosts_ = rolloutCosts[r];
        computeNoise(extraRollouts_[r]);
        computeProjectedNoise(extraRollouts_[r]);
        computeRolloutControlCosts(extraRollouts_[r]);
    }

    extraRolloutsAdded_ = true;

    return true;
}

bool PolicyImprovement::computeNoise(Rollout& rollout)
{
    for (int d=0; d<numDimensions_; ++d)
    {
        rollout.noise_[d] =  rollout.parameters_[d] - parameters_[d];
    }

    return true;
}

bool PolicyImprovement::computeProjectedNoise(Rollout& rollout)
{
    for (int d=0; d<numDimensions_; ++d)
    {
        for (int t=0; t<numTimeSteps_; ++t)
        {
            rollout.noiseProjected_[d][t] = projectionMatrices_[d][t] * rollout.noise_[d];
            rollout.parametersNoiseProjected_[d][t] = rollout.parameters_[d] + rollout.noiseProjected_[d][t];
        }
    }

    return true;
}

bool PolicyImprovement::computeRolloutControlCosts(Rollout& rollout)
{
    policy_->computeControlCosts(controlCosts_, rollout.parametersNoiseProjected_, 0.5*controlCostWeight_, rollout.controlCosts_);

    return true;
}

bool PolicyImprovement::copyParametersFromPolicy()
{
    if (!policy_->getParameters(parameters_))
    {
        printf("ERROR: Failed to get policy parameters.\n");
        return false;
    }

    return true;
}

};
