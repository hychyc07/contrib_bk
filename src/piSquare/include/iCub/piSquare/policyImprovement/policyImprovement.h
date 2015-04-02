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

#ifndef POLICYIMPROVEMENT_H_
#define POLICYIMPROVEMENT_H_

#include <Eigen/Core>

#include <iCub/piSquare/policyLibrary/policy.h>
#include <iCub/piSquare/policyImprovement/multivariateGaussian.h>

namespace pi2
{

/**
* structure containing the whole amount of data for a single rollout
* REMARK: the therm "rollout" here refers to a set of theta values, not to the corresponding trajectory.
* This because the improvement regards the variation of the theta vector.
*/
struct Rollout
{
    std::vector<Eigen::VectorXd> parameters_;								/**< [numDimensions] numParameters */
    std::vector<Eigen::VectorXd> noise_;									/**< [numDimensions] numParameters */
    std::vector<std::vector<Eigen::VectorXd> > noiseProjected_;				/**< [numDimensions][numTimeSteps] numParameters */
    std::vector<std::vector<Eigen::VectorXd> > parametersNoiseProjected_;   /**< [numDimensions][numTimeSteps] numParameters */
    Eigen::VectorXd stateCosts_;											/**< numTimeSteps */
    std::vector<Eigen::VectorXd> controlCosts_;								/**< [numDimensions] numTimeSteps */
    std::vector<Eigen::VectorXd> totalCosts_;								/**< [numDimensions] numTimeSteps */
    std::vector<Eigen::VectorXd> cumulativeCosts_;							/**< [numDimensions] numTimeSteps */
    std::vector<Eigen::VectorXd> probabilities_;							/**< [numDimensions] numTimeSteps */

	/**
    * Gets the rollout cost = state cost + control costs per dimension
	* @return the rollout cost
    */
    double getCost();
};

/**
* \ingroup piSquare
*
* Class containing methods and data structures needed to implement PI^2 algorithm
*/

class PolicyImprovement
{
public:

	/**
	* class constructor
	*/
    PolicyImprovement();

	/**
	* class destructor
	*/
    ~PolicyImprovement();

	/**
	* Initializes the policy improvement object, which contains all the data and routine for implementing
	* PI^2 algorithm, by extracting the information from the dmp already initialized.
	* @param numRollouts is the number of rollouts for each trial
	* @param numTimeSteps represents the length of the trajectory generated starting from the theta parameters and
	* used during reinforcement learning.
	* @param numReusedRollouts is the number of rollouts which are kept from the previous trial
	* @param numExtraRollouts is the number of extra-generated rollouts
	* @param policy "directly" contains information about numTimeSteps_ parameter, the rest is contained
	* into the dmpPolicy object.
	* @param useCumulativeCosts specifies if cumulative cost is used or not
	* @return true if the object is initialized, false otherwise
	*/
    bool initialize(const int numRollouts, const int numTimeSteps, const int numReusedRollouts, const int numExtraRollouts, 
					boost::shared_ptr<library::Policy> policy, bool useCumulativeCosts=true);

	/**
	* initializes all the fields of the rollout data structure. The "default" rollout object
	* created in this function is then used to fill some members of the policy improvement object.
	* @param numRollouts is the number of rollouts for each trial
	* @param numReusedRollouts is the number of rollouts which are kept from the previous trial
	* @param numExtraRollouts is the number of extra-generated rollouts
	* @return true if operation is successful, false otherwise
	*/
    bool setNumRollouts(const int numRollouts, const int numReusedRollouts, const int numExtraRollouts);

	/**
	* gets the next set of rollouts. Only "new" rollouts that need to be executed are returned, not rollouts 
	* which might be reused from the previous set.
	* @param rollouts are the theta parameters representing rollouts, for each dimension, for each rollout
	* @param noiseStdDev is the variance of the noise
	* @return true if operation is successful, false otherwise
	*/
    bool getRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts, const std::vector<double>& noiseStddev);

	/** 
	* Set rollouts cost per time-step inside the rollouts_ data structure held by the policyImprovement object.
	* Only the first "n" rows of the costs matrix is used, where n is the number of rollouts
	* generated by getRollouts(), because some rollouts may be used from previous iterations.
	* @param cost matrix refers to the state component of the whole cost, while the control component is computed here.
	* @param controlCostWeight is a scalar representing the contro cost weight
	* @param rolloutCostsTotal is the output cost associated to the rollout
	* @return true if operation is successful, false otherwise
	*/
    bool setRolloutCosts(const Eigen::MatrixXd& costs, const double controlCostWeight, std::vector<double>& rolloutCostsTotal);

	/**
	* Performs the PI^2 update and provides parameter updates at every time step
	* The parameterUpdates returned here are still time step dependent...
	* @param parameterUpdates = [numDimensions] numTimeSteps x numParameters
	* @return true if operation is successful, false otherwise
	*/
    bool improvePolicy(std::vector<Eigen::MatrixXd>& parameterUpdates);

	/**
	* Adds extra rollouts to the set of rollouts to be reused
	* @param rollouts
	* @param rolloutCosts
	* @return true if operation is successful, false otherwise
	*/
    bool addExtraRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts, std::vector<Eigen::VectorXd>& rolloutCosts);

private:
	
	///flag indicating if the object is initialized or not
    bool initialized_;

	///number of dimensions for the reinforcement learning problem
    int numDimensions_;

	///vector whose elements are the number of parameters for each dimension
    std::vector<int> numParameters_;

	///number of time steps used for reinforcement learning
    int numTimeSteps_;

	///number or rollouts
    int numRollouts_;

	///number of reused rollouts
    int numRolloutsReused_;

	///number of extra rollouts
    int numRolloutsExtra_;

	///flag indicating if rollouts are reused or not in the current iteration (is it useful?)
    bool rolloutsReused_;
    
	///flag indicating if rollouts are reused or not in the next iteration (is it useful?)
	bool rolloutsReusedNext_;

	///flag indicating if the extra rollouts have been added for use in the next iteration
    bool extraRolloutsAdded_;
    
	///flag indicating how many new rollouts have been generated in this iteration
	int numRolloutsGen_;

	///flag indicating if cumulative costs or state costs are used
    bool useCumulativeCosts_;

	///pointer to a policy object containing all the needed data
    boost::shared_ptr<library::Policy> policy_;

	///Dimensions: [num_dimensions] num_parameters x num_parameters
    std::vector<Eigen::MatrixXd> controlCosts_;

	///Dimensions: [num_dimensions] num_parameters x num_parameters
    std::vector<Eigen::MatrixXd> invControlCosts_;

	///scalar representing the cost weight in the computation of the cost functional
    double controlCostWeight_;

	///Dimensions: [num_dimensions] num_time_steps x num_parameters
    std::vector<Eigen::MatrixXd> basisFunctions_;

	///Dimensions: [num_dimensions] num_parameters
    std::vector<Eigen::VectorXd> parameters_;

	///rollouts vector (in therms of theta parameters, not trajectories!)
    std::vector<Rollout> rollouts_;

	///reused rollouts vector
    std::vector<Rollout> reusedRollouts_;

	///extra rollouts vector
    std::vector<Rollout> extraRollouts_;

	///objects that generate noise for each dimension
    std::vector<MultivariateGaussian> noiseGenerators_;

	///Dimensions: noise projection_matrices[dimension][time_step]
    std::vector<std::vector<Eigen::MatrixXd>> projectionMatrices_;

	///Dimensions: [num_dimensions] num_time_steps x num_parameters
    std::vector<Eigen::MatrixXd> parameterUpdates_;

    //Temporary variables pre-allocated for efficiency

	///Dimensions: [num_dimensions] num_parameters
    std::vector<Eigen::VectorXd> tmpNoise_;

	///Dimensions: [num_dimensions] num_parameters
    std::vector<Eigen::VectorXd> tmpParameters_;

	///Dimensions: num_time_steps
    Eigen::VectorXd tmpMaxCost_;

	///Dimensions: num_time_steps
    Eigen::VectorXd tmpMinCost_;

	///Dimensions: num_time_steps
    Eigen::VectorXd tmpMaxMinusMinCost_;

	///Dimensions: num_time_steps
    Eigen::VectorXd tmpSumRolloutProbabilities_;

	///Vector used for sorting rollouts by their cost
    std::vector<std::pair<double, int> > rolloutCostSorter_;

	/**
	* allocates temporary variables with default values
	* @return true if operation is successful, false otherwise
	*/
    bool preAllocateTempVariables();
    
	/**
	* computes the projection matrices for the noise. For each dimension and for each
	* time step there is a matrix which multiplies the input noise.
	* @return true if operation is successful, false otherwise
	*/
	bool preComputeProjectionMatrices();

	/**
	* just calls computeProjectedNoise for each rollout 
	* @return true if operation is successful, false otherwise
	*/
    bool computeProjectedNoise();

	/**
	* just calls computeRolloutControlCosts for each rollout
	* @return true if operation is successful, false otherwise
	*/
    bool computeRolloutControlCosts();

	/**
	* Compute cumulative costs at each timestep
	* @return true if operation is successful, false otherwise
	*/
    bool computeRolloutCumulativeCosts();

	/**
	* Computes the probability associated to each rollout. Such probability represents
	* the weight of each rollout during the parameters update. Roughly speaking, rollouts
	* which received a lower cost are more important and influent in the weighted sum
	* with respect to those which received an higher cost. Here the probabilities are
	* normalized wrt the time steps of the trajectory...need to clarify.
	* @return true if operation is successful, false otherwise
	*/
    bool computeRolloutProbabilities();

	/**
	* Parameters update rule
	* @return true if operation is successful, false otherwise
	*/
    bool computeParameterUpdates();

	/**
	* computes the noise of a rollout as the difference between its parameters and
	* the current parameters vector
	* @param rollout is the rollout whose noise has to be evaluated
	* @return true if operation is successful, false otherwise
	*/
    bool computeNoise(Rollout& rollout);

	/**
	* perfoms the noise projection by using the projection matrices values
	* @param rollout is the rollout whose projected noise has to be evaluated
	* @return true if operation is successful, false otherwise
	*/
    bool computeProjectedNoise(Rollout& rollout);

	/**
	* just calls the method inside the policy object for control costs computation
	* @param rollout is the rollout whose control cost has to be evaluated
	* @return true if operation is successful, false otherwise
	*/
    bool computeRolloutControlCosts(Rollout& rollout);

	/**
	* extracts the parameters from the policy object
	* @return true if operation is successful, false otherwise
	*/
    bool copyParametersFromPolicy();

	/**
	* actual generation of the noisy parameters vectors. The number of rollouts to be generated
	* also depends on the number of rollouts we want to keep from the previous exeution: classical
	* trade-off between exploration and exploitation.
	* @param noiseVariance is the exploration noise variance
	* @return true if operation is successful, false otherwise
	*/
    bool generateRollouts(const std::vector<double>& noiseVariance);

};

}

#endif /* POLICYIMPROVEMENT_H_ */
