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

#ifndef POLICY_H_
#define POLICY_H_

#include <vector>

#define EIGEN2_SUPPORT
#include <Eigen/Core>

namespace library
{

/**
* \ingroup piSquare
*
* Base class representing a general parameterized policy used in Reinforcement Learning algorithms. 
*/

class Policy
{
public:

	/**
    * class constructor
    */
    Policy(){};

	/**
    * class destructor
    */
    virtual ~Policy(){};

	/**
    * Sets the number of time steps used in reinforcement learning
    * @param numTimeSteps is the number of time steps used in reinforcement learning
	* @return true if operation is successful, false otherwise
    */
    virtual bool setNumTimeSteps(const int numTimeSteps) = 0;

	/**
    * Gets the number of time steps used in reinforcement learning
    * @param numTimeSteps is the number of time steps used in reinforcement learning
	* @return true if operation is successful, false otherwise
    */
    virtual bool getNumTimeSteps(int& numTimeSteps) = 0;

	/**
    * Gets the number of dimensions
    * @param numDimensions is the number of dimensions of the reinforcement learning problem
	* @return true if operation is successful, false otherwise
    */
    virtual bool getNumDimensions(int& numDimensions) = 0;

	/**
    * Gets the number of policy parameters per dimension 
    * @param numParams is the parameter numbers vector, one element for each dimension
	* @return true if operation is successful, false otherwise
    */
    virtual bool getNumParameters(std::vector<int>& numParams) = 0;

	/**
    * Gets the basis functions that multiply the policy parameters in the dynamical system.
	* @param basisFunctions is an array of "numTimeSteps x numParameters" matrices, per dimension
	* @return true if operation is successful, false otherwise
    */
    virtual bool getBasisFunctions(std::vector<Eigen::MatrixXd>& basisFunctions) = 0;

	/**
    * Gets the positive semi-definite matrix of the quadratic control cost.
	* The weight of this control cost is provided by the task.
    * @param controlCosts is an array of square, positive semi-definite matrix: numParams x numParams
	* @return true if operation is successful, false otherwise
    */
    virtual bool getControlCosts(std::vector<Eigen::MatrixXd>& controlCosts) = 0;

	/**
    * Updates the policy parameters based on the updates per timestep.
    * @param updates is the parameter updates per time-step, numTimeSteps x numParameters
	* @return true if operation is successful, false otherwise
    */
    virtual bool updateParameters(const std::vector<Eigen::MatrixXd>& updates) = 0;

	/**
    * Gets the policy parameters per dimension 
    * @param parameters is the vector of vector parameters
	* @return true if operation is successful, false otherwise
    */
    virtual bool getParameters(std::vector<Eigen::VectorXd>& parameters) = 0;

	/**
    * Sets the policy parameters per dimension
    * @param parameters is the vector of vector parameters
	* @return true if operation is successful, false otherwise
    */
    virtual bool setParameters(const std::vector<Eigen::VectorXd>& parameters) = 0;

	/**
    * Compute the control costs over time, given the control cost matrix per dimension and parameters over time
	* @param controlCostMatrices: [numDimensions] numParameters x numParameters: Quadratic control cost matrix (R)
	* @param parameters: [numDimensions][numTimeSteps] numParameters: Parameters over time (can also be theta + projected noise)
	* @param weight: constant multiplier for the control costs
	* @param controlCosts: [numDimensions] numTimeSteps: Control costs over time
	* @return true if operation is successful, false otherwise
    */
    virtual bool computeControlCosts(const std::vector<Eigen::MatrixXd>& controlCostMatrices, 
									 const std::vector<std::vector<Eigen::VectorXd> >& parameters,
                                     const double weight, std::vector<Eigen::VectorXd>& controlCosts);

};

}

#endif /* POLICY_H_ */
