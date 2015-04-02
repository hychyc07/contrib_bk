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

#ifndef DMP_POLICY_H_
#define DMP_POLICY_H_

#define EIGEN2_SUPPORT
#include <Eigen/Eigen>

#include <boost/shared_ptr.hpp>

#include <iCub/piSquare/policyLibrary/policy.h>
#include <iCub/piSquare/dmpMotionGeneration/dynamicMovementPrimitive.h>

namespace library
{

/**
* \ingroup piSquare
*
* Specialization of policy base class by defining DMPs as parameterized policies.
*/

class DMPPolicy : public Policy
{

public:

	/**
    * class constructor
    */
    DMPPolicy();

	/**
    * class destructor
    */
    virtual ~DMPPolicy();
	
	/**
    * Initializes the DMPPolicy object with a pointer to a already initialized dmp
    * @param dmp is the dynamic motion primitive used for initialization
	* @return true if operation is successful (initialization), false otherwise
    */
    bool initialize(boost::shared_ptr<dmp::DynamicMovementPrimitive> dmp);

	/**
    * Gets a dmp object
    * @param dmp is a pointer to the output dmp object
	* @return true if operation is successful, false otherwise
    */
    bool getDMP(boost::shared_ptr<dmp::DynamicMovementPrimitive>& dmp);

	/**
    * Sets the number of time steps used for reinforcement learning
    * @param numTimeSteps is the number of time steps
	* @return true if operation is successful, false otherwise
    */
    bool setNumTimeSteps(const int numTimeSteps);

	/**
    * Gets the number of time steps used for reinforcement learning
    * @param numTimeSteps is the number of time steps
	* @return true if operation is successful, false otherwise
    */
    bool getNumTimeSteps(int& numTimeSteps);

	/**
    * Gets the number of dimensions of the dmp
    * @param numDimensions is the number of dimensions
	* @return true if operation is successful, false otherwise
    */
    bool getNumDimensions(int& numDimensions);

	/**
    * Gets the number of policy parameters per dimension
    * @param numParams is a vector holding the number of parameters for each dimension
	* @return true if operation is successful, false otherwise
    */
    bool getNumParameters(std::vector<int>& numParams);

	/**
	* Extracts from the underlying dmp a vector of matrices representing the basis
	* functions used for the nonlinear function f. Information about the number
	* of time steps for the trajectory is used.
	* In a sense it is the inverse step of regression. From the knowledge of the theta
	* parameters, the nonlinear function values are computed by keeping in account the
	* basis functions values and the canonial system.
    * @param basisFunctions is a vector of basis function matrices (numDimensions x [numTimeSteps] x [numParameters])
	* @return true if operation is successful, false otherwise
    */
    bool getBasisFunctions(std::vector<Eigen::MatrixXd>& basisFunctions);

	/**
    * Compute the positive semi-definite matrix of the quadratic control cost.
	* More precisely it is a vector of matrix: a matrix for each dimension and 
	* the dimensions are coherent with the number of theta parameters used for 
	* each dimension. In this case such control cost matrix is just an identity
	* matrix.
    * @param controlCosts is a vector of control costs matrices, one for each parameter.
	* @return true if operation is successful, false otherwise
    */
    bool getControlCosts(std::vector<Eigen::MatrixXd>& controlCosts);

	/**
    * Performs the parameters time independent updates vector starting from
	* the parameters updates for each time steps.
    * @param updates is a vector of updates matrices (one updates vector for each time step)
	* @return true if operation is successful, false otherwise
    */
    bool updateParameters(const std::vector<Eigen::MatrixXd>& updates);

	/**
    * Extracts from the dmp the theta parameters vectors.
    * @param parameters is a vector of parameters vectors (one for each dimension)
	* @return true if operation is successful, false otherwise
    */
    bool getParameters(std::vector<Eigen::VectorXd>& parameters);

	/**
    * Sets the theta parameters vectors for the dmp
    * @param parameters is a vector of parameters vectors (one for each dimension)
	* @return true if operation is successful, false otherwise
    */
    bool setParameters(const std::vector<Eigen::VectorXd>& parameters);


private:

	///indicates if the object is initialized or not
    bool initialized_;

	///number of time steps for reinforcement learning
    int numTimeSteps_;

	//pointer to a dmp object containing all the data representing a dmp
    boost::shared_ptr<dmp::DynamicMovementPrimitive> dmp_;
};

}

#endif /* DMP_POLICY_H_ */
