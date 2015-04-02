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

#ifndef LWR_H_
#define LWR_H_

#include <vector>
#include <string>

#define EIGEN2_SUPPORT
#include <Eigen/Eigen>

#include <yarp/os/ResourceFinder.h>

namespace lwr
{

/**
* \ingroup piSquare
*
* Class solving a regression problem by using locally weighted regression (LWR) approach.
*/

class LocallyWeightedRegression
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
    * class constructor
    */
    LocallyWeightedRegression();
    
	/**
    * class destructor
    */
	~LocallyWeightedRegression();

	/**
    * Generates the parameters which characterize each radial function. They can be equispaced
	* or logarithmic located. Such issue comes from the fact that the canonical system phase
	* variable (representing time) has a negative exponential behaviour.
    * @param numRfs is the number of basis functions used for regression
	* @param activation indicates "how much far" each gaussian is with respect to the next
	* and the previous one, i.e. at which distance the gaussians intersect with their neighbours
	* @param exponentiallySpaced says if rfs are exponentially spaced or not
	* @param canSysCutoff affect the location of the gaussian functions centers
	* @return true if operation is successful, false otherwise
    */
    bool initialize(const int numRfs, const double activation, const bool exponentiallySpaced, const double canSysCutoff);

	/**
    * implementation of the locally weightet regression (lwr) algorithm.
	* The kernel functions of the model are gaussians.
	* This function represents the final step of the dmp parameters learning process.
    * @param xInputVector is the function domain vector
	* @param yTargetVector is the labels vector
	* @return true if operation is successful, false otherwise
    */
    bool learnWeights(const Eigen::VectorXd& xInputVector, const Eigen::VectorXd& yTargetVector);

	/**
    * given the x input value of the function domain, it predicts the function value
	* as a weighted sum of gaussian functions.
    * @param xQuery is the input value of the function domain to be predicted
	* @param yPrediction is the prediction value
	* @return true if operation is successful, false otherwise
    */
    bool predict(const double xQuery, double& yPrediction);

	/**
    * generates the basis function matrix (for a single dimension)
    * @param xInputVector is the function domain vector
	* @param basisFunctionMatrix is the output basis function matrix, a column for each basis function
	* @return true if operation is successful, false otherwise
    */
    bool generateBasisFunctionMatrix(const Eigen::VectorXd& xInputVector, Eigen::MatrixXd& basisFunctionMatrix);

	/**
    * gets the thetas parameters
    * @param thetas is the output parameters vector
	* @return true if operation is successful, false otherwise
    */
    bool getThetas(Eigen::VectorXd& thetas);

	/**
    * sets the thetas parameters
    * @param thetas is the parameters vector to be set
	* @return true if operation is successful, false otherwise
    */
    bool setThetas(const Eigen::VectorXd& thetas);

	/**
    * updates the theta vectors (thetas += deltaThetas)
    * @param deltaThetas is the parameter updates vector
	* @return true if operation is successful, false otherwise
    */
    bool updateThetas(const Eigen::VectorXd& deltaThetas);

	/**
    * gets the parameters of the basis functions
    * @param widths is the bandwidth vector, an element for each function
	* @param centers is the means vector, an element for each function
	* @return true if operation is successful, false otherwise
    */
    bool getWidthsAndCenters(Eigen::VectorXd& widths, Eigen::VectorXd& centers);

	/**
    * gets the number of basis functions
    * @param numRfs is the number of the basis functions used for regression
	* @return true if operation is successful, false otherwise
    */
    bool getNumRFS(int& numRfs);

private:

	/**
    * computes the value for a given kernel index and a given domain value
    * @param xInput is the input domain value
	* @param centerIndex is the kernel index
	* @return the value of the kernel
    */
    double getKernel(const double xInput, const int centerIndex);

    bool initialized_;

    ///Number of receptive fields used in this LWR model
    int numRfs_;
	
	/**
    * determines whether gaussians are exponentially spaced (true) or equally spaced (false)
    * exponential scale is used to deal with the nonlinearity of the phase variable of the 
	* canonical system of a DMP. This member is actually not used, the related information
	* is just passed as a parameter to the initialize function.
	*/
    bool exponentially_spaced_;

    //Centers and bandwidth of the receptive fields

    Eigen::VectorXd centers_;
    Eigen::VectorXd widths_;

    //Slopes of the local linear approximations
    Eigen::VectorXd thetas_;
};

}

#endif /* LWR_H_ */
