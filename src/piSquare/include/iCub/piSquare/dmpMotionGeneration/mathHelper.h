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

#ifndef MATH_HELPER_H_
#define MATH_HELPER_H_

#include <string>
#include <sstream>

#include <Eigen/Eigen>

#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <iCub/piSquare/dmpMotionGeneration/trajectory.h>

namespace dmp {

/**
* \ingroup piSquare
*
* Class whose static members are used to generate minimum jerk trajectories
* from a start position to a goal position.
*/

class MathHelper
{

public:

	/**
    * Computes the minimum jerk trajectory starting from start and ending to goal. The length of such
	* trajectory is computed starting from the duration and the delta_t passed as parameters. The
	* trajectory duration (in terms of time) is updated each time a point is added to the trajectory.
    * @param start is the start positions vector
	* @param goal is the goal positions vector
	* @param duration is the time duration of the movement
	* @param deltaT is the time step used for discretization
	* @param trajectory is the minimum jerk trajectory to be generated
	* @return true if operation is successful, false otherwise
    */
    static bool generateMinJerkTrajectory(const yarp::sig::Vector &start, const yarp::sig::Vector &goal, const double duration,
										  const double deltaT, Trajectory &trajectory);

	/**
	* Converts a yarp vector in an eigen vector
	* @param yarpVector is the input vector in the yarp format
	* @param eigenVector is the output vector in the eigen format
	* @return true if operation is successful, false otherwise
	*/
	static bool yarpToEigenVector(const yarp::sig::Vector& yarpVector, Eigen::VectorXd& eigenVector);

	/**
	* Converts an eigen vector in a yarp vector
	* @param eigenVector is the input vector in the eigen format
	* @param yarpVector is the output vector in the yarp format
	* @return true if operation is successful, false otherwise
	*/
	static bool eigenToYarpVector(const Eigen::VectorXd& eigenVector, yarp::sig::Vector& yarpVector);


private:

	/**
    * calculates a step of the minimum jerk trajectory by computing the coefficient of the
	* minimum jerk polynomial, i.e. the solution to a generic minimum jerk optimization problem.
    * @param start is the start positions vector
	* @param goal is the goal positions vector
	* @param duration is the time duration of the movement
	* @param deltaT is the time step used for discretization
	* @param next is the next minimum jerk trajectory element to be generated
	* @return true if operation is successful, false otherwise
    */
    static bool calculateMinJerkNextStep(const yarp::sig::Vector &start, const yarp::sig::Vector &goal, const double duration,
										 const double deltaT, yarp::sig::Vector &next);

	// Converts a yarp matrix in an eigen matrix
	// @param yarpMatrix is the input matrix in the yarp format
	// @param eigenMatrix is the output matrix in the eigen format
	// @return true if operation is successful, false otherwise
	//
	//bool yarpToEigenMatrix(const yarp::sig::Matrix& yarpMatrix, Eigen::MatrixXd& eigenMatrix);

	
	// Converts an eigen matrix in a yarp matrix
	// @param eigenMatrix is the input matrix in the eigen format
	// @param yarpMatrix is the output matrix in the yarp format
	// @return true if operation is successful, false otherwise
	//
	//bool eigenToYarpMatrix(const Eigen::MatrixXd& eigenVector, yarp::sig::Matrix& yarpMatrix);

};

}

#endif /* MATH_HELPER_H_ */
