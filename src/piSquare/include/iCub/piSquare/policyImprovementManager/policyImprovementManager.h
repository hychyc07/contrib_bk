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

/**
 * 
 *
 * \defgroup piSquare piSquare
 * @ingroup icub_contrib_libraries

 * Classes implementing Policy Improvement with Path Integrals (PI^2) algorithm
 * by exploiting Dynamic Movement Primitives (DMPs) as parameterized policies.
 *
 * 
 * \section intro_sec Description
 * 
 * piSquare library provides a solution of a general Reinforcement Learning problem
 * through PI^2 algorithm as search strategy and DMPs as parameterized policies.
 * 
 * A DMP is a second order dynamical system which allows to realize a desired time
 * behaviour of the state variable from a fixed start value to a fixed goal value.
 * The time behaviour is the result of the cooperation of two therms: a linear
 * asymptotically dynamics from start to goal positions modulated by a parameterized
 * non linear function. DMPs evolution are not directly time dependent: in order to express
 * the DMP "time" evolution a first order asymptotically stable system is used. It is
 * called phase or canonical system and it just provides a negative exponential trend
 * from 1 to 0 while time variable goes from 0 to 1.
 * 
 * A first distinction between the concept of "trajectory" optimized by PI^2 and the
 * concept of "state" of the given reinforcement learning problem has to be made.
 * The therm "trajectory" refers to a desired sequence of values for the variables
 * representing the physical system. Such sequence is encoded into DMPs as a parameterized
 * function, and the optimization is performed in therms of such parameters.
 * The therm "state" represents the state space characterizing the particular reinforcement
 * learning problem to be faced. It is a variable measured through sensors after each
 * movement of the physical system.
 * 
 * As an example, think to a robotic arm which has to grasp a ball.
 * The DMPs encode a reference position or velocity "trajectory" to be performed by the arm,
 * expressed as a parametric function as mentioned before. Then such trajectory has
 * to be tracked by a controller which provides the actual input values for the motors.
 * Now the interaction between the robotic hand and the environment comes. During the motion
 * some sensors, e.g. tactile sensors, try to obtain information about the "state" of the
 * grasp, for example which is the pressure imposed by the hand on the ball over time.
 * The outcome of the motion is evaluated in therms of the state information, for example
 * if the pressure becomes bigger then a certain threshold for a sufficient time interval
 * then one could say that the grasp was successful.
 * 
 * Thus, summarizing: the DMP parameters are the variables to be optimized with respect
 * to a given cost functional representing the task; the DMPs parameters are used to generate
 * a reference trajectory in therms of position/velocity values to be performed by the
 * robotic system; the state is measured during the actual motion in order to realize if
 * the task was successful or not.
 *
 * The trajectory provided by the DMPs at each iteration, from a given start state to a given goal state,
 * is optimized with respect to a cost functional representing the task to be faced.
 * PI^2 algorithm represents a local approach to the reinforcement learning problem,
 * i.e. it performs a local search for better trajectories starting from an initial trajectory.
 * Usually such initial trajectory is learned by demonstration, and this explains why
 * both start and goal positions are supposed to be known.
 * 
 * The initial values for the optimization parameters are extracted from the initial
 * trajectory through regression and better trajectories are looked for by modifying the
 * parameters values and executing the resulting trajectories, also called rollouts.
 * More precisely, a number of "noisy" rollouts is performed by adding some noise to
 * the current parameters vector. Then the rollouts are executed and evaluated with
 * the cost functional definition. Finally the updating rule of PI^2 computes the new
 * parameters vector considering the executed rollouts and the cost they received.
 * The updating rule originates from stochastic optimal control theory, but (very!)
 * roughly speaking one can think about it as a sort of weighted average among the
 * rollouts: the contribution of a single rollout is higher if it received a lower cost
 * and viceversa.
 * 
 * From a software organization perspective, the library contains all the methods for
 * both PI^2 algorithm implementation and DMPs management in a general way, without referring
 * to a particular application. Then, every application which needs to use the library
 * has to provide the definition of the task through the implementation of a method
 * describing how the cost functional is evaluated.
 *
 *
 * \section lib_sec Libraries
 *
 * Yarp, Eigen, Boost.
 *
 *
 * \section parameters_sec Parameters
 *
 * <b>Constructor Parameters</b> 
 * The following parameters are required to instantiate the main class constructor.
 * These parameters refer to the task definition, which is provided by an external
 * module/application using the library by directy calling the main class constructor.
 *
 * - rf \n 
 * Resource Finder object for extracting and handling parameters read from configuration file
 *
 * - movementDuration \n
 * The time duration of the physical movement to be optimized
 *
 * - start \n
 * Initial values for the trajectory encoded into the DMP
 *
 * - goal \n
 * Final values for the trajectory encoded into the DMP
 *
 *
 * <b>Configuration File Parameters </b>
 * The following key-value pairs can be specified as parameters in the configuration file.
 * These values refer to PI^2 and DMPs implementation settings.
 *
 * - num_time_steps \n
 * Number of steps used by the reinforcement learning algorithm. Once a trajectory has been
 * encoded into a DMP, it is represented by the DMP parameters vector, no matter what the
 * trajectory time duration was. For this reason, the num_time_steps parameter is used to
 * generate trajectory for reinforcement learning starting from DMP parameters.
 *
 * - num_trials	\n
 * Number of trajectory improvement. Each trial consists in executing a noiseless trajectory,
 * i.e. a trajectory whose parameters has been optimized using a certain number of noisy rollouts.
 *
 * - num_rollouts \n
 * Number of noisy trajectories used to improve the current DMPs parameters vectors (a parameter
 * vector for each problem dimension). Each rollouts is obtained by adding gaussian noise to the
 * current parameters values.
 *
 * - num_reused_rollouts \n
 * Number of rollouts kept from the previous iteration. Instead of generating the whole set of
 * rollouts at each iteration, it is possible to keep a certain number of previously generated
 * rollouts, namely the best ones. This parameter represent the classical tradeoff in reinforcement
 * learning between exploration and exploitation.
 *
 * - variance \n
 * The exploration noise variance. If a unique value is specified, then it is assumed to be the
 * variance value for each dimension. Otherwise, there must be a value for each problem dimension.
 * Since the noise has a gaussian distribution with a zero mean,
 * the noise variance is the only parameter used to affect the exploration noise magnitude. The
 * more is the variance, the more the rollouts parameters are different with respect to the current
 * noiseless parameters. Variance parameter affects speed and convergence properties of the algorithm.
 *
 * - control_cost_weight \n
 * The influence of the internal control cost with respect to the whole cost, i.e. state + control.
 * The internal control is referred to be the parameter vector of each dmp, and the way its cost
 * is evaluated is actually a quadratic form multiplied by the control_cost_weight parameter. Thus
 * such control has not a physical interpretation, it is just the amplitude of DMP parameters used
 * to encode a certain trajectory.
 *
 * - noise_decay \n
 * The scaling factor for the noise variance at each iteration. It ensures the algorithm convergence
 * by reducing the noise influence along the time.
 *
 * - delta_t \n
 * Time interval used for trajectories discretization.
 *
 * - dmp_gain \n
 * Weight of the asymptotically stable part of the DMP with respect to the modulating non linear function.
 * By default, in order to ensure system stability, the non linear function influence becomes lower and lower
 * while the system evolves. In the limit case, at the final step, the non linear function is not considered
 * anymore. This parameter is a constant multiplying the linear part of the dynamical system equation, thus
 * it affects the way the non linear function becomes less important during time. For example, for high values
 * (e.g 50) the linear part is so strong that the non linear function can modulate the trajectory behaviour
 * only in the first half of the trajectory itself.
 *
 * - num_rfs \n
 * Number of receptive fields/basis functions used inside each DMP to represent the nonlinear function.
 * Since the optimization parameters are the weights multiplying each basis function, this parameter
 * directly affects the dimensions of the parameters vectors (one for each dimension).
 *
 * - rfs_width_boundary \n
 * How much each basis function center is far from the neighbours. From another perspective, it represents
 * when a basis function intersects with the previous and the following one. This parameter affects the
 * basis functions bandwidth.
 *
 * - can_sys_cutoff
 * The cutoff of the DMP canonical system.It determines when the DMP trajectory evolution stops because
 * the phase variable (i.e. the canonical system state variable) has become so small to be considered
 * as equal to zero.
 *
 *
 * \section tested_os_sec Tested OS
 * 
 * The library is currently under test.
 *

 * \section example_sec Example
 *
 * piSquareDemo code provides a trivial example of how to use the library.
 *

 * \note <b>Release status</b>:  this library is currently under development!
 * Date: first draft may 2012
 *
 * \author Andrea Ceroni
 **/

#ifndef POLICY_IMPROVEMENT_MANAGER_H_
#define POLICY_IMPROVEMENT_MANAGER_H_


#define EIGEN2_SUPPORT
#include <Eigen/Eigen>

#include <boost/shared_ptr.hpp>

#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <iCub/piSquare/policyLibrary/dmpPolicy.h>
#include <iCub/piSquare/policyImprovement/policyImprovement.h>
#include <iCub/piSquare/dmpMotionGeneration/mathHelper.h>
#include <iCub/piSquare/dmpMotionGeneration/constants.h>

using namespace Eigen;
using namespace pi2;
using namespace library;
using namespace yarp::os;
using namespace yarp::sig;


/**
* \ingroup piSquare
*
* Base class providing an high level management of PI^2 main loop. It must be
* derived by any external module attempting to use PI^2 algorithm for its purposes.
*/

class PolicyImprovementManager
{

public:

	/**
    * class constructor
    * @param rf is a resource finder object pointer for parameters management
	* @param movementDuration is the time duration of the movement whose trajectory
	* has to be optimized through reinforcement learning
	* @param start is the start positions vector
	* @param goal is the goal positions vector
	* @param averagedCost is a flag indicating if the cost related to the current noise-less trial has to be averaged
	* over a certain number of identical trials or not
    */
    PolicyImprovementManager(ResourceFinder* rf, double movementDuration, Vector& start, Vector& goal, bool averagedCost = false);

	/**
    * class constructor
    * @param rf is a resource finder object pointer for parameters management
	* @param movementDuration is the time duration of the movement whose trajectory
	* has to be optimized through reinforcement learning
	* @param dimensions is the number of dimensions for the application
	* @param initialTrajectory is an already initialized trajectory used as initial trajectory for optimization
	* @param averagedCost is a flag indicating if the cost related to the current noise-less trial has to be averaged
	* over a certain number of identical trials or not
    */
	PolicyImprovementManager(ResourceFinder* rf, double movementDuration, int dimensions, dmp::Trajectory& initialTrajectory, bool averagedCost = false);

	/**
    * class destructor
    */
    virtual ~PolicyImprovementManager();

	/**
    * initializes all those parameters whose values has to be fetched from configuration file.
	* @return true if initialization is successful, false otherwise
    */
    bool initialize();

	/**
    * performs PI^2 algorithm exploiting all the underlying software for dmp management and
	* policy parameters improvement. The main loop consists in realizing a set of rollouts,
	* evaluating their costs, and searching for better rollouts using the parameters updating
	* rule provided by PI^2 algorithm itself.
	* @return true if operation is successful, false otherwise
    */
    bool runPI2();

	/**
    * assigns a vector cost (one element for each time step) to the trajectory obtained from the current
	* values of theta parameters. Firstable the dmp is "propagated" with the proper time duration and
	* number of time steps in order to obtain a trajectory. Then the cost of such trajectory is computed.
    * @param parameters is a vector of parameters vectors, one for each dimension
	* @param costs is the cost vector associated to the rollout, a cost for each time step
	* @param trajectoryOut is the trajectory associated to the rollout (i.e. parameters)
	* @return true if operation is successful, false otherwise
    */
	bool rollout(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, dmp::Trajectory& trajectoryOut);

	/**
    * assigns a cost to an input trajectory. This function is what specializes a task from another one, thus
	* it must be implemented by every external software before being able to use the library.
    * @param inputTrajectory is the trajectory whose cost has to be evaluated
	* @param outputCosts is the output cost vector for the given trajectory
    */
    virtual bool computeCost(dmp::Trajectory& inputTrajectory, Vector& outputCosts) = 0;

protected:

	///flag indicating if the object has been initialized or not
    bool initialized_;

	///resource finder object for parameters management
	ResourceFinder* rf_;

	boost::shared_ptr<DMPPolicy> policy_;

	///number of dimensions of the reinforcement learning problem
	int numDimensions_;

	///time duration of the movement to be optimized
	double movementDuration_;

	///initial position of the movement
	yarp::sig::Vector startPositions_;

	///final requested position of the movement
	yarp::sig::Vector goalPositions_;

	///number of time steps used in reinforcement learning
	int numTimeSteps_;

	///discretization step length
	double dt_;

	/**
	* sampling frequency of the movement, it depends on both the movement time duration
	* and the number of time steps characterizing the reinforcement learning algorithm
	*/
	double samplingFrequency_;

	///number of noiseless trajectories executions
    int numTrials_;

	///number or noisy rollouts for each trial
    int numRollouts_;

	///number of rollouts kept from the previous trial (the bests)
	int numReusedRollouts_;

	///variance vector of the exploration noise
    std::vector<double> variance_;

	///scalar for noise reduction at each trial
	double noiseDecay_;

	///scalar for weighting the control cost contribution in the whole cost functional
	double controlCostWeight_;

	/// initial trajectory, used when starting from an already initialized trajectory passed as parameter
	dmp::Trajectory initialTrajectory_;

	/// the best trajectory over the whole set of trials
	dmp::Trajectory bestTrajectory_;

	/// the lower cost over the whole set of trials
	double bestCost_;

	/// the trial in which the best trajectory occourred
	int bestTrajectoryIndex_;

	/// cost trend over trials
	VectorXd costTrend_;

	/// flag indicating if the cost related to the noise-less trial has be computed as an average over n trials
	bool averagedCost_;

	/**
	* Stores the cost trend over trials as a .txt file
	* @param fileName is the path of the file to be written
	* returns true if the operation is successful, false otherwise
	*/
	bool writeCostTrendToFile(std::string fileName);

};

const int NUM_SAMPLE_TRIALS = 5;


#endif /* POLICY_IMPROVEMENT_MANAGER_H_ */
