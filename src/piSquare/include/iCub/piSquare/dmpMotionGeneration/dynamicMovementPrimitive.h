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

#ifndef DYNAMIC_MOVEMENT_PRIMITIVE_H_
#define DYNAMIC_MOVEMENT_PRIMITIVE_H_

#include <string>

#define EIGEN2_SUPPORT
#include <Eigen/Eigen>

#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <iCub/piSquare/locallyWeightedRegression/lwr.h>
#include <iCub/piSquare/locallyWeightedRegression/lwrParameters.h>
#include <iCub/piSquare/dmpMotionGeneration/dmpParameters.h>
#include <iCub/piSquare/dmpMotionGeneration/trajectory.h>
#include <iCub/piSquare/dmpMotionGeneration/transformationSystem.h>


namespace dmp
{

//forward declaration
class TransformationSystem;

/**
* \ingroup piSquare
*
* Class containing methods and data structures to implement Dynamic Movement Primitives (DMPs) used as
* parameterized policies by PI^2 algorithm.
*/

class DynamicMovementPrimitive
{

public:

	/**
    * Class constructor, it assigns the resource finder object both to itself and to the inner Parameters object.
    * @param rf is a resource finder object for parameters management
    */
	DynamicMovementPrimitive(yarp::os::ResourceFinder* rf);
    
	/**
    * class destructor
    */
	~DynamicMovementPrimitive();

	/**
    * Two temporary Parameters objects are created, one for the dmp parameters and one for those of
	* locally weighted regression (lwr). Such objects are then passed to another initialization
	* function which manages all the "low level" initializations.
    * @param numTransformationSystem
	* @return true if operation is successful, false otherwise
    */
    bool initialize(int numTransformationSystems);

	/**
    * dmp initialization is completed here by realizing the following steps:
	* Parameters are saved inside the dmp;
	* A number of transformation systems equal to the problem dimensions is created (however 
	* they all refers to the same dmp);
	* Such transformation systems are initialized using the parameters holded inside lwrParams,
	* and these parameters themselves are used for the initialization of the lwr object inside
	* each transformation system;
	* The previous parameters are used to create the gaussian function for regression
    * @param numTransformationSystems
	* @param dmpParams contains the parameters for the dmp
	* @param lwrParams contains the parameters for locally weighted regression (lwr)
	* @return true if operation is successful, false otherwise
    */
    bool initialize(int numTransformationSystems, Parameters dmpParams, lwr::Parameters lwrParams);

	/**
    * checks of the object is initialized or not
	* @return true if the object is initialized, false otherwise
    */
    bool isInitialized() const;

	/**
    * perform parameters reinitialization by calling just the initialization routine
	* inside the Parameters objects. No other initializations are performed
	* @return true if operation is successful, false otherwise
    */
    bool reInitializeParams();

	/**
	* The input trajetory must be fitted in the dmp, i.e. it must be represented by
	* using the modulating nonlinear function f, whose influence is incrementally
	* reduced while approaching the end time (and this fact guarantees the stability
	* of the system to the attractor point defined by the goal state).
	* Once it has be done, the dmp parameters are computed through regression.
    * @param trajectory is the input trajectory to be learned
	* @return true if operation is successful, false otherwise
    */
    bool learnFromTrajectory(const Trajectory& trajectory);

	/**
    * initializes the dmp trajectory by learning a minimum jerk trajectory. The start positions and
	* goal positions parameters are used both for creating the minimum jerk movement and for setting
	* start and goal state inside the dmp itself (in particular, inside the transformation system).
	* deltaT parameter is used to obtain the frequency used to sample the minimum jerk trajectory.
	* The trajectory object is completely separated from the concept of initial and goal state: it
	* just contains values for position, velocity and acceleration of the trajectory it represents.
	* Start and goal instead are managed by the transformation system, and in any case in principle
	* they have not to be equal to those passed as parameters in this function: they are obtained by 
	* watching the initial and final position values of the trajectory.
    * @param start is the trajectory starting point (one for each dimension) 
	* @param goal is the trajectory goal point (one for each dimension)
	* @param duration is the time duration of the trajectory
	* @param deltaT is the time interval for discretization
	* @return true if operation is successful, false otherwise
    */
    bool learnFromMinJerk(const yarp::sig::Vector& start, const yarp::sig::Vector& goal, const double duration, const double deltaT);

	/**
    * initializes the dmp trajectory by learning an arbitrary trajectory, given as parameter.
	* The trajectory is supposed to contain only the "position" trajectory, while first and
	* second derivatives (i.e. velocity and acceleration) are computed by derivation.
    * @param trajectory is the custom trajectory to be learned
	* @return true if operation is successful, false otherwise
    */
    bool learnFromCustomTrajectory(dmp::Trajectory& trajectory);


	/**
    * assigns thetas parameters without computing them from an input trajectory
    * @param thetas are the thetas parameters for each dimension
	* @param start is the start positions vector
	* @param goal is the goal positions vector
	* @param duration is the trajectory time duration
	* @param deltaT is the discretization time step
	* @return true if operation is successful, false otherwise
    */
    bool learnFromThetas(const std::vector<Eigen::VectorXd>& thetas, const Eigen::VectorXd& initialStart, const Eigen::VectorXd& initialGoal,
                         const double samplingFrequency, const double initialDuration);

	/**
    * stores a vector representing a trajectory as a .txt file
    * @param trajectory is the trajectory to be stored
	* @param fileName is the path of the file containing the trajectory
	* @return true if operation is successful, false otherwise
    */
	bool writeVectorToFile(std::vector<double> trajectory, std::string fileName);

	/**
    * gets the trajectory time duration
    * @param initialDuration is the output time duration
	* @return true if operation is successful, false otherwise
    */
    bool getInitialDuration(double& initialDuration);

	/**
    * gets initial start positions
    * @param initialStart is the initial start positions vector
	* @return true if operation is successful, false otherwise
    */
    bool getInitialStart(Eigen::VectorXd& initialStart);

	/**
    * gets initial goal positions
    * @param initialGoal is the initial goal positions vector
	* @return true if operation is successful, false otherwise
    */
    bool getInitialGoal(Eigen::VectorXd& initialGoal);

	/**
    * gets goal positions
    * @param initialGoal is the goal positions vector
	* @return true if operation is successful, false otherwise
    */
    bool getGoal(Eigen::VectorXd& initialGoal);

	/**
    * gets the parameters, a vector for each dimension
    * @param thetas is a vector of parameters vectors
	* @return true if operation is successful, false otherwise
    */
    bool getThetas(std::vector<Eigen::VectorXd>& thetas);

	/**
    * set the parameters, a vector for each dimension
    * @param thetas is a vector of parameters vectors
	* @return true if operation is successful, false otherwise
    */
    bool setThetas(const std::vector<Eigen::VectorXd>& thetas);

	/**
    * gets the parameters characterizing the basis functions used
	* for lwr, i.e. centers and bandwidth
    * @param widths are the gaussian functions bandwidth
	* @param centers are the gaussian functions centers/means
	* @return true if operation is successful, false otherwise
    */
    bool getWidthsAndCenters(std::vector<Eigen::VectorXd>& widths, std::vector<Eigen::VectorXd>& centers);

	/**
    * gets the parameters characterizing a given basis function 
    * @param widths is the gaussian function bandwidth
	* @param centers is the gaussian function center/mean
	* @return true if operation is successful, false otherwise
    */
    bool getWidthsAndCenters(const int transSystemIndex, Eigen::VectorXd& widths, Eigen::VectorXd& centers);

	/**
    * gets the number of basis functions for a give transformation system
    * @param transId is the id of the transformation system
	* @param numRfs is the numbero of rfs
	* @return true if operation is successful, false otherwise
    */
    bool getNumRFS(const int transId, int& numRfs);

	/**
    * gets the number of basis functions for every transformation system
    * @param numRfs is a vector holding the number of rfs for each transformation system
	* @return true if operation is successful, false otherwise
    */
    bool getNumRFS(std::vector<int>& numRfs);

	/**
    * For each dimension, it creates a matrix representing the values of each basis function
	* assumed for each time step of the trajectory. Thus the vector length is equal to the
	* number of dimensions, and each element is a matrix whose dimensions are equal to the
	* number of time steps times the number of basis functions. In other words, each element
	* represents the value for a particular basis function, at a given time step, for a
	* specific dimension.
	* The lwr model is used to generate such values.
    * @param numTimeSteps is the number of time steps usef for reinforcement learning
	* @param basisFunctions are the resulting basis function sets, one for each dimension
	* @return true if operation is successful, false otherwise
    */
    bool getBasisFunctions(const int numTimeSteps, std::vector<Eigen::MatrixXd>& basisFunctions);

	/**
    * takes info about start and goal positions in order to use them for setting up the dmp
    * @param samplingFrequency represents the sampling frequency of the motion
	* @return true if operation is successful, false otherwise
    */
    bool setup(const double samplingFrequency);

    /**
    * setup without considering start position
    * @param goal is the goal positions vector
	* @param movementDuration is the time duration of the movement
	* @param samplingFrequency is the sampling frequency of the movement
	* @return true if operation is successful, false otherwise
    */
    bool setup(const Eigen::VectorXd& goal, const double movementDuration = -1.0, const double samplingFrequency = -1.0);

	/**
    * Some set up instructions assuming that the dmp has already received an initialization
    * @param start is the start positions vector
	* @param goal is the goal positions vector
	* @param movementDuration is the time duration of the movement
	* @param samplingFrequency is the sampling frequency of the movement
	* @return true if operation is successful, false otherwise
    */
    bool setup(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, const double movementDuration = -1.0, const double samplingFrequency = -1.0);

	/**
    * changes the goal positions vector
    * @param goal is the new goal vector (it may involve just a part of the original goal vector)
	* @param startIndex is the first transformation system affected by the change
	* @param endIndex is the last transformation system affected by the change
	* @return true if operation is successful, false otherwise
    */
    bool changeGoal(const Eigen::VectorXd& goal, const int startIndex, const int endIndex);
    
	/**
    * changes the goal for a specific transformation system
    * @param newGoal is the new goal position
	* @param index represents the affected transformation system
	* @return true if operation is successful, false otherwise
    */
	bool changeGoal(const double newGoal, const int index);

	/**
    * changes the start positions vector
    * @param start is the new start positions vector
	* @return true if operation is successful, false otherwise
    */
    bool changeStart(const Eigen::VectorXd& start);

	/**
    * indicates if the object has been set up or not
	* @return true if the object has been set up, false otherwise
    */
    bool isSetup() const;

	/**
    * indicates if start positions have been set
	* @return true if start position have been set, false otherwise
    */
    bool isStartSet() const;

	/**
    * declares the start positions ase unset
	* @return true if operation is successful, false otherwise
    */
    void unsetStart();

	/**
    * set the trajectory time duration. Relations between time and frequency must be coherent
    * @param movementDuration is the time duration of the movement
	* @param samplingFrequency is the movement sampling frequency
	* @return true if operation is successful, false otherwise
    */
    bool setDuration(const double movementDuration, const int samplingFrequency);

	/**
    * propagates the transformation system step by step, and at each time the current values (positio, velocity, acceleration) 
	* are added to the output trajectory.
	* The duration of the DMP need to be set previously using one of the setup functions. The sampling duration and the number 
	* of samples specified determine the length of the trajectory and its sampling frequecy.
    * @param trajectory is the trajectory obtained through system integration
	* @param samplingDuration is the duration of trajectory sampling
	* @param numSamples is the number of samples required
	* @return true if operation is successful, false otherwise
    */
    bool propagateFull(Trajectory& trajectory, const double samplingDuration, const int numSamples);

	/**
    * integrates the transformation system in order to compute its value for the next step. It just
	* call its overloaded version.
    * @param desiredCoordinates
	* @param movementFinished is a flag indicating if the movement is finished or not
	* @return true if operation is successful, false otherwise
    */
    bool propagateStep(Eigen::VectorXd& desiredCoordinates, bool& movementFinished);
    
	/**
    * integrates the system by using the underlying lwr model, and then stores the obtained values of
	* y, yd, ydd (i.e. current position, velocity, acceleration) into the output desired coordinates.
    * @param desiredCoordinates
	* @param movementFinished is a flag indicating if the movement is finished or not
	* @param samplingDuration is the duration of the trajectory sampling
	* @param numSamples is the number of samples required
	* @return true if operation is successful, false otherwise
    */
	bool propagateStep(Eigen::VectorXd& desiredCoordinates, bool& movementFinished, const double samplingDuration, const int numSamples);

	/**
    * gets the current position of the transformation system
    * @param currentDesiredPosition is the output desired position
	* @param startIndex is the firt transformation system involved
	* @param endIndex is the last transformation system involved
	* @return true if operation is successful, false otherwise
    */
    bool getCurrentPosition(Eigen::VectorXd& currentDesiredPosition, const int startIndex, const int endIndex);

	/**
    * gets the time progress of the canonical system
	* @return the current progress of the canonical system
    */
    double getProgress() const;

	/**
    * gets the number of transformation systems
	* @return the number of the transformation systems
    */
    int getNumTransformationSystems() const;

	/**
    * checks if the time evolution of the transformation system is increasing or not,
	* i.e if the start position is lower then the goal position.
    * @param transformationSystemIndex is the transformation system involved
	* @param isIncreasing is not used
	* @return true if transformation system is increasing, false otherwise
    */
    bool isIncreasing(int transformationSystemIndex, bool& isIncreasing);

	/**
    * Evaluates the canonical system using its duration (tau) and alphaX at numTimeSteps time steps going from 0 to 1.
	* arrange the deltaT considering both the time steps number of the trajectory and the duration of the movement 
	* encoded into the dmp. Then it integrate the canonycal system to get an exponentially decreasing trend.
    * @param numTimeSteps is the number of time steps used for reinforcement learning
	* @param canSystemVector is the output vector representing the canonical system values
	* @return true if operation is successful, false otherwise
    */
    bool getCanonicalSystem(const int numTimeSteps, Eigen::VectorXd& canSystemVector);


private:

    //the structure that contains the states of the canonical system
    typedef struct
    {
        double x;
        double time;
    } DMPCanonical;

	/**
    * performs internal debug initializations and memory allocations
    */
	void initialize();
    
	/**
    * considering just a step, it computes the value(s) for the nonlinear target function
	* needed to encode the input trajectory into the dmp.
	* @return true if operation is successful, false otherwise
    */
	bool integrateAndFit();

	/**
    * Values for theta parameters are learned through regression of the target nonlinear function conteined
	* inside the transformation systems.
	* @return true if operation is successful, false otherwise
    */
    bool learnTransformationTarget();

	/**
    * integrates the transformation system by firstly predicting the values of the nonlinear
	* component f_ from the knowledge of the dmp parameters.
    * @param dtTotal is the time step duration
	* @param numIteration
	* @return true if operation is successful, false otherwise
    */
    bool integrate(const double dtTotal, const int numIteration);

	/**
    * Evaluates the canonical system at the given time step (canonicalSystemTime) and writes it to canonicalSystemX
    * @param canonicalSystemX is the value of the canonical system at the given time instant
	* @param canonicalSystemTime is the given time instant
    */
    void integrateCanonicalSystem(double& canonicalSystemX, const double canonicalSystemTime) const;

	/**
    * reset the canonical system to the default configuration, i.e state equal to 1 and time equal to 0
    */
    void resetCanonicalState();

	///flag indicating if the object is initialized or not
    bool initialized_;

	///resource finder for parameters management
    yarp::os::ResourceFinder* rf_;

	///parameters that are read from file, same for all DMPs
    Parameters params_;

    ///struct that contains the variables of the canonical system (the phase variable x and time)
    DMPCanonical canonicalSystem_;

    ///vector containing relevant information for each dimension of the movement system.
    std::vector<TransformationSystem> transformationSystems_;

	///input for the locally weighted regression algorithm used to find the values for theta parameters
    std::vector<double> trajectoryTargetFunctionInput_;

	///just for debug, not useful
    Eigen::VectorXd debugTrajectoryPoint_;

};

// inline functions follow
inline bool DynamicMovementPrimitive::isInitialized() const
{
    return initialized_;
}
inline bool DynamicMovementPrimitive::isSetup() const
{
    return params_.isSetup_;
}
inline bool DynamicMovementPrimitive::isStartSet() const
{
    return params_.isStartSet_;
}
inline void DynamicMovementPrimitive::unsetStart()
{
    params_.isStartSet_ = false;
}

inline void DynamicMovementPrimitive::resetCanonicalState()
{
    canonicalSystem_.x = 1.0;
    canonicalSystem_.time = 0.0;
    params_.numSamples_ = 0;
}
inline bool DynamicMovementPrimitive::getInitialDuration(double& initialDuration)
{
    if (!initialized_)
    {
        return false;
    }
    initialDuration = params_.initialTau_;
    return true;
}
inline int DynamicMovementPrimitive::getNumTransformationSystems() const
{
    return params_.numTransformationSystems_;
}

}

#endif /* DYNAMIC_MOVEMENT_PRIMITIVE_H_ */
