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

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>
#include <string>

#define EIGEN2_SUPPORT
#include <Eigen/Eigen>

#include <yarp/os/ResourceFinder.h>

#include <iCub/piSquare/dmpMotionGeneration/dmpParameters.h>
#include <iCub/piSquare/dmpMotionGeneration/constants.h>

namespace dmp
{

static const int MAX_TRAJECTORY_LENGTH = 10000;
static const int MAX_TRAJECTORY_DIMENSION = 180;

static const int ABSOLUTE_MAX_TRAJECTORY_LENGTH = 20 * MAX_TRAJECTORY_LENGTH;
static const int ABSOLUTE_MAX_TRAJECTORY_DIMENSION = 20 * MAX_TRAJECTORY_DIMENSION;

/**
* \ingroup piSquare
*
* Class containing a trajectory, generally in terms of (position, velocity, acceleration) elements,
* and providing methods to deal with it.
*/

class Trajectory
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/** 
	* class constructor:
	* After instantiating a trajectory object, one need to initialize it with
	* the appropriate samplingFrquency, then one can add(trajectory_points).
	* @param rf is a resource finder object for parameters management
	*/
	Trajectory(yarp::os::ResourceFinder* rf);

	/** 
	* class destructor
	*/
    ~Trajectory();

	/** 
	* initializes the trajectory with the variable names provided as argument.
	* The trajectory dimension depends on the argument containsVelAcc. If the argument is set to false (default)
	* then the variable names will be treated as position only variables and velocity (_d) and acceleration (_dd) names
	* will be added. It also sets the sampling frequency.
	* @param variableNamesKeyWord is the trajectory identifier
	* @param trajectoryDimension is the number of dimensions for the current trajectory
	* @param samplingFrequency is the sampling frequency used for discretization
	* @param maxTrajectoryLength is the upper bound for trajectory length
	* @param maxTrajectoryDimension is the upper bound for trajectory dimension
	* @return true if operation is successful, false otherwise
	*/
    bool initialize(const std::string& variableNamesKeyWord, const int trajectoryDimension, const double samplingFrequency, 
					const int maxTrajectoryLength = MAX_TRAJECTORY_LENGTH, const int maxTrajectoryDimension = MAX_TRAJECTORY_DIMENSION);

	/** 
	* initializes the object with variable names and units read from .ini file.
	* If the trajectory is already initialized, the trajectory will be re-initialized with the given arguments.
	* The vector holding the transformation systems names is just used to obtain the trajectory dimension (not
	* its length!) which is then multiplied by three (i.e. position, velocity and acceleration).
	* @param variableNames is a vector containing the transformation system names
	* @param samplingFrequency is the sampling frequency used for discretization
	* @param containsVelAcc is a flag indicating if the trajectory also involves velocity and acceleration behaviours
	* @param maxTrajectoryLength is the upper bound for trajectory length
	* @param maxTrajectoryDimension is the upper bound for trajectory dimension
	* @return true if the object is initialized, false otherwise
	*/
    bool initialize(const std::vector<std::string>& variableNames, const double samplingFrequency, bool containsVelAcc = false,
                    const int maxTrajectoryLength = MAX_TRAJECTORY_LENGTH, const int maxTrajectoryDimension = MAX_TRAJECTORY_DIMENSION);

	/**
	* realizes the actual initialization, by initializing the trajectory and setting its dimension 
	* and sampling frequency accordingly.
	* @param trajectoryDimension is equal to the number of dimensions multiplied by three
	* @param samplingFrequency is the sampling frequency used for discretization
	* @param maxTrajectoryLength is the upper bound for trajectory length
	* @param maxTrajectoryDimension is the upper bound for trajectory dimension
	* @return true if operation is successful, false otherwise
	*/
    bool initialize(const int trajectoryDimension, const double samplingFrequency, const int maxTrajectoryLength = MAX_TRAJECTORY_LENGTH,
                    const int maxTrajectoryDimension = MAX_TRAJECTORY_DIMENSION);

	/** 
	* checks if the trajectory is initialized or not
	* @return true if the object has been initialized, false otherwise
	*/
    bool isInitialized() const;

	/** 
	* adds the trajectoryPoint (of size trajectoryDimension) to the trajectory. The procedure checks whether 
	* sampling frequency is specified correctly and also updates the trajectoryDuration
	* @param trajectoryPoint is the (position, velocity, acceleration) triple of values to be added to the trajectory
	* @return true if operation is successful, false otherwise
	*/
    bool add(const Eigen::VectorXd& trajectoryPoint);

	/** 
	* updates a single value of an already existing trajectory
	* @param lengthIndex is the time instant affected by the change
	* @param dimensionIndex is the dimension affected by the change
	* @param value is the new trajectory value
	* @return true if operation is successful, false otherwise
	*/
    bool update(const int lengthIndex, const int dimensionIndex, const double value);

	/** 
	* stores the trajectory as a .txt file
	* @param fileName is the file path
	* @return true if operation is successful, false otherwise
	*/
	bool writeTrajectoryToFile(std::string fileName);

	/** 
	* clears the trajectory
	*/
    void clear();

	/** 
	* gets the number of trajectory dimensions
	* @return the number of trajectory dimensions
	*/
    int getDimension() const;

	/** 
	* gets the trajectory length
	* @return the trajectory length
	*/
    int getLength() const;

	/** 
	* gets the max trajectory dimensions allowed
	* @return the max trajectory dimensions allowed
	*/
    int getMaxDimension() const;

	/** 
	* gets the max trajectory length allowed
	* @return the max trajectory length allowed
	*/
    int getMaxLength() const;

	/** 
	* sets the sampling frequency for discretization
	* @param sampleFrequency is the input sampling frequency to be set
	* @return true if operation is successful, false otherwise
	*/
    bool setSamplingFrequency(double sampleFrequency);

	/** 
	* gets the sampling frequency for discretization
	* @param sampleFrequency is the output sampling frequency to be retrieved
	* @return the sampling frequency
	*/
    double getSamplingFrequency() const;

	/** 
	* gets the initial positions vector
	* @param trajectoryStart is the output initial position vector to be retrieved
	* @return true if operation is successful, false otherwise
	*/
    bool getStartPosition(Eigen::VectorXd& trajectoryStart) const;

	/** 
	* gets the final positions vector
	* @param trajectoryEnd is the output final positions vector to be retrieved
	* @return true if operation is successful, false otherwise
	*/
    bool getEndPosition(Eigen::VectorXd& trajectoryEnd) const;

	/** 
	* gets a particular element of the trajectory data matrix
	* @param lengthIndex is the row index
	* @param trajectoryDimension is the column index
	* @return the value of the trajectory matrix element to be retrieved
	*/
    double getTrajectoryValue(int lengthIndex, int trajectoryDimension) const;

	/** 
	* gets a particular trajectory point (position, velocity, acceleration) for all the dimensions
	* @param trajectoryIndex identifies the element of the trajectory to be retrieved
	* @param trajectoryPoint is the output trajectory value
	* @return true if operation is successful, false otherwise
	*/
    bool getTrajectoryPoint(const int trajectoryIndex, Eigen::VectorXd& trajectoryPoint);

	/** 
	* gets the position value for a given time index and trajectory dimension
	* @param trajectoryIndex is the time index (row index)
	* @param trajectoryTrace is the dimension index (column index)
	* @return the trajectory position value to be retrieved
	*/
    double getTrajectoryPosition(int trajectoryIndex, int trajectoryTrace) const;

	/** 
	* gets the velocity value for a given time index and trajectory dimension
	* @param trajectoryIndex is the time index (row index)
	* @param trajectoryTrace is the dimension index (column index)
	* @return the trajectory velocity value to be retrieved
	*/
    double getTrajectoryVelocity(int trajectoryIndex, int trajectoryTrace) const;

	/** 
	* gets the acceleration value for a given time index and trajectory dimension
	* @param trajectoryIndex is the time index (row index)
	* @param trajectoryTrace is the dimension index (column index)
	* @return the trajectory acceleration value to be retrieved
	*/
    double getTrajectoryAcceleration(int trajectoryIndex, int trajectoryTrace) const;

	/** 
	* computes the mean squares error with respect to another trajectory
	* @param otherTrajectory is the trajectory to be compared
	* @param mseVector is the output vector representing the mse
	* @return true if operation is successful, false otherwise
	*/
    bool computeMSE(const Trajectory& otherTrajectory, Eigen::VectorXd& mseVector) const;

	/** 
	* computes the first and second derivatives for the trajectory
	*/
	void computeDerivatives();

	/** 
	* get the trajectory data as a [trajectoryLength] x [trajectoryDimensions] matrix
	* @param trajectoryMatrix is the output matrix containing the trajectory data
	* @return true if operation is successful, false otherwise
	*/
    bool getTrajectoryData(Eigen::MatrixXd& trajectoryData);

private:

	///flag indicating if the trajectory has been initialized or not
    bool initialized_;

	///number of trajectory time steps
    int trajectoryLength_;

	///number of trajectory dimensions
    int trajectoryDimension_;

	///max trajectory length allowed 
    int maxTrajectoryLength_;

	///max trajectory dimension allowed
    int maxTrajectoryDimension_;

	///sampling frequency for discretization
    double samplingFrequency_;

	/**
	* mainly for debugging purpose (since it is actually redundant) to check for consistency
	* between trajectoryLength, samplingFrequency and trajectoryDuration
	*/
    double trajectoryDuration_;

    //This matrix actually contains the data of the trajectory.
	Eigen::MatrixXd trajectoryData_;

	///resource finder object for parameters management
	yarp::os::ResourceFinder* rf_;

	/** 
	* indicates if the input dimension index has a meaningful value with respect
	* to the current trajectory matrix
	* @param dimensionIndex is the input index to be checked
	* @return true if the index has a meaningful value, false otherwise
	*/
    bool isWithinDimensionBoundaries(const int dimensionIndex) const;

	/** 
	* indicates if the input length index has a meaningful value with respect
	* to the current trajectory matrix
	* @param lengthIndex is the input index to be checked
	* @return true if the index has a meaningful value, false otherwise
	*/
    bool isWithinLengthBoundaries(const int lengthIndex) const;

};

// inline functions follow

inline bool Trajectory::isInitialized() const
{
    return initialized_;
}

inline int Trajectory::getDimension() const
{
    return trajectoryDimension_;
}
inline int Trajectory::getLength() const
{
    return trajectoryLength_;
}

inline bool Trajectory::setSamplingFrequency(double samplingFrequency)
{
    if (samplingFrequency <= 0.0)
    {
        printf("ERROR: Sampling frequency is %.1f and therefore invalid.\n",samplingFrequency);
        return false;
    }
    samplingFrequency_ = samplingFrequency;
    trajectoryDuration_ = static_cast<double> (trajectoryLength_) / samplingFrequency_;
    return true;
}
inline double Trajectory::getSamplingFrequency() const
{
    return samplingFrequency_;
}

inline bool Trajectory::getStartPosition(Eigen::VectorXd &trajectoryStart) const
{
    if (trajectoryLength_ == 0)
    {
        printf("ERROR: Trajectory is empty (trajectory length is %i\n)",trajectoryLength_);
        return false;
    }
    if (trajectoryDimension_ == 0)
    {
        printf("ERROR: Trajectory is empty (trajectory dimension is %i\n)",trajectoryDimension_);
        return false;
    }

    if (trajectoryStart.size() == trajectoryDimension_)
    {
        trajectoryStart = trajectoryData_.row(trajectoryLength_ - 1);
    }
    else if (trajectoryStart.size() == trajectoryDimension_ / POS_VEL_ACC)
    {
        for (int i = 0; i < trajectoryDimension_ / POS_VEL_ACC; i++)
        {
            trajectoryStart(i) = trajectoryData_(0, i * POS_VEL_ACC);
        }
    }
    else
    {
        return false;
    }

    return true;
}

inline bool Trajectory::getEndPosition(Eigen::VectorXd &trajectoryEnd) const
{
    if (trajectoryLength_ == 0)
    {
        printf("ERROR: Trajectory is empty (trajectory length is %i\n)",trajectoryLength_);
        return false;
    }
    if (trajectoryDimension_ == 0)
    {
        printf("ERROR: Trajectory is empty (trajectory dimension is %i\n)",trajectoryDimension_);
        return false;
    }

    if (trajectoryEnd.size() == trajectoryDimension_)
    {
        trajectoryEnd = trajectoryData_.row(trajectoryLength_ - 1);
    }
    else if (trajectoryEnd.size() == trajectoryDimension_ / POS_VEL_ACC)
    {
        for (int i = 0; i < trajectoryDimension_ / POS_VEL_ACC; i++)
        {
            trajectoryEnd(i) = trajectoryData_(trajectoryLength_ - 1, i * POS_VEL_ACC);
        }
    }
    else
    {
        return false;
    }
    return true;
}

inline double Trajectory::getTrajectoryPosition(int trajectoryIndex, int trajectoryTrace) const
{
    if ((trajectoryTrace >= 0) && (trajectoryTrace < trajectoryDimension_ / POS_VEL_ACC))
    {
        if ((trajectoryIndex >= 0) && (trajectoryIndex < trajectoryLength_))
        {
            return trajectoryData_(trajectoryIndex, (trajectoryTrace * POS_VEL_ACC) + _POS_ - 1);
        }
        else
        {
            printf("ERROR: Trajectory index (%i) not valid (trajectory length is %i), returning 0.0\n",trajectoryIndex, trajectoryLength_);
            return 0.0;
        }
    }
    else
    {
        printf("ERROR: Trajectory trace (%i) not valid (trajectory dimension is %i), returning 0.0\n",trajectoryTrace, trajectoryDimension_);
        return 0.0;
    }
    return 0.0;
}
inline double Trajectory::getTrajectoryVelocity(int trajectoryIndex, int trajectoryTrace) const
{
    if ((trajectoryTrace >= 0) && (trajectoryTrace < trajectoryDimension_ / POS_VEL_ACC))
    {
        if ((trajectoryIndex >= 0) && (trajectoryIndex < trajectoryLength_))
        {
            return trajectoryData_(trajectoryIndex, (trajectoryTrace * POS_VEL_ACC) + _VEL_ - 1);
        }
        else
        {
            printf("ERROR: Trajectory index (%i) not valid (trajectory length is %i), returning 0.0\n",trajectoryIndex, trajectoryLength_);
            return 0.0;
        }
    }
    else
    {
        printf("ERROR: Trajectory trace (%i) not valid (trajectory dimension is %i), returning 0.0\n",trajectoryTrace, trajectoryDimension_);
        return 0.0;
    }
    return 0.0;
}
inline double Trajectory::getTrajectoryAcceleration(int trajectoryIndex, int trajectoryTrace) const
{
    if ((trajectoryTrace >= 0) && (trajectoryTrace < trajectoryDimension_ / POS_VEL_ACC))
    {
        if ((trajectoryIndex >= 0) && (trajectoryIndex < trajectoryLength_))
        {
            return trajectoryData_(trajectoryIndex, (trajectoryTrace * POS_VEL_ACC) + _ACC_ - 1);
        }
        else
        {
            printf("ERROR: Trajectory index (%i) not valid (trajectory length is %i), returning 0.0\n",trajectoryIndex, trajectoryLength_);
            return 0.0;
        }
    }
    else
    {
        printf("ERROR: Trajectory trace (%i) not valid (trajectory dimension is %i), returning 0.0\n",trajectoryTrace, trajectoryDimension_);
        return 0.0;
    }
    return 0.0;
}

inline double Trajectory::getTrajectoryValue(int lengthIndex, int dimensionIndex) const
{
    if (isWithinLengthBoundaries(lengthIndex))
    {
        if (isWithinDimensionBoundaries(dimensionIndex))
        {
            return trajectoryData_(lengthIndex, dimensionIndex);
        }
        else
        {
            printf("ERROR: Trajectory dimension %i is out of bound [0..%i], returning 0.0\n",dimensionIndex, trajectoryDimension_);
            return 0.0;
        }
    }
    else
    {
        printf("ERROR: Trajectory index %i is out of bound [0..%i], returning 0.0\n",lengthIndex, trajectoryLength_);
        return 0.0;
    }
    return 0.0;
}

inline bool Trajectory::getTrajectoryData(Eigen::MatrixXd& trajectoryData)
{
	if (trajectoryLength_ == 0)
    {
        printf("ERROR: Trajectory is empty (trajectory length is %i\n)", trajectoryLength_);
        return false;
    }

    if (trajectoryDimension_ == 0)
    {
        printf("ERROR: Trajectory is empty (trajectory dimension is %i\n)", trajectoryDimension_);
        return false;
    }

	trajectoryData.resize(trajectoryLength_, trajectoryDimension_);
	trajectoryData = trajectoryData_.block(0, 0, trajectoryLength_, trajectoryDimension_);

	return true;
}

inline bool Trajectory::isWithinDimensionBoundaries(const int dimensionIndex) const
{
    return ((dimensionIndex >= 0) && (dimensionIndex < trajectoryDimension_));
}
inline bool Trajectory::isWithinLengthBoundaries(const int lengthIndex) const
{
    return ((lengthIndex >= 0) && (lengthIndex < trajectoryLength_));
}

}

#endif /* TRAJECTORY_H_ */
