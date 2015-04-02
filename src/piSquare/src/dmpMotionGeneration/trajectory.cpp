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

#include <math.h>
#include <stdio.h>
#include <sstream>
#include <errno.h>
#include <assert.h>

#include <iCub/piSquare/dmpMotionGeneration/trajectory.h>
#include <iCub/piSquare/dmpMotionGeneration/dmpParameters.h>

USING_PART_OF_NAMESPACE_EIGEN

namespace dmp
{
static const int MAX_TRAJECTORY_SAMPLING_FREQUENCY = 1100;

Trajectory::Trajectory(yarp::os::ResourceFinder* rf) :
    initialized_(false),
	trajectoryLength_(0),
	trajectoryDimension_(0),
	maxTrajectoryLength_(0),
	maxTrajectoryDimension_(0),
	samplingFrequency_(-1.0),
	trajectoryDuration_(0.0),
	rf_(rf)
{
}

Trajectory::~Trajectory()
{
}

bool Trajectory::initialize(const std::vector<std::string>& variableNames, const double samplingFrequency, bool containsVelAcc,
                            const int maxTrajectoryLength, const int maxTrajectoryDimension)
{
	/**
	* another initialization function is called passing both the matrix "true" dimension and the sampling
	* frequency as parameters.
	*/

    if(initialized_) printf("Trajectory is already initialized. Invalidating trajectory initialization and re-initializing it with given parameters.\n");
    initialized_ = false;

    int trajectoryDimension = static_cast<int> (variableNames.size());
    if (!containsVelAcc)
    {
        trajectoryDimension *= POS_VEL_ACC;
    }

    if (!initialize(trajectoryDimension, samplingFrequency, maxTrajectoryLength, maxTrajectoryDimension))
    {
        printf("ERROR: Could not initialize trajectory.\n");
        initialized_ = false;
        return initialized_;
    }

    initialized_ = true;
    return initialized_;
}

bool Trajectory::initialize(const std::string& variableNamesKeyWord, const int trajectoryDimension, const double samplingFrequency,
                            const int maxTrajectoryLength, const int maxTrajectoryDimension)
{
    initialized_ = false;

    bool initializationSuccessfull = initialize(trajectoryDimension, samplingFrequency, maxTrajectoryLength, maxTrajectoryDimension);

    return initializationSuccessfull;
}

bool Trajectory::initialize(const int trajectoryDimension, const double samplingFrequency, const int maxTrajectoryLength,
                            const int maxTrajectoryDimension)
{
	/**
	* The trajectory duration (in terms of time) is computed as the ratio between the length of
	* the matrix (i.e. the second dimension of the matrix, the first one is the trajectory
	* dimension) and the sampling frequency, and initially it is equal to zero just because the
	* trajectory length is equal to zero at the beginning.
	* The call to the clear function seems to be redundant
	*/

    if(initialized_) printf("Trajectory is already initialized. Invalidating trajectory initialization and re-initializing it with given parameters.\n");
    initialized_ = false;

    //check whether trajectory_dimension is valid, and set if it is valid.
    if ((trajectoryDimension <= 0) || (trajectoryDimension > maxTrajectoryDimension))
    {
		printf("ERROR: Trajectory dimension %i is not valid. Should be within [0..%i].\n", trajectoryDimension, maxTrajectoryDimension);
        initialized_ = false;
        return initialized_;
    }
    trajectoryDimension_ = trajectoryDimension;
    if((trajectoryDimension_ % POS_VEL_ACC) != 0) printf("Trajectory dimension (%i) is not a multiple of 3 (for pos, vel, and acc), this may cause a problem\n", trajectoryDimension_);

    //check whether sampling frequency is valid, and set if it is valid.
    if ((samplingFrequency <= 0.0) || (samplingFrequency > MAX_TRAJECTORY_SAMPLING_FREQUENCY))
    {
		printf("ERROR: Invalid sampling frequency %.1f. It must be within (0..%i)\n", samplingFrequency, MAX_TRAJECTORY_SAMPLING_FREQUENCY);
        initialized_ = false;
        return initialized_;
    }
    samplingFrequency_ = samplingFrequency;
    trajectoryDuration_ = static_cast<double> (trajectoryLength_) / samplingFrequency_;

    maxTrajectoryLength_ = maxTrajectoryLength;
    maxTrajectoryDimension_ = maxTrajectoryDimension;
    trajectoryData_ = MatrixXd::Zero(maxTrajectoryLength_, maxTrajectoryDimension_);

    clear();
	
	return (initialized_ = true);
}

bool Trajectory::add(const VectorXd& trajectoryPoint)
{
    if (!initialized_ || (samplingFrequency_ <= 0.0))
    {
        printf("ERROR: Sampling frequency (%f) is not set yet. Not adding trajectory point.\n", samplingFrequency_);
        return false;
    }

    if (trajectoryPoint.size() != trajectoryDimension_)
    {
        printf("ERROR: Trajectory dimension %i is not the same as the dimension of the trajectory point %i.\n", trajectoryDimension_, trajectoryPoint.size());
        return false;
    }

    if (trajectoryLength_ >= maxTrajectoryLength_)
    {
        printf("ERROR: Maximum trajectory length (%i) reached ! TODO: remove me...\n", maxTrajectoryLength_);
        return false;
    }
    else
    {
        //VectorXd comes in as colum vector... so we have to transpose it
        trajectoryData_.block(trajectoryLength_, 0, 1, trajectoryDimension_) = trajectoryPoint.transpose();
        trajectoryLength_++;

        //update the duration of the trajectory. It has been checked that sampling_frequency is non zero
        trajectoryDuration_ = static_cast<double> (trajectoryLength_) / samplingFrequency_;
    }

    return true;
}

bool Trajectory::update(const int lengthIndex, const int dimensionIndex, const double value)
{
    if (!initialized_ || (samplingFrequency_ < 0.0))
    {
        printf("ERROR: Sampling frequency (%f) is not set yet. Not updating trajectory point.\n", samplingFrequency_);
        return false;
    }

    if (!isWithinDimensionBoundaries(dimensionIndex))
    {
        printf("ERROR: Trajectory dimension index %i exceeds trajectory dimension %i. Trajectory has size (%i x %i).\n", dimensionIndex, trajectoryDimension_, trajectoryData_.rows(), trajectoryData_.cols());
        return false;
    }
    if (!isWithinLengthBoundaries(lengthIndex))
    {
        printf("ERROR: Trajectory length index %i exceeds trajectory length %i. Trajectory has size (%i x %i).\n", lengthIndex, trajectoryLength_, trajectoryData_.rows(), trajectoryData_.cols());
        return false;
    }

    trajectoryData_(lengthIndex, dimensionIndex) = value;
    return true;
}

bool Trajectory::getTrajectoryPoint(const int lengthIndex, VectorXd& trajectoryPoint)
{
    if ((!isWithinLengthBoundaries(lengthIndex)) || (trajectoryPoint.size() != trajectoryDimension_))
    {
        return false;
    }
    trajectoryPoint = trajectoryData_.block(lengthIndex, 0, 1, trajectoryDimension_).transpose();
    return true;
}

bool Trajectory::writeTrajectoryToFile(std::string fileName)
{
	FILE *fp;
    if ((fp = fopen(fileName.c_str(), "w")) == NULL)
    {
        printf("ERROR: Cannot open file %s\n", fileName.c_str());
        return false;
    }

    for (int i = 0; i < trajectoryLength_; i++)
    {
        for (int j = 0; j < trajectoryDimension_; j++)
        {
			fprintf(fp, "%f\t", trajectoryData_(i,j));
        }

		fprintf(fp, "\n");
    }

	fclose(fp);

	return true;
}

void Trajectory::clear()
{
    trajectoryData_.setZero(maxTrajectoryLength_, maxTrajectoryDimension_);
    trajectoryLength_ = 0;

    //also update the duration of the trajectory;
    trajectoryDuration_ = 0.0;
}

void Trajectory::computeDerivatives()
{

    if ((trajectoryDimension_ % POS_VEL_ACC) != 0)
    {
       printf("ERROR: Cannot compute the derivatives for trajectories which dimension is not a multiple of %i.", POS_VEL_ACC);
        return;
    }
    int trajectoryTrace = trajectoryDimension_ / POS_VEL_ACC;

    int addPosPoints = 4;
    int posLength = addPosPoints + trajectoryLength_ + addPosPoints;
    int velLength = -2 + posLength - 2;
    int accLength = -2 + velLength - 2;

    MatrixXd tmpTrajectoryPos = MatrixXd::Zero(posLength, trajectoryTrace);
    for (int i = 0; i < posLength; i++)
    {
        for (int j = 0; j < trajectoryTrace; j++)
        {
            if (i < addPosPoints)
            {
                tmpTrajectoryPos(i, j) = trajectoryData_(0, j * POS_VEL_ACC);
            }
            else if (i < addPosPoints + trajectoryLength_)
            {
                tmpTrajectoryPos(i, j) = trajectoryData_(i - addPosPoints, j * POS_VEL_ACC);
            }
            else
            {
                tmpTrajectoryPos(i, j) = tmpTrajectoryPos(i - 1, j);
            }
        }
    }

    MatrixXd tmpTrajectoryVel = MatrixXd::Zero(velLength, trajectoryTrace);
    for (int i = 0; i < velLength; i++)
    {
        for (int j = 0; j < trajectoryTrace; j++)
        {
            tmpTrajectoryVel(i, j) = (tmpTrajectoryPos(i, j) - (static_cast<double> (8.0) * tmpTrajectoryPos(i + 1, j)) + (static_cast<double> (8.0)
                    * tmpTrajectoryPos(i + 3, j)) - tmpTrajectoryPos(i + 4, j)) / static_cast<double> (12.0);
            tmpTrajectoryVel(i, j) *= samplingFrequency_;
        }
    }

    MatrixXd tmpTrajectoryAcc = MatrixXd::Zero(accLength, trajectoryTrace);

    for (int i = 0; i < accLength; i++)
    {
        for (int j = 0; j < trajectoryTrace; j++)
        {
            tmpTrajectoryAcc(i, j) = (tmpTrajectoryVel(i, j) - (static_cast<double> (8.0) * tmpTrajectoryVel(i + 1, j)) + (static_cast<double> (8.0)
                    * tmpTrajectoryVel(i + 3, j)) - tmpTrajectoryVel(i + 4, j)) / static_cast<double> (12.0);
            tmpTrajectoryAcc(i, j) *= samplingFrequency_;
        }
    }

    clear();

    VectorXd trajectoryPoint = VectorXd::Zero(trajectoryDimension_);
    for (int i = 0; i < accLength; i++)
    {
        for (int j = 0; j < trajectoryTrace; j++)
        {
            trajectoryPoint(j * POS_VEL_ACC + 0) = tmpTrajectoryPos(i + 4, j);
            trajectoryPoint(j * POS_VEL_ACC + 1) = tmpTrajectoryVel(i + 2, j);
            trajectoryPoint(j * POS_VEL_ACC + 2) = tmpTrajectoryAcc(i + 0, j);
        }
        add(trajectoryPoint);
    }

}

bool Trajectory::computeMSE(const Trajectory& otherTrajectory, VectorXd& mseVector) const
{

    if (trajectoryDimension_ != otherTrajectory.getDimension())
    {
        printf("ERROR: Trajectories do not have same dimension (%i vs. %i).\n", trajectoryDimension_, otherTrajectory.getDimension());
        return false;
    }

    int numTrajectoryPointsUsedForComputingMse = trajectoryLength_;
    int otherTrajectoryLength = otherTrajectory.getLength();
    if (trajectoryLength_ != otherTrajectoryLength)
    {
        printf("Trajectory length are different: %i vs %i.\n", trajectoryLength_, otherTrajectoryLength);
        if (fabs((double)trajectoryLength_ - otherTrajectoryLength) > 10)
        {
            printf("ERROR: Trajectory lengths differ more than 10 points (%i vs %i).\n", trajectoryLength_, otherTrajectoryLength);
            return false;
        }

        if (trajectoryLength_ > otherTrajectoryLength)
        {
            numTrajectoryPointsUsedForComputingMse = otherTrajectoryLength;
        }
        else
        {
            numTrajectoryPointsUsedForComputingMse = trajectoryLength_;
        }
    }

    if ((trajectoryDimension_ % POS_VEL_ACC) != 0)
    {
        printf("ERROR: Compute the MSE for trajectories which do not have a multiple of 3 dimension is not implemented yet. The trajectory contains %i dimensions.\n", trajectoryDimension_);
        return false;
    }

    int numPositionTraces = trajectoryDimension_ / POS_VEL_ACC;

    if (mseVector.size() != numPositionTraces)
    {
        printf("ERROR: MSE vector has wrong size (%i), it should have size %i.\n", mseVector.size(), numPositionTraces);
        return false;
    }

    for (int i = 0; i < numTrajectoryPointsUsedForComputingMse; i++)
    {
        for (int j = 0; j < numPositionTraces; j++)
        {
            mseVector(j) += pow(trajectoryData_(i, j * POS_VEL_ACC) - otherTrajectory.getTrajectoryPosition(i, j), 2);
        }
    }

    for (int j = 0; j < numPositionTraces; j++)
    {
        mseVector(j) /= numTrajectoryPointsUsedForComputingMse;
    }
    return true;
}

int Trajectory::getMaxDimension() const
{
    assert(initialized_);
    return maxTrajectoryDimension_;
}
int Trajectory::getMaxLength() const
{
    assert(initialized_);
    return maxTrajectoryLength_;
}

}