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

#include <iCub/piSquare/dmpMotionGeneration/mathHelper.h>
#include <iCub/piSquare/dmpMotionGeneration/constants.h>

using namespace Eigen;

namespace dmp
{

bool MathHelper::calculateMinJerkNextStep(const yarp::sig::Vector &start, const yarp::sig::Vector &goal, const double duration,
										  const double deltaT, yarp::sig::Vector &next)
{
    if ((duration <= 0) || (deltaT <= 0) || (deltaT > duration))
    {
        if(duration <= 0) printf("ERROR: Duration is less or equal 0.\n");
        if(deltaT <= 0) printf("ERROR: Delta t is less or equal 0.\n");
        if(deltaT > duration) printf("ERROR: Delta t is greater than duration.\n");
        return false;
    }
    if ((start.size() != POS_VEL_ACC) || (goal.size() != POS_VEL_ACC) || (next.size() != POS_VEL_ACC))
    {
        if(start.size() != POS_VEL_ACC) printf("ERROR: Start vector has wrong size (%i), should be (%i).\n", start.size(), POS_VEL_ACC);
        if(goal.size() != POS_VEL_ACC) printf("ERROR: Goal vector has wrong size (%i), should be (%i).\n", goal.size(), POS_VEL_ACC);
        if(next.size() != POS_VEL_ACC) printf("ERROR: Next vector has wrong size (%i), should be (%i).\n", next.size(), POS_VEL_ACC);
        return false;
    }

    const double t1 = deltaT;
    const double t2 = t1 * deltaT;
    const double t3 = t2 * deltaT;
    const double t4 = t3 * deltaT;
    const double t5 = t4 * deltaT;

    const double tau1 = duration;
    const double tau2 = tau1 * duration;
    const double tau3 = tau2 * duration;
    const double tau4 = tau3 * duration;
    const double tau5 = tau4 * duration;

    //calculate the constants
    const double dist = goal(_POS_-1) - start(_POS_-1);
    const double c1 = 6. * dist / tau5 + (goal(_ACC_-1) - start(_ACC_-1)) / (2. * tau3) - 3. * (start(_VEL_-1) + goal(_VEL_-1)) / tau4;
    const double c2 = -15. * dist / tau4 + (3. * start(_ACC_-1) - 2. * goal(_ACC_-1)) / (2. * tau2) + (8. * start(_VEL_-1) + 7. * goal(_VEL_-1)) / tau3;
    const double c3 = 10. * dist / tau3 + (goal(_ACC_-1) - 3. * start(_ACC_-1)) / (2. * duration) - (6. * start(_VEL_-1) + 4. * goal(_VEL_-1)) / tau2;
    const double c4 = start(_ACC_-1) / 2.;
    const double c5 = start(_VEL_-1);
    const double c6 = start(_POS_-1);

    next(_POS_ - 1) = c1 * t5 + c2 * t4 + c3 * t3 + c4 * t2 + c5 * t1 + c6;
    next(_VEL_ - 1) = 5. * c1 * t4 + 4 * c2 * t3 + 3 * c3 * t2 + 2 * c4 * t1 + c5;
    next(_ACC_ - 1) = 20. * c1 * t3 + 12. * c2 * t2 + 6. * c3 * t1 + 2. * c4;

    return true;
}

bool MathHelper::generateMinJerkTrajectory(const yarp::sig::Vector &start, const yarp::sig::Vector &goal, const double duration,
										   const double deltaT, Trajectory &trajectory)
{
    int trajectoryDimension = trajectory.getDimension();

    if ((trajectoryDimension % POS_VEL_ACC) != 0)
    {
        printf("ERROR: Trajectory dimension (%i) must be a multiple of 3 to contain position, velocity, and acceleration information for each trace.\n",
                  trajectoryDimension);
        return false;
    }
    trajectoryDimension /= POS_VEL_ACC;

    if ((trajectoryDimension != start.size()) || (trajectoryDimension != goal.size()))
    {
        printf("ERROR: Trajectory dimension (%i), does not match start (%i) and goal (%i).\n", trajectoryDimension, start.size(), goal.size());
        return false;
    }
    trajectory.clear();

    int numSteps = static_cast<int> (duration / deltaT);

	yarp::sig::Vector tmpCurrent = yarp::math::zeros(POS_VEL_ACC);
    yarp::sig::Vector tmpGoal = yarp::math::zeros(POS_VEL_ACC);
    yarp::sig::Vector tmpNext = yarp::math::zeros(POS_VEL_ACC);
    yarp::sig::Vector next = yarp::math::zeros(trajectoryDimension * POS_VEL_ACC);
	Eigen::VectorXd eigenNext;

    //add first trajectory point
    for (int j = 0; j < trajectoryDimension; j++)
    {
        next(j * POS_VEL_ACC + 0) = start(j);
        next(j * POS_VEL_ACC + 1) = 0;
        next(j * POS_VEL_ACC + 2) = 0;
    }

	yarpToEigenVector(next, eigenNext);
    if (!trajectory.add(eigenNext))
    {
        printf("ERROR: Could not add first trajectory point.\n");
        return false;
    }

    for (int i = 1; i < numSteps; i++)
    {
        for (int j = 0; j < trajectoryDimension; j++)
        {
            if (i == 1)
            {
                tmpCurrent(0) = start(j);
            }
            else
            {
                //update the current state
                for (int k = 0; k < POS_VEL_ACC; k++)
                {
                    tmpCurrent(k) = next(j * POS_VEL_ACC + k);
                }
            }
            tmpGoal(0) = goal(j);

            if (!MathHelper::calculateMinJerkNextStep(tmpCurrent, tmpGoal, duration - ((i - 1) * deltaT), deltaT, tmpNext))
            {
                printf("ERROR: Could not compute next minimum jerk trajectory point.\n");
                return false;
            }

            for (int k = 0; k < POS_VEL_ACC; k++)
            {
                next(j * POS_VEL_ACC + k) = tmpNext(k);
            }
        }

		yarpToEigenVector(next, eigenNext);
        if (!trajectory.add(eigenNext))
        {
            printf("ERROR: Could not add next trajectory point.\n");
            return false;
        }
    }

	if(!trajectory.writeTrajectoryToFile("data/minJerkTrajectory.txt")) return false;

    return true;
}

bool MathHelper::yarpToEigenVector(const yarp::sig::Vector& yarpVector, Eigen::VectorXd& eigenVector)
{
	if(yarpVector.size() == 0)
	{
		printf ("ERROR: input vector is empty.\n");
        return false;
	}

	//resize and fill eigen vector with yarp vector elements
	eigenVector.resize(yarpVector.size());
	for(unsigned int i = 0; i < yarpVector.size(); i++)
	{
		eigenVector(i) = yarpVector(i);
	}

	return true;
}

bool MathHelper::eigenToYarpVector(const Eigen::VectorXd& eigenVector, yarp::sig::Vector& yarpVector)
{
	if(eigenVector.size() == 0)
	{
		printf ("ERROR: input vector is empty.\n");
        return false;
	}

	//resize and fill yarp vector with eigen vector elements
	yarpVector.resize(eigenVector.size());
	for(int i = 0; i < eigenVector.size(); i++)
	{
		yarpVector(i) = eigenVector(i);
	}

	return true;
}

}
