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

#include <errno.h>
#include <sstream>
#include <assert.h>

#include <boost/foreach.hpp>

#include <iCub/piSquare/dmpMotionGeneration/transformationSystem.h>
#include <iCub/piSquare/dmpMotionGeneration/constants.h>

USING_PART_OF_NAMESPACE_EIGEN

namespace dmp
{

TransformationSystem::TransformationSystem() :
	initialized_(false),
	transId_(-1),
	z_(0), zd_(0),
	y_(0), yd_(0), ydd_(0),
	t_(0), td_(0), tdd_(0),
	y0_(0), initialY0_(0),
	goal_(0), initialGoal_(0),
	f_(0), ft_(0),
	mse_(0), 
	meanFt_(0),
	numMseDataPoints_(0)
{
}

TransformationSystem::~TransformationSystem()
{
}

bool TransformationSystem::initialize(int transId, lwr::Parameters lwrParams)
{
    if (initialized_)
    {
        printf("Transformation system already initialized. Reinitializing with new parameters.\n");
    }

    transId_ = transId;

    if (!lwrParams.isInitialized())
    {
        printf("ERROR: LWP parameters are not initialized.\n");
        initialized_ = false;
        return initialized_;
    }

    if (!lwrModel_.initialize(lwrParams.getNumRFS(), lwrParams.getRFSWidthBoundary(), true, lwrParams.getCanSysCutOff()))
    {
        printf("ERROR: Could not initialize LWR model.\n");
        initialized_ = false;
        return initialized_;
    }

    trajectoryTarget_.clear();

    initialized_ = true;
    return initialized_;
}

bool TransformationSystem::writeTrajectoryTargetToFile(std::string fileName)
{
	FILE *fp;
	int count = 0;
    if ((fp = fopen(fileName.c_str(), "w")) == NULL)
    {
        printf("ERROR: Cannot open file %s\n", fileName.c_str());
        return false;
    }

	std::vector<double>::iterator iter = trajectoryTarget_.begin();
	for(; iter != trajectoryTarget_.end(); ++iter)
	{
		fprintf(fp, "%f\n", *iter);
		count++;
	}

	return true;
}

}
