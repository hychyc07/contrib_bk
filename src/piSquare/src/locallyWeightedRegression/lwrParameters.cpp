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

#include <sstream>

#include <yarp/os/ResourceFinder.h>

#include <iCub/piSquare/locallyWeightedRegression/lwrParameters.h>

namespace lwr
{

Parameters::Parameters(yarp::os::ResourceFinder* rf) : 
	initialized_(false),
	rf_(rf)
{
}

Parameters::~Parameters()
{
}

bool Parameters::initialize()
{
	//getting parameters from .ini file

	std::string parameterName = "rfs_width_boundary";

	if (!rf_->check(parameterName.c_str()))
    {
        printf("ERROR: Parameter /%s not found!\n", parameterName.c_str());
        return false;
    }

	rfsWidthBoundary_ = rf_->find(parameterName.c_str()).asDouble();
    if ((rfsWidthBoundary_ < 0 || rfsWidthBoundary_ > 1.0))
    {
        printf("ERROR: Wrong value for width_boundary parameter (%f) ...should be between 0 and 1.\n", rfsWidthBoundary_);
        initialized_ = false;
        return initialized_;
    }

	parameterName = "num_rfs";

	if (!rf_->check(parameterName.c_str()))
    {
        printf("ERROR: Parameter /%s not found!\n", parameterName.c_str());
        return false;
    }

	numRfs_ = rf_->find(parameterName.c_str()).asInt();
    if (numRfs_ <= 0)
    {
        printf("ERROR: Wrong value for num_rfs parameter (%f).\n", numRfs_);
        initialized_ = false;
        return initialized_;
    }

	parameterName = "can_sys_cutoff";

	if (!rf_->check(parameterName.c_str()))
    {
        printf("ERROR: Parameter /%s not found!\n", parameterName.c_str());
        return false;
    }

	canSysCutoff_ = rf_->find(parameterName.c_str()).asDouble();

    return (initialized_ = true);
}

}

