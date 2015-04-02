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

#include <iCub/piSquare/dmpMotionGeneration/dmpParameters.h>

namespace dmp
{

Parameters::Parameters(yarp::os::ResourceFinder* rf) :
		isLearned_(false), 
		isSetup_(false),
		isStartSet_(false), 
		numTransformationSystems_(0),
		tau_(0),
		initialTau_(0),
		deltaT_(0),
		initialDeltaT_(0),
		teachingDuration_(-1),
		canSysCutoff_(0),
		alphaX_(-1),
		kGain_(-1),
		dGain_(-1),
		numSamples_(0),
		rf_(rf)
{
}

Parameters::~Parameters()
{ 
}

bool Parameters::initialize()
{
	/**
    * think about whether teaching_duration should be read from file or "always" set
    * according to the length of the trajectory of which the dmp is learned
	*/
    teachingDuration_ = 0.0;
	
	numSamples_ = 0;

	//getting parameters from .ini file
	std::string parameterName = "can_sys_cutoff";

	if (!rf_->check(parameterName.c_str()))
    {
        printf("ERROR: Parameter /%s not found!\n", parameterName.c_str());
        return false;
    }

	canSysCutoff_ = rf_->find(parameterName.c_str()).asDouble();
    if (canSysCutoff_ <= 0)
    {
        printf("ERROR: Canonical system cutoff frequency is invalid (%.1f sec).\n", canSysCutoff_);
        return false;
    }

	parameterName = "dmp_gain";

	if (!rf_->check(parameterName.c_str()))
    {
        printf("ERROR: Parameter /%s not found!\n", parameterName.c_str());
        return false;
    }

	//the scaling between these two gains represents the fact that an overdumped system response is needed
	kGain_ = rf_->find(parameterName.c_str()).asDouble();
	dGain_ = kGain_ / 4.0;

    return true;
}

}
