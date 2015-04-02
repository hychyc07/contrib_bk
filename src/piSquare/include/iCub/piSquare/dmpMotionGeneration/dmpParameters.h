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

#ifndef DMP_PARAMETERS_H_
#define DMP_PARAMETERS_H_

#include <string>

#include <yarp/os/ResourceFinder.h>

#include <iCub/piSquare/dmpMotionGeneration/constants.h>

namespace dmp
{

/**
* \ingroup piSquare
*
* Manages DMP parameters. They are shared between all the existing DMPs.
*/

class Parameters
{

public:

    friend class DynamicMovementPrimitive;

	/**
    * Class constructor
    * @param rf is a resource finder object for parameters management
    */
	Parameters(yarp::os::ResourceFinder* rf);

	/**
    * class destructor
    */
    ~Parameters();

	/**
	* Initializes the dmp by reading the parameters .ini file. In particular values are fetched for
	* canSysCutoff, dmpGain.
	* @return true if initialization is successful, false otherwise.
	*/
    bool initialize();

private:

    ///Indicates whether the DMP has been learned, i.e. the parameters of the DMP has been computed/set
    bool isLearned_;

    ///Indicates whether the DMP is setup, i.e. the start, goal, and duration has been set
    bool isSetup_;

	/**
    * Indicates whether the start of the DMP has been set. This flag is used to update the start position of
    * successive DMPs, when the start position cannot be determined beforehand.
    */
    bool isStartSet_;

	///the number of transformation systems, one for each prpblem dimension
    int numTransformationSystems_;

    ///Timing parameters used during learning and integrating the system
    double tau_;
    double initialTau_;
    double deltaT_;
    double initialDeltaT_;

    ///Time durations
    double teachingDuration_;

	///affects the location of basis functions centers
    double canSysCutoff_;

    ///Time constants
    double alphaX_;

	///Scaling factors
    double kGain_;
    double dGain_;

    int numSamples_;

	///resource finder object for parameters management
	yarp::os::ResourceFinder* rf_;

};

}

#endif /* DMP_PARAMETERS_H_ */
