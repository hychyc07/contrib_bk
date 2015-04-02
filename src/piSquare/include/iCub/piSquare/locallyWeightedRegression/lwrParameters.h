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

#ifndef LWR_PARAMETERS_H_
#define LWR_PARAMETERS_H_

#include <string>
#include <vector>

#include <yarp/os/ResourceFinder.h>

namespace lwr
{

/**
* \ingroup piSquare
*
* Manages parameters needed to perform a Locally Weighted Regression.
*/

class Parameters
{

public:

	/**
    * class constructor
    */
	Parameters(yarp::os::ResourceFinder* rf);
	
	/**
    * class destructor
    */
	virtual ~Parameters();

	/**
    * initializes the object by retrieving the parameters from .ini file
	* @return true if operation is successful, false otherwise
    */
	bool initialize();

	/**
    * indicates if the object is initialized or not
	* @return true if operation is successful, false otherwise
    */
	bool isInitialized() const;

	/**
    * gets the number of basis functions
	* @return the number of basis functions
    */
	int getNumRFS() const;

	/**
    * gets the boundary for basis functions width
	* @return the boundary for basis functions width
    */
    double getRFSWidthBoundary() const;

	/**
    * gets the canonical system cutoff
	* @return the canonical system cutoff
    */
    double getCanSysCutOff() const;

private:

	///flag indicating if the object is initialized or not
	bool initialized_;

	///Number of receptive fields
	int numRfs_;

	///This value specifies at which value to neighboring RFs will intersect
	double rfsWidthBoundary_;

	///affects the location of basis functions centers
	double canSysCutoff_;

	///resource finder object for parameters management
    yarp::os::ResourceFinder* rf_;

};

//inline functions follow
inline bool Parameters::isInitialized() const
{
	return initialized_;
}

inline int Parameters::getNumRFS() const
{
	return numRfs_;
}

inline double Parameters::getCanSysCutOff() const
{
    return canSysCutoff_;
}

inline double Parameters::getRFSWidthBoundary() const
{
    return rfsWidthBoundary_;
}

}

#endif /* LWR_PARAMETERS_H_ */
