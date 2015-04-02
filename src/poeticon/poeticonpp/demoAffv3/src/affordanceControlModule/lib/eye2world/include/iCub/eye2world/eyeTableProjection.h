/*******************************************************************************
 * Copyright (C) 2009 Christian Wressnegger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *******************************************************************************/

#ifndef __ICUB_VISLAB__VISLAB_PROJECTION_H_
#define __ICUB_VISLAB__VISLAB_PROJECTION_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>

#include <stdexcept>

#include <iCub/eye2world/simpleHomography.h>

namespace vislab {
namespace math {

class EyeTableProjection: public SimpleHomography {
private:
	iCub::iKin::iCubEye eye;
	const ::yarp::os::ConstString camera;

	static CameraCalibration getCameraCalibration(::yarp::os::Property& p);

	double safetyRadius;

public:
	/**
	 * The constructor
	 * @param eye The specifier of the the eye to use ("right" | "left").
	 * @param calibration The calibration of the eye according to the configuration
	 * 				files produced by the calibration modules for the eye cameras.
     * @param tabletop The relative offset of the tabletop to the reference frame of the robot. 
     * @param headV2 true iff head v.2 is employed 
	 */
	EyeTableProjection(const ::yarp::os::ConstString& eye, ::yarp::os::Property& calibration,
			const ::yarp::sig::Vector* tabletop = NULL, const bool headV2 = false);
	/**
	 * The destructor.
	 */
	virtual ~EyeTableProjection();

	/**
	 * Sets the base transformation for the camera to the robot's base coordinates based on the
	 * current state of the torso and the head of the robot.
	 * @param torso3d The current state of the robot's torso retrieved by e.g. /icub/torso/state:o
	 * @param head6dThe current state of the robot's head retrieved by e.g. /icub/head/state:o
	 */
	void setBaseTransformation(const ::yarp::sig::Vector& torso3d, const ::yarp::sig::Vector& head6d);

	/**
	 * Sest a radius for the area around the robot's base coordinates where points shouldn't be projected
	 * to. If a point is internally projected to that area, it is correct such that the output matches the
	 * above criteria. The constructor sets this value to 0.2 (20 centimeters) by default.
	 * @param r The safety radius [m]. 0.01 = 1 millimeter.
	 */
	void setSafetyRadius(const double r);

	/**
	 * Projects the given 2D point to the table top but limits the output the robot's possible field of view.
	 * @param in The 2D input coordinates to be projected.
	 * @param out The 3D output of {@var in}.
	 */
	void project(const ::yarp::sig::Vector& in, ::yarp::sig::Vector& out);
};

}
}

#endif /* __ICUB_VISLAB__VISLAB_PROJECTION_H_ */
