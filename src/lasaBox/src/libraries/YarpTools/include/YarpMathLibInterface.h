#ifndef YARPMATHLIBINTERFACE_H_
#define YARPMATHLIBINTERFACE_H_

/**
 *
\defgroup icub_YarpTools YarpTools
@ingroup icub_lasaBox_libraries

A library with some utility function to interface Yarp with MathLib, and other utilities...

\section intro_sec Description

This set of class are used for some of the software found in the LasaBox.

More information can be found by doxygen-izing the source code completely.



\section tested_os_sec Tested OS

Linux

\author Eric Sauser

Copyright (C) 2011 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at $ICUB_ROOT/contrib/src/lasaBox/src/libraries/YarpTools
**/


#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include "MathLib/MathLib.h"

//using namespace MathLib;

MathLib::Vector     YarpVectorToVector  (const yarp::sig::Vector &yvec);
MathLib::Vector&    YarpVectorToVector  (const yarp::sig::Vector &yvec, MathLib::Vector& result);
MathLib::Matrix     YarpMatrixToMatrix  (const yarp::sig::Matrix &ymat);
MathLib::Matrix&    YarpMatrixToMatrix  (const yarp::sig::Matrix &ymat, MathLib::Matrix& result);

yarp::sig::Vector   VectorToYarpVector  (const MathLib::Vector &vec);
yarp::sig::Vector&  VectorToYarpVector  (const MathLib::Vector &vec, yarp::sig::Vector& result);
yarp::sig::Matrix   MatrixToYarpMatrix  (const MathLib::Matrix &mat);
yarp::sig::Matrix&  MatrixToYarpMatrix  (const MathLib::Matrix &mat, yarp::sig::Matrix& result);

bool                LoadYarpMatrix      (const char* filename, yarp::sig::Matrix &result);
bool                SaveYarpMatrix      (const char* filename, yarp::sig::Matrix &mat);

MathLib::Vector&    YarpPose7ToPose6(yarp::sig::Vector &pose, MathLib::Vector &result);
void                YarpPose7ToPose6(yarp::sig::Vector &pose, MathLib::Vector3 &pos, MathLib::Vector3 &ori);
void                YarpPose6ToPose6(yarp::sig::Vector &pose, MathLib::Vector3 &pos, MathLib::Vector3 &ori);
void                Pose6ToYarpPose6(MathLib::Vector3 &pos, MathLib::Vector3 &ori, yarp::sig::Vector &pose);
void                AddPose6ToYarpPose6(MathLib::Vector3 &pos, MathLib::Vector3 &ori, yarp::sig::Vector &pose);
void                AddVel6ToYarpVel6(MathLib::Vector3 &pos, MathLib::Vector3 &ori, yarp::sig::Vector &pose);

MathLib::Vector &   AddPose6ToPose6(MathLib::Vector &pose, const MathLib::Vector &offset);
MathLib::Vector &   SubPose6ToPose6(MathLib::Vector &pose, const MathLib::Vector &offset);
MathLib::Vector &   GetDeltaPose6FromPose6(const MathLib::Vector &src, const MathLib::Vector &target, MathLib::Vector &delta);

void                YarpPose7ToYarpPose6(yarp::sig::Vector &pose, yarp::sig::Vector &result);

MathLib::Matrix3&   YarpMatrix4ToMatrix3  (const yarp::sig::Matrix &ymat, MathLib::Matrix3& result);
MathLib::Matrix4&   YarpMatrix4ToMatrix4  (const yarp::sig::Matrix &ymat, MathLib::Matrix4& result);

MathLib::Vector3&   YarpVector3ToVector3  (const yarp::sig::Vector &vec, MathLib::Vector3& result);
yarp::sig::Vector&  Vector3ToYarpVector3  (const MathLib::Vector3 &vec, yarp::sig::Vector &result);

#endif /*YARPMATHLIBINTERFACE_H_*/
