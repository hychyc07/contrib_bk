/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Andrea Del Prete
  * email: andrea.delprete@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

#ifndef SFC_UTIL_ROB_H
#define SFC_UTIL_ROB_H

#include <vector>
#include <map>
#include <sstream>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Semaphore.h>
#include <ace/Recursive_Thread_Mutex.h>
#include <ace/Thread.h>
#include <iCub/ctrl/math.h>
#include <iCub/iDyn/iDyn.h>
#include "iCub/skinDynLib/common.h"
#include "iCub/skinForceControl/robot_interfaces.h"
#include "iCub/skinForceControl/util.h"

namespace iCub
{

namespace skinForceControl
{

    Status setFirmwarePidGains(iCub::skinForceControl::RobotInterfaces *r, iCub::skinDynLib::BodyPart bp, const int* kp, const int* ki, 
        const int* kd, const int *shift, const int* max, const int* fc) throw();

    yarp::sig::Matrix getH0rightArm();

    yarp::sig::Matrix getH0lefttArm();

    yarp::sig::Matrix getH0torso();

    /**
     * Compute the matrix Einv for the axis angle orientation. The matrix is so 
     * defined: dx/dt = Einv*[v; omega]
     */
    yarp::sig::Matrix computeEinv(const yarp::sig::Vector &axisAngleOrientation);

    /**
     * Convert axis angle orientation representation to quaternion.
     */
    yarp::sig::Vector axisAngle2quaternion(const yarp::sig::Vector &axisAngle);

    /**
     * Project a six dimensional vector to a different reference frame, using the 
     * specified rotation matrix.
     * This method can be used to project wrenches, generalized velocities, generalized accelleration, 
     * but not generalized positions (i.e. it can be used for geometric vectors but not for geometric points).
     */
    yarp::sig::Vector projectSixDimVector(const yarp::sig::Vector &v, const yarp::sig::Matrix &R);

    /**
     * Project a generalized pose vector (position and orientation in axis-angle notation) to a 
     * different reference frame, using the specified rotation matrix.
     * This method cannot be used to project wrenches, generalized velocities, generalized accelleration
     * (i.e. it can be used for points but not for geometric vectors).
     */
    yarp::sig::Vector projectPose(const yarp::sig::Vector &v, const yarp::sig::Matrix &R);

    /**
     * Project a matrix composed by 6-dim column vectors to a different reference frame
     * using the specified rotation matrix R.
     */
    yarp::sig::Matrix project6dimMatrix(const yarp::sig::Matrix &M, const yarp::sig::Matrix &R);

    /**
     * Compute the 6x6 wrench projection matrix corresponding to the rototranslation H.
     * The resulting matrix is: [R  0; S(r)*R  R]
     */
    yarp::sig::Matrix wrenchProjectionMatrix(const yarp::sig::Matrix &H);

    /**
     * Compute the Jacobian of a specific point of the robot chain.
     * @param J 6xN jacobian of the link where the specified point p lies
     * @param p 3d coordinates of the point w.r.t. the link reference frame
     * @note J and p have to be expressed w.r.t. the same reference frame
     */
    yarp::sig::Matrix contactPointJacobian(const yarp::sig::Matrix &J, const yarp::sig::Vector &p);

    /**
     * Compute the Jacobian of a specific point of the robot chain.
     * @param p 3d coordinates of the point w.r.t. the link reference frame
     * @param J 6xN jacobian of the link where the specified point p lies
     * @return true if operation succeeded, false otherwise
     * @note J and p have to be expressed w.r.t. the same reference frame
     */
    bool contactPointJacobian(const yarp::sig::Vector &p, yarp::sig::Matrix &J);

    /**
     * Compute Apinv that is a truncated pseudo-inverse of A such that:
     * lb <= Apinv*y <= ub
     * while trying to minimize the square error:
     * || A*Apinv*y - y ||^2
     * @param A input matrix
     * @param y input vector
     * @param lb lower bounds of x
     * @param ub upper bounds of x
     * @param Apinv output truncated pseudo-inverse of A
     * @param x output vector x=Apinv*y
     * @param tol tollerance used to check the bounds
     * @return the number of components that have not been set to zero
     */
    unsigned int pinvBounded(const yarp::sig::Matrix &A, const yarp::sig::Vector &y, const yarp::sig::Vector &lb, 
        const yarp::sig::Vector &ub, yarp::sig::Matrix &Apinv, yarp::sig::Vector &x, double tol=0.0);

    /**
    * Check whether all the entries of a vector x are within the specified lower and upper bounds.
    * @return false if lb<=x<=ub, true otherwise
    */
    bool isOverBound(const yarp::sig::Vector &x, const yarp::sig::Vector &lb, const yarp::sig::Vector &ub, double tol=0.0);

    /**
    * Compute the joint acceleration bounds.
    * @param qMin joint position lower bounds in deg
    * @param qMax joint position upper bounds in deg
    * @param dqMax joint velocity upper bounds in deg/sec
    * @param q current joint positions in deg
    * @param dq current joint velocities in deg/sec
    * @param act activation thresholds in deg, when q is act degrees off the limit the max velocity is reduced by 30%
    * @param period controller time period in sec
    * @param ddqL output acceleration lower bounds in deg/sec^2
    * @param ddqU output acceleration upper bounds in deg/sec^2
    */
    bool computeAccBounds(const yarp::sig::Vector &qMin, const yarp::sig::Vector &qMax, const yarp::sig::Vector &dqMax, 
        const yarp::sig::Vector &q, const yarp::sig::Vector &dq, const yarp::sig::Vector &act, double period, 
        yarp::sig::Vector &ddqL, yarp::sig::Vector &ddqU);

    /**
    * Project a 3d point on a plane defined by a point and the normal direction.
    * @param x the point to project
    * @param p a point of the plane
    * @param n the plane normal direction
    * @return the projection of x on the plane defined by the point p and the direction n.
    */
    yarp::sig::Vector projectOnPlane(const yarp::sig::Vector &x, const yarp::sig::Vector &p, const yarp::sig::Vector &n);

    /**
    * Rotate a 3d vector onto a plane defined by the normal direction.
    * The norm of the vector after rotation is of course unchanged.
    * @param x the point to rotate
    * @param n the plane normal direction
    * @return the rotation of x on the plane defined by the direction n.
    */
    yarp::sig::Vector rotateOntoPlane(const yarp::sig::Vector &x, const yarp::sig::Vector &n);

    /**
    * Rototranslate the 3d point p using the 4-by-4 rototranslation matrix H.
    */
    void rototranslate(const yarp::sig::Matrix &H, yarp::sig::Vector &p);

    /**
    * Rototranslate the 3d point p using the 4-by-4 rototranslation matrix H.
    */
    void rototranslate(const yarp::sig::Matrix &H, const yarp::sig::Vector &p, yarp::sig::Vector &res);

    /**
    * Rotate the 3d point p using the 4-by-4 rototranslation matrix H.
    */
    void rotate(const yarp::sig::Matrix &H, const yarp::sig::Vector &p, yarp::sig::Vector &res);

}

} // end namespace

#endif

