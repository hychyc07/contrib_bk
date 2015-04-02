/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Andrea DelPrete
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

#include "iCub/skinForceControl/utilRobotics.h"
#include <string>
#include <map>
#include <stdarg.h>     // va_list va_arg va_end va_start
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/dev/PolyDriver.h>

using namespace iCub::skinForceControl;
using namespace iCub::skinDynLib;
using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::math;
using namespace std;


Status iCub::skinForceControl::setFirmwarePidGains(RobotInterfaces *r, BodyPart bp, const int* kp, const int* ki, 
        const int* kd, const int *shift, const int* max, const int* fc) throw(){
    Pid* pids = new Pid[16];
    if(r==NULL || r->itrq[bp]==NULL || r->idbg[bp]==NULL) 
        return Status("Error while setting firmware gains, interfaces not available");
    if(!r->itrq[bp]->getTorquePids(pids))
        return Status("Error while getting torque pids");
    Status s;
    int d = 1;
    if(bp == LEFT_ARM)
        d = -1;
    for(int i=0;i<5;i++){
        pids[i].kp = d*kp[i];
        pids[i].kd = d*kd[i];
        pids[i].ki = d*ki[i];
        pids[i].scale = shift[i];
        pids[i].max_output = max[i];
        if(! r->idbg[bp]->setDebugParameter(i, 5,  filt(fc[i])))
            s.addErrMsg("Error while setting fc of joint "+toString(i));
    }
    if(! r->itrq[bp]->setTorquePids(pids))
        s.addErrMsg("Error while setting torque pids ");
    return s;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
yarp::sig::Matrix iCub::skinForceControl::getH0rightArm(){
    yarp::sig::Matrix H(4,4);
    H.zero();
    double ct = cos(165*CTRL_DEG2RAD);
    double st = sin(165*CTRL_DEG2RAD);
    H(0,0) = -ct;         // -ct  0   -st
    H(0,2) = -st;         //  0  -1    0
    H(0,3) = +0.0256;     // -st  0   +ct
    H(1,1) = -1.0;
    H(1,3) = -0.0500;
    H(2,0) = -st;
    H(2,2) = +ct;
    H(2,3) = +0.1070;
    H(3,3) = +1.0;
    return H;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
yarp::sig::Matrix iCub::skinForceControl::getH0lefttArm(){
    yarp::sig::Matrix H(4,4);
    H.zero();
    double ct = cos(165*CTRL_DEG2RAD);
    double st = sin(165*CTRL_DEG2RAD);
    H(0,0) = +ct;        // +ct  0   -st
    H(0,2) = -st;        //  0   1    0
    H(0,3) = -0.0256;    // +st  0   +ct
    H(1,1) = +1.0;
    H(1,3) = +0.0500;
    H(2,0) = +st;
    H(2,2) = +ct;
    H(2,3) = -0.1073;
    H(3,3) = +1.0;
    return H;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
yarp::sig::Matrix iCub::skinForceControl::getH0torso(){
    yarp::sig::Matrix H(4,4);
    H.zero();
    H(0,2)= +1.0;  //  0  0  1
    H(1,0)= -1.0;  // -1  0  0
    H(2,1)= -1.0;  //  0 -1  0
    H(3,3)= +1.0;
    return H;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iCub::skinForceControl::computeEinv(const Vector &axisAngleOrientation){
    Vector l = axisAngle2quaternion(axisAngleOrientation);
    Matrix Einv(6,7);
    Einv.eye();

    Einv(3,3) = -2*l(1);
    Einv(3,4) = +2*l(0);
    Einv(3,5) = -2*l(3);
    Einv(3,6) = +2*l(2);

    Einv(4,3) = -2*l(2);
    Einv(4,4) = +2*l(3);
    Einv(4,5) = +2*l(0);
    Einv(4,6) = -2*l(1);

    Einv(5,3) = -2*l(3);
    Einv(5,4) = -2*l(2);
    Einv(5,5) = +2*l(1);
    Einv(5,6) = +2*l(0);
    return Einv;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iCub::skinForceControl::axisAngle2quaternion(const Vector &axisAngle){
    Vector quaternion(4);
    quaternion(0) = cos(axisAngle(3)/2);
    double sinTheta = sin(axisAngle(3)/2);
    quaternion(1) = axisAngle(0) * sinTheta;
    quaternion(2) = axisAngle(1) * sinTheta;
    quaternion(3) = axisAngle(2) * sinTheta;
    return quaternion;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iCub::skinForceControl::projectSixDimVector(const Vector &v, const Matrix &R){
    //return cat(R*v.subVector(0,2), R*v.subVector(3,5));
    //Matrix R6(6,6); R6.zero();
    //R6.setSubmatrix(R,0,0);
    //R6.setSubmatrix(R,3,3);
    //return R6*v;
    Vector res(6);
    for(int i=0;i<3;i++){
        res[i] = R(i,0)*v(0)+R(i,1)*v(1)+R(i,2)*v(2);
        res[3+i] = R(i,0)*v(3)+R(i,1)*v(4)+R(i,2)*v(5);
    }
    return res;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iCub::skinForceControl::projectPose(const Vector &x, const Matrix &H){
    Vector res(7);
    res.setSubvector(0, H*(cat(x.subVector(0,2),1)));
    res.setSubvector(3, H.submatrix(0,2,0,2)*x.subVector(3,5));
    res(6) = x[6];
    return res;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iCub::skinForceControl::project6dimMatrix(const Matrix &M, const Matrix &R){
    Matrix R6(6,6); R6.zero();
    R6.setSubmatrix(R,0,0);
    R6.setSubmatrix(R,3,3);
    return R6*M;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iCub::skinForceControl::wrenchProjectionMatrix(const Matrix &H)
{
    Matrix res(6,6);
    Matrix R = H.submatrix(0,2,0,2);
    res.setSubmatrix(R, 0, 0);
    res.setSubmatrix(R, 3, 3);
    res.setSubmatrix(crossProductMatrix(H.subcol(0,3,3))*R, 3, 0);
    return res;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix iCub::skinForceControl::contactPointJacobian(const Matrix &J, const Vector &p){
    Matrix H = eye(4,4);
    H.setSubcol(p,0,3);
    return adjointInv(H)*J;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCub::skinForceControl::contactPointJacobian(const Vector &p, Matrix &J){
    if(J.rows()!=6)
        return false;
    Matrix H = eye(4,4);
    H.setSubcol(p,0,3);
    J = adjointInv(H)*J;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int iCub::skinForceControl::pinvBounded(const Matrix &A, const Vector &y, const Vector &lb, 
    const Vector &ub, Matrix &Apinv, Vector &x, double tol)
{
    int m = A.rows(), n = A.cols(), k = m<n?m:n;
	Matrix U(m,k), V(n,k);
	Vector Sdiag(k);
	SVD(A, U, Sdiag, V);

	Matrix Spinv = zeros(k,k);
	for(int c=0; c<k; c++)
		if(Sdiag(c) > 0.0)
			Spinv(c,c) = 1.0/Sdiag(c);
    
    // project y over the columns of U
    Vector x_proj = y*U;
    int j = k-1;
    x = V*Spinv*x_proj;
    //printf("PINV BOUND\nx least square: %s\nlb: %s\n, ub: %s\n", x.toString(3).c_str(), lb.toString(3).c_str(), ub.toString(3).c_str());
    //printf("A:\n%s\n", A.toString(3).c_str());
    //printf("y: %s\n", y.toString(3).c_str());
    //printf("sing val of A: %s\n", Sdiag.toString(3).c_str());
    while(j>=0 && isOverBound(x, lb, ub, tol))
    {
        printf("x is over bound: %s\n", x.toString(3).c_str());
        printf("Gonna remove component number %d\n", j);
        Spinv(j,j) = 0.0;   // remove a component
        j--;
	    x = V*Spinv*x_proj;
    }
    
    Apinv = V*Spinv*U.transposed();
    return j+1;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCub::skinForceControl::isOverBound(const yarp::sig::Vector &x, const yarp::sig::Vector &lb, const yarp::sig::Vector &ub, double tol){
    for(size_t i=0; i<x.size(); i++)
        if(x(i)<lb(i)-tol || x(i)>ub(i)+tol){
            if(x(i)<lb(i)-tol)
                printf("x(%d) = %f < %f\n", i, x(i), lb(i));
            else
                printf("x(%d) = %f > %f\n", i, x(i), ub(i));
            return true;
        }
    return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool iCub::skinForceControl::computeAccBounds(const Vector &qMin, const Vector &qMax, const Vector &dqMax, const Vector &q, 
    const Vector &dq, const Vector &act, double period, Vector &ddqL, Vector &ddqU)
{
    ddqL = (qMin - q - period*dq)/(period*period);  // in deg
    ddqU = (qMax - q - period*dq)/(period*period);  // in deg
    double dqU, dqL, k;
    for(unsigned int i=0;i<q.length(); i++){
        k = -1.0/(act[i]*act[i]);
        dqU =      dqMax[i]*(1.0 - exp(k*pow(q[i]-qMax[i], 2)));  // in deg
        dqL = -1.0*dqMax[i]*(1.0 - exp(k*pow(q[i]-qMin[i], 2)));  // in deg
        
        ddqU[i] = min(ddqU[i], (dqU-dq[i])/period);    // in deg
        ddqL[i] = max(ddqL[i], (dqL-dq[i])/period);    // in deg
    }
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iCub::skinForceControl::projectOnPlane(const Vector &x, const Vector &p, const Vector &n)
{
    double nNorm = norm(n);
    if(nNorm==0.0)
        return x;
    Vector nn = n/nNorm;
    return x + dot(p-x,nn)*nn;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector iCub::skinForceControl::rotateOntoPlane(const Vector &x, const Vector &n)
{
    double nNorm = norm(n);
    if(nNorm==0.0)
        return x;
    Vector nn = n/nNorm;
    double xNorm = norm(x);
    return xNorm*versor(x-dot(x,nn)*nn);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCub::skinForceControl::rototranslate(const Matrix &H, Vector &p)
{
    double temp0 = H(0,0)*p(0) + H(0,1)*p(1) + H(0,2)*p(2) + H(0,3);
    double temp1 = H(1,0)*p(0) + H(1,1)*p(1) + H(1,2)*p(2) + H(1,3);
    p(2) = H(2,0)*p(0) + H(2,1)*p(1) + H(2,2)*p(2) + H(2,3);
    p(0)=temp0; p(1)=temp1;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCub::skinForceControl::rototranslate(const Matrix &H, const Vector &p, Vector &res)
{
    if(res.size()!=3) 
        res.resize(3);
    res(0) = H(0,0)*p(0) + H(0,1)*p(1) + H(0,2)*p(2) + H(0,3);
    res(1) = H(1,0)*p(0) + H(1,1)*p(1) + H(1,2)*p(2) + H(1,3);
    res(2) = H(2,0)*p(0) + H(2,1)*p(1) + H(2,2)*p(2) + H(2,3);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCub::skinForceControl::rotate(const Matrix &H, const Vector &p, Vector &res)
{
    if(res.size()!=3) 
        res.resize(3);
    res(0) = H(0,0)*p(0) + H(0,1)*p(1) + H(0,2)*p(2);
    res(1) = H(1,0)*p(0) + H(1,1)*p(1) + H(1,2)*p(2);
    res(2) = H(2,0)*p(0) + H(2,1)*p(1) + H(2,2)*p(2);
}