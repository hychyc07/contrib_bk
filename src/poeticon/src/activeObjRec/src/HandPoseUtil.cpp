#include <iostream>

#include <opencv2/core/core.hpp>

#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>

#include "iCub/HandPoseUtil.h"
#include "iCub/Util.h"

using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub;

Vector HandPoseUtil::rotateVector(const Vector &v, const Vector &axis, double angle_rad) 
{
    Vector r(4);
    r[0]=axis[0]; r[1]=axis[1]; r[2]=axis[2]; r[3]=angle_rad;
    Matrix mR=ctrl::axis2dcm(r);

    Vector vrot(4);
    vrot[0] = v[0];
    vrot[1] = v[1];
    vrot[2] = v[2];
    vrot[3] = 0;

    Vector res = mR * vrot; 
    res.pop_back();

    return res;
}


Vector HandPoseUtil::comp2ndNormal(const Vector &x, const Vector &n, double angle_deg) 
{
    // find arbitrary vector on plane orthogonal to hand normal
    Vector x1(3);

    if (n[2] == 0) 
    {
        x1[0] = x[0];
        x1[1] = x[1];
        x1[2] = 1;
    }
    else
    {
        x1[0] = 1;
        x1[1] = 1;
        x1[2] = -((x1[0]-x[0])*n[0] + (x1[1]-x[1])*n[1] - x[2]*n[2])/ n[2];
    }

    Vector curr_nx = Util::normalize(x1 - x);
    //std::cout << "curr_nx" << curr_nx.toString() << std::endl;


    // rotate vector so it points to the left of the robot as far as possible
    double min_d = 10000;
    Vector nxd(3);
    double rot_step = M_PI / 16;
    for (double rot = 0; rot < 2*M_PI; rot += rot_step)
    {
        Vector c = rotateVector(curr_nx, n, rot);
        Vector y(3);
        y[0] = 0;
        y[1] = 1;
        y[2] = 0;
        double d = yarp::math::dot(c, y);
        //std::cout << "c " << rot << ": " << c.toString() << " d: " << d << std::endl;
        if (d < min_d)
        {
            min_d = d;
            nxd[0] = c[0];
            nxd[1] = c[1];
            nxd[2] = c[2];
        }

    }
    //std::cout << "opt nxd:" << nxd.toString() << std::endl;

    bool flip = false;
    if (angle_deg > 180)
    {
        angle_deg -= 180;
        flip = true;
    }

    min_d = 10000;
    Vector nx(3);
    for (double rot = 0; rot < M_PI; rot += rot_step)
    {
        Vector c1 = rotateVector(nxd, n, rot);
        double a = acos(yarp::math::dot(c1, nxd) / (norm(c1)*norm(nxd)))*180/M_PI;
        //std::cout << "c " << rot << ": " << c1.toString() << " a: " << a << "angle_deg: " << angle_deg << std::endl;
        double d = abs(a-angle_deg);
        if (d < min_d)
        {
            min_d = d;
            nx[0] = c1[0];
            nx[1] = c1[1];
            nx[2] = c1[2];
        }
    }

    if (flip)
    {
        nx[0] *= -1;
        nx[1] *= -1;
        nx[2] *= -1;
    }
    return nx;
}

Vector HandPoseUtil::addOffset(const Vector &handX, const Vector &handO, 
            double gazeOffsetX, double gazeOffsetY, double gazeOffsetZ, bool isLeftHand)
{
    Vector result(3);

    if (isLeftHand)
    {
        gazeOffsetZ *= -1;
        gazeOffsetY *= -1;
    }

    Vector nx = ctrl::axis2dcm(handO).getCol(0);
    Vector ny = ctrl::axis2dcm(handO).getCol(1);
    Vector nz = ctrl::axis2dcm(handO).getCol(2);

    result[0] = handX[0] + nz[0] * gazeOffsetZ;
    result[1] = handX[1] + nz[1] * gazeOffsetZ;
    result[2] = handX[2] + nz[2] * gazeOffsetZ;

    result[0] += nx[0] * gazeOffsetX;
    result[1] += nx[1] * gazeOffsetX;
    result[2] += nx[2] * gazeOffsetX;

    result[0] += -ny[0] * gazeOffsetY;
    result[1] += -ny[1] * gazeOffsetY;
    result[2] += -ny[2] * gazeOffsetY;
    
    return result;
}


void HandPoseUtil::getGazeAngles(const Vector &eyePos, const Vector &handPos, 
        const Vector &handOrient, double &e, double &r, bool lefthand) 
{
    Vector nz, nx, ny;
    Matrix m = ctrl::axis2dcm(handOrient);
    m = m.submatrix(0,2,0,2); 
    nx = m.getCol(0);
    ny = m.getCol(1);
    nz = m.getCol(2); 

    // account for different reference frame of left hand
    if (lefthand) 
    {
        ny[0] *= -1;
        ny[1] *= -1;
        ny[2] *= -1;
        nz[0] *= -1;
        nz[1] *= -1;
        nz[2] *= -1;
    }

    Vector gaze = Util::normalize(eyePos - handPos);

    Vector axisPalm = nz;

    e = acos(yarp::math::dot(gaze, axisPalm))*Util::RAD2DEG;

    // rot axes
    Vector rotAxis = Util::normalize(yarp::math::cross(gaze,axisPalm));
    double rotAngle = 90 - e;

    Vector axisIndex = Util::normalize(rotateVector(nx, rotAxis, rotAngle*Util::DEG2RAD));
    Vector axisThumbOp = Util::normalize(rotateVector(ny, rotAxis, rotAngle*Util::DEG2RAD));

    //std::cout << "\nrotAngle : " << rotAngle << std::endl;
    //std::cout << "rotAxis  : " << rotAxis.toString() << std::endl;
    //std::cout << "gaze     : " << gaze.toString() << std::endl;
    //std::cout << "axisPalm : " << axisPalm.toString() << std::endl;
    //std::cout << "nz       : " << nz.toString() << std::endl;
    //std::cout << "nx       : " << nx.toString() << std::endl;
    //std::cout << "ny       : " << ny.toString() << std::endl;
    //std::cout << "axisIndex: " << axisIndex.toString() << std::endl << std::endl;
    //std::cout << "axisThumbOp: " << axisIndex.toString() << std::endl << std::endl;

    r = acos(yarp::math::dot(gaze, axisIndex))*Util::RAD2DEG;


    double rThumb = acos(dot(gaze, axisThumbOp))*Util::RAD2DEG;
    //r = acos(dot(oz, ny))*Util::RAD2DEG;
    //double dir = dot(ox, ny);
    if (rThumb > 90)
    {   
       r = 360-r;
    }

    if (lefthand)
    {
        r = 360-r;
    }

    //std::cout << "e/r: " << e << " " << r << std::endl;
}

