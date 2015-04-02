// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2011 RBCS ITT
 *
 * Author:  Juan G Victores
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#ifndef CARTESIAN_FORCE
#define CARTESIAN_FORCE

#include <stdio.h>
#include <math.h>
#include <string>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/os/Port.h>
#include <yarp/dev/PolyDriver.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace iCub::iDyn;
using namespace std;

class cartesianForce {

    yarp::dev::PolyDriver robotDevice;

    yarp::dev::IPositionControl *pos;
    yarp::dev::IControlMode *ictrl;
    yarp::dev::ITorqueControl *itrq;

    BufferedPort<Vector> *port_ext_contacts;

    int numberControlledJoints;  // degrees of freedom (torque) controlled
    int dof;
    int dof_T;
    int blockedIndex[7];

    Vector zero_vec;
    Matrix zero6x6;
    Matrix eye6x6;
    Matrix J_ab, JT_ab;
    Matrix J_r, JT_r;

    // HOMOGENEOUS TRANSFORMATION MATRIXES
    Matrix  H_tf_ab;            // torso final  TO  link arm base
    Matrix  H_r_tb;             // root         TO  torso base
    Matrix  H_tb_tf;            // torso base   TO  torso final link (depends on torso joint angles q_T)
    Matrix  H_r_ab;             // root         TO  arm base         (depends on the torso joint angles q_T)
    Matrix  R_r_ab;             // root         TO  arm base         (only rotation)
    Matrix  H_ab_ee;            // arm base     TO  end effector     (depends on the arm joint angles q)
    Matrix  R_ab_ee;            // arm base     TO  end effector     (only rotation)

    // end effector force = force exherted by the e.e. upon the environment
    Vector  f_r, f_ab, f_ee; // current end effector force (force and moment) WRT robot root, arm base frame, e.e. frame

    FILE* outFile;
    Matrix Kj;
    Vector encoders;
    Vector encoders_T;
    Vector q;
    Vector q_T;
    Vector dq;
    Vector G;
    Vector cmdTorques;

    AWLinEstimator *linEst;
    iCubArmNoTorsoDyn *armChain;
    iCubTorsoDyn *torsoChain;

public:

    bool send_cmd;
    bool verbose;

    Vector home_deg;
    yarp::dev::IEncoders *encs;
    yarp::dev::IEncoders *encs_torso;
    std::string controlled_part;
    std::string robotname;

    double Kp, Kd;
    Vector Fd;

    cartesianForce() {

        dof=7;

        //let's control the shoulder and the elbow
        numberControlledJoints = 5; // was 4, but I'm going to use all the motors i can, xD

        // constructor
        pos=0;
        encs=0;
        ictrl=0;
        itrq=0;

        send_cmd = false;

        Kp = 1;  // Proportional gain default (Fd-Fread)
        Kd = 0.01;  // Damping effect (proportional to estimated joint velocity)
        Fd.resize(6,0.0); // Desired force vector

        zero6x6.resize(6,6);
        zero6x6.zero();    
        eye6x6.resize(6,6);
        eye6x6.eye();    
    
        zero_vec.resize(6, 0.0);
        for (int i=0; i<7; i++)
            blockedIndex[i] = 0;

        outFile = fopen("KMatrix.txt", "w"); // Some logging file...

        verbose = true;
    }

    bool open();

    bool start();
    bool stop();
    bool home();

    void loop();

    bool interrupt();

    bool close();

    Vector evalVel(const Vector &x);

    Vector projectSixDimVector(const Vector &v, const Matrix &R);

    Matrix projectJacobian(const Matrix &J, const Matrix &R);

    Matrix computeLAMBDA();  // calculates LAMBDA, the equivalent to M in OpSpace
    Matrix computeLAMBDA2(); // Let's do it the Lilly way...
    Matrix computeLAMBDA3(); // Let's do it the Khatib way! (ref [Khatib00])
    Matrix getLambdaH(const int& jointNumber);  // get binary Matrix1x6 on joint types
    Matrix getS(const int& joint); 
    Matrix getPHI(const int& first,const int& second);  // calculates Rigid Body Transformation Matrix
    Matrix getRelR(const int& first,const int& second);  // Rot Matrix(3,3) from "first" to "second"
    Matrix getRelRcom(const int& joint);  // Rot Matrix(3,3) from "joint" to link com
    Vector getVector3(const int& first,const int& second);  // get Vector(3) from "first" to "second" of armChain
    Vector getVector3com(const int& joint);  // "l" Vector(3) from "joint" to link com
    Matrix makeCross(const Vector& in);  // creates skew-sym equivalent to cross product
};

#endif
