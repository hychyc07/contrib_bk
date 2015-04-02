// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "iCub/cartesianForce/cartesianForce.h"
#include "stdio.h"

using namespace std;

#define SEND_COMMANDS
#define N 5

const double THRESHOLD = 2.2;
static const double GRAV_ACC = 9.81;

bool cartesianForce::open() {
    H_tf_ab.resize(4,4); // Hleft or Hright
    H_tf_ab.zero();
    double ct = cos(165*CTRL_DEG2RAD);
    double st = sin(165*CTRL_DEG2RAD);
    if (controlled_part=="left_arm") {
        H_tf_ab(0,0) = ct;        // ct  0   st
        H_tf_ab(0,2) = st;        //  0   1    0
        H_tf_ab(0,3) = 0.003066;  // -st  0   ct
        H_tf_ab(1,1) = 1.0;
        H_tf_ab(1,3) = -0.0500;
        H_tf_ab(2,0) = -st;
        H_tf_ab(2,2) = ct;
        H_tf_ab(2,3) = -0.110261;
    } else {
        H_tf_ab(0,0) = -ct;         // -ct  0   -st
        H_tf_ab(0,2) = -st;         //  0  -1    0
        H_tf_ab(0,3) = 0.0033066;     // -st  0   +ct
        H_tf_ab(1,1) = -1.0;
        H_tf_ab(1,3) = -0.0500;
        H_tf_ab(2,0) = -st;
        H_tf_ab(2,2) = +ct;
        H_tf_ab(2,3) = -0.110261;
    }
    H_r_tb.resize(4,4);
    H_r_tb.zero();
    H_r_tb(0,1)= -1.0;  //  0 -1  0
    H_r_tb(1,2)= -1.0;  //  0  0 -1
    H_r_tb(2,0)=  1.0;  //  1  0  0
    H_r_tb(3,3)=  1.0;

    // Basic port naming...
    std::string remotePorts="/";
    remotePorts+=robotname;
    remotePorts+="/"+controlled_part;
    std::string localPorts="/cartesianForce/"+controlled_part;

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());   //local port names
    options.put("remote", remotePorts.c_str());    //where we connect to
    robotDevice.open(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    // Device interface view...
    bool ok;
    ok = robotDevice.view(pos);
    ok = ok && robotDevice.view(encs);
    ok = ok && robotDevice.view(encs_torso);
    ok = ok && robotDevice.view(ictrl);
    ok = ok && robotDevice.view(itrq);
    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return false;
    }

    int axes;
    pos->getAxes(&axes);
    encoders.resize(axes,0.0);
    q.resize(dof,0.0);
    dq.resize(dof,0.0);
    f_ee.resize(6, 0.0);
    f_ab.resize(6, 0.0);
    f_r.resize(6, 0.0);
    cmdTorques.resize(numberControlledJoints, 0.0);

    // Read the current position, save that info for homing later...
    home_deg.resize(axes);
    while (!encs->getEncoders(home_deg.data())) {
        fprintf(stderr, ".");
        Time::delay(0.1);
    }

    Vector w0(3), dw0(3), ddp0(3), Fend(3), Mend(3);
    w0=dw0=ddp0=Fend=Mend=0.0;

    // TORSO
    int axes_torso;
    encs_torso->getAxes(&axes_torso);
    dof_T = axes_torso;
    encoders_T.resize(axes_torso,0.0);
    q_T.resize(dof_T,0.0);

    torsoChain = new iCubTorsoDyn();
    torsoChain->setAng(q_T);
    torsoChain->prepareNewtonEuler();
    // the gravity has to be expressed w.r.t. the frame 0 of the torso chain (it is NOT the root)
    ddp0[0] = GRAV_ACC;
    torsoChain->initNewtonEuler(w0, dw0, ddp0, Fend, Mend);

    // Making port for recieving external force info...
    port_ext_contacts = new BufferedPort<Vector>;
    if (!port_ext_contacts->open("/local/ext_contacts:i"))
        printf("It was not possible to open the ext_contacts:i port");
    std::string wbContPortName = "/wholeBodyDynamics/" + controlled_part + "/endEffectorWrench:o";
    while (!Network::exists(wbContPortName.c_str())) {  // check if wholeBody is running
        fprintf(stderr,"Waiting for wholeBodyDynamics (port: %s)...\n", wbContPortName.c_str());
        Time::delay(1.0);
    }

    // Try to connect to this port to recieve from the wholeBodyDynamics output port...
    // ((you may comment it out when running with App manager))
    fprintf(stderr,"wholeBodyDynamics exists! Trying to connect...\n");
    if (Network::connect(wbContPortName.c_str(), port_ext_contacts->getName().c_str())) {
//        f_ee = *(port_ext_contacts->read(true)); // hangs with sim...
        printf("[success] done!\n");
    } else printf("[ERROR] Could not connect to wholeBodyDynamics!!\n");

    // -- ARM-related --
    ddp0 = 0.0;
    if (controlled_part=="right_arm") {
        armChain = new iCubArmNoTorsoDyn("right");
        ddp0[1] = -GRAV_ACC;
    } else {
        armChain = new iCubArmNoTorsoDyn("left");
        ddp0[1] = GRAV_ACC;
    }
    armChain->setAng(q);
    armChain->setDAng(dq);
    armChain->prepareNewtonEuler();
    armChain->initNewtonEuler(w0, dw0, ddp0, Fend, Mend);

    linEst = new iCub::ctrl::AWLinEstimator(16,1.0);

    return true;
}


bool cartesianForce::start() {
    //switching joints to torque control
#ifdef SEND_COMMANDS
    for (int i = 0; i < numberControlledJoints; i++)
        if (blockedIndex[i] ==1) // if its blocked we put it in position mode
            ictrl->setPositionMode(i);
        else  // and this is the default behaviour
            ictrl->setTorqueMode(i);
    fprintf(stderr,"(REAL) Set non-blocked joints to torque mode.\n");
#else
    fprintf(stderr,"(SIM) Set non-blocked joints to torque mode.\n");
#endif
    return true;
}


bool cartesianForce::stop() {
#ifdef SEND_COMMANDS
    for (int i = 0; i < numberControlledJoints; i++)
        ictrl->setPositionMode(i);

    fprintf(stderr,"(REAL) Set all joints to position mode.\n");
#else
    fprintf(stderr,"(SIM) Set all joints to position mode.\n");
#endif
    return true;
}


bool cartesianForce::home() {
#ifdef SEND_COMMANDS
    for (int i = 0; i < numberControlledJoints; i++)
        ictrl->setPositionMode(i);
    fprintf(stderr,"(REAL) Set all joints to position mode.\n");

    pos->positionMove(home_deg.data());
//	fprintf(stderr,"(REAL) Moved to: %s \n", home_deg.toString().c_str());
    fprintf(stderr,"(REAL) Moved HOME.\n");
#else
    fprintf(stderr,"(SIM) Set all joints to position mode.\n");
//	fprintf(stderr,"(SIM) Moved to: %s \n", home_deg.toString().c_str());
    fprintf(stderr,"(SIM) Moved HOME.\n");
#endif
    return true;
}


void cartesianForce::loop() {

    // *** ARM ***
    if (encs->getEncoders(encoders.data()) == true) {
        for (int i=0; i<7; i++)
            q(i) = encoders(i);
        dq = evalVel(q);
        armChain->setAng(CTRL_DEG2RAD * q);
        armChain->computeNewtonEuler();
    } else {
        printf("Get arm encoders failed!\n");
    }

    // *** TORSO ***
    if (encs_torso->getEncoders(encoders_T.data()) == true) {
        for (int i=0; i<dof_T; i++)
            q_T(i) = encoders_T(2-i);
        torsoChain->setAng(CTRL_DEG2RAD * q_T);
    } else {
        printf("Get torso encoders failed!\n");
    }

    // *** END EFFECTOR FORCE ***
    Vector* f_eeRead = port_ext_contacts->read(false);
    if (f_eeRead) {
        f_ee = *f_eeRead;                       // e.e. force in end effector frame
    } else {
        printf("Get external contacts failed!\n");
    }

    // *** HOMOGENEOUS TRANSFORMATION MATRIXES ***
    H_ab_ee = armChain->getH();           // rototranslation  from arm base    to end effector
    R_ab_ee = H_ab_ee.submatrix(0,2,0,2); // rotation         from arm base    to end effector

    // H_r_tb is in the constructor       // rototranslation  from root        to torso base
    H_tb_tf = torsoChain->getH();         // rototranslation  from torso base  to torso final link (neck base)
    // H_tf_ab is in the constructor      // rototranslation  from torso base  to arm base

    H_ab_ee = armChain->getH();           // rototranslation  from arm base    to end effector

    H_r_ab  = H_r_tb * H_tb_tf * H_tf_ab; // rototranslation  from root        to arm base
    printf("H_r_ab: %s\n",H_r_ab.toString().c_str());

    R_r_ab  = H_r_ab.submatrix(0,2,0,2);  // rotation         from arm base    to root

    J_ab = armChain->GeoJacobian();

    for (int i=0; i<7; i++)
        if (blockedIndex[i] ==1)
            J_ab.setCol(i,zero_vec);

    // JT_ab = J_ab.transposed();  // Not used
    J_r  = projectJacobian(J_ab, R_r_ab);  // EE Jacobian expressed in root frame
    JT_r = J_r.transposed();               // EE Jacobian transposed in root frame

    // Use the root frame; uncomment and use JT_ab here for testing in arm base.
    Matrix JTcut = JT_r.submatrix(0,numberControlledJoints-1,0,5);      // remove the non controlled joints

    f_ab  = projectSixDimVector(f_ee, R_ab_ee);  // e.e. force in arm base frame
    f_r  = projectSixDimVector(f_ab, R_r_ab);    // e.e. force in root frame

//    printf("Fread: %s\n",f_ee.toString().c_str());
//    printf("Y: Fd=%.6f | f_r: %.6f | error=%.6f\n",Fd[1],f_r[1],Fd[1]+f_r[1]);
    printf("Z: Fd=%.6f | f_r: %.6f | error=%.6f\n",Fd[2],f_r[2],Fd[2]+f_r[2]);
    fflush(stdout);

    G = armChain->getTorques().subVector(0, numberControlledJoints-1);

//    double t1 = Time::now();
//    Matrix LAMBDA = computeLAMBDA2();
//    double t2 = Time::now();
//    fprintf(stderr,"--------------------------------Time: %lf\n",t2-t1);
//    fprintf(stderr,"LAMBDA2: \n%s \n", LAMBDA.toString().c_str());
// ************ IMPORTANT: ACTUAL CONTROL LAW IMPLEMENTATION HERE *************
//    cmdTorques = G + JTcut*Fd; // Warning: open loop implementation
// ****************************************************************************

//  cmdTorques = G + JTcut*LAMBDA*Fd; // Warning: open loop implementation
    cmdTorques = G ;
//    Vector torquesP = JTcut*((Kp)*(Fd-f_r));  // pay attention to the multiplication order
//    Vector torquesD = -1.0*Kd*dq.subVector(0, numberControlledJoints-1);
//  cmdTorques = torquesP + torquesD + G;
//    cmdTorques = G + JT_r*((Kp)*(Fd-f_r)) - Kd*(dq); //BAD

#ifdef SEND_COMMANDS
    if (verbose)
        fprintf(stderr, "(REAL) Trying to send gtrqs: %s \n", cmdTorques.toString().c_str());
    for (int i = 0; i < numberControlledJoints; i++)
        if (fabs(cmdTorques.data()[i]) > THRESHOLD)
            fprintf(stderr, "(REAL) WARNING: not sending, saturating the current threshold!\n");
        else  // normal case
            itrq->setRefTorque(i, cmdTorques.data()[i]);
#else
    if (verbose)
        fprintf(stderr, "(SIM) Trying to send gtrqs: %s \n", cmdTorques.toString().c_str());
    for (int i = 0; i < numberControlledJoints; i++)
        if (fabs(cmdTorques.data()[i]) > THRESHOLD)
            fprintf(stderr, "(SIM) WARNING: not sending, saturating the current threshold on %d!\n", i);
#endif

}

bool cartesianForce::interrupt() {
    return true;
}


bool cartesianForce::close()
{
    printf("In class close().\n");

// Count on user stopping; if this is functional it should at least check for existence
//#ifdef SEND_COMMANDS
//	for(int i = 0; i < numberControlledJoints; i++)
//		ictrl->setPositionMode(i);
//	printf("(REAL) Set to Position mode.\n");
// count on user homing
//	pos->positionMove(home_deg.data());
//	printf("(REAL) Moved to: %s \n", home_deg.toString().c_str());
//#else
//	printf("(SIM) Set to Position mode.\n");
//#endif

    if (port_ext_contacts)   {
        port_ext_contacts->interrupt();
        port_ext_contacts->close();
        delete port_ext_contacts;
        port_ext_contacts = 0;
    }
    if (armChain)           {
        delete armChain;
        armChain=0;
    }
    if (linEst)             {
        delete linEst;
        linEst = 0;
    }
    if (torsoChain)         {
        delete torsoChain;
        torsoChain=0;
    }

    robotDevice.close();

    return true;
}


Vector cartesianForce::evalVel(const Vector &x)
{
    iCub::ctrl::AWPolyElement el;
    el.data=x;
    el.time=Time::now();

    return linEst->estimate(el);
}

Vector cartesianForce::projectSixDimVector(const Vector &v, const Matrix &R) {
    Vector res = R*v.subVector(0,2);
    Vector v2 = v.subVector(3,5);
    res.push_back(dot(R.getRow(0),v2));
    res.push_back(dot(R.getRow(1),v2));
    res.push_back(dot(R.getRow(2),v2));
    return res;
}

Matrix cartesianForce::projectJacobian(const Matrix &J, const Matrix &R) {
    Matrix R6(6,6);
    Vector r_new, r_old;
    Vector z3;
    z3.resize(3, 0.0);
    for (int i=0; i<6; i++) {
        if (i>2) {
            r_old = R.getRow(i-3);
            r_new = z3;
            r_new.push_back(r_old(0));
            r_new.push_back(r_old(1));
            r_new.push_back(r_old(2));
        } else {
            r_new = R.getRow(i);
            r_new.push_back(0.0);
            r_new.push_back(0.0);
            r_new.push_back(0.0);
        }
        R6.setRow(i, r_new);
    }
    return R6*J;
}

Matrix cartesianForce::computeLAMBDA2() { // [Lilly93] notation.
    // Step 1. Calculate I_i. 0-4.
    deque<Matrix> I_i;
    Matrix _I_i(6,6);
    double _m;
    for (int k=0; k<N; k++) {
        _I_i.zero();
        _m = armChain->getMass(k);
        _I_i(3,3) = _I_i(4,4) = _I_i(5,5) = _m;
        Matrix _topLeft = armChain->getInertia(k);  // RChackers tell us getInertia() is at COG (COG=ci).
        // Parallel axis theorem for inertias: I (x) = I (y) − m p(x) p(x) + m p(y) p(y)  // from [Jain11]
        // -> I(joint) = I(com) − _m * dot()vector3(joint)*dot()vector3(joint) + dot()vector3(com)*dot()vector3(com)// from [Jain11]
        Matrix _H_i_ci = armChain->getCOM(k);
        Vector _l_i_ci(3);
        _l_i_ci(0) = _H_i_ci(0,3);
        _l_i_ci(1) = _H_i_ci(1,3);
        _l_i_ci(2) = _H_i_ci(2,3);
        _topLeft = _topLeft + _m * makeCross(_l_i_ci) * makeCross(_l_i_ci);
        _I_i.setSubmatrix(_topLeft,0,0);
        I_i.push_back(_I_i);
//        printf("I_i[%d]:\n%s\n",k,I_i[k].toString().c_str());
    }
    // Step 2. Calculate Phi_i -> we'll use getS(int);
    // Step 3. Calculate X_h_i adapted to how it will be used
    deque<Matrix> X_h_i;
    Matrix _X_h_i(6,6);
    Matrix _R_h_i(3,3);
    for (int k=0; k<N; k++) {
        _R_h_i = getRelR(k,k+1);
        _X_h_i.setSubmatrix(makeCross(_R_h_i*getVector3(k,k+1)),3,0);
        _X_h_i.setSubmatrix(_R_h_i,0,0);
        _X_h_i.setSubmatrix(_R_h_i,3,3);
        X_h_i.push_back(_X_h_i); // Start at 0, go up to N-1
//        printf("X_h_i[%d]:\n%s\n",k,X_h_i[k].toString().c_str());
    }
    // Step 4. Algorithm (h=i-1)
    deque<Matrix> LAMBDA_i_i;
    LAMBDA_i_i.push_back(I_i[0]);  // LAMBDA_i_i[0]
    for (int k=1; k<N; k++) { // 1,2,3,4
        Matrix _LAMBDA_i_h = X_h_i[k-1].transposed() * LAMBDA_i_i[k-1] * X_h_i[k-1]; // (X_i-1_i)
        Matrix _K_i_h = getS(k) * pinv(getS(k).transposed()*_LAMBDA_i_h*getS(k)) * getS(k).transposed();
        Matrix _L_i_h = eye6x6 - ( _K_i_h * _LAMBDA_i_h );
        LAMBDA_i_i.push_back(I_i[k] + (_LAMBDA_i_h * _L_i_h));
//        printf("LAMBDA[%d]:\n%s\n",k,LAMBDA_i_i[k].toString().c_str());
    }
    return LAMBDA_i_i[4];
}

/*getX(const int& first, const int& second) {
    Matrix ret(6,6);
    ret.zero();
    _R_f_s = _H_i_ci.submatrix(0,2,0,2);
    _X_i_ci.setSubmatrix(_R_i_ci,0,0);
    _X_i_ci.setSubmatrix(_R_i_ci,3,3);

}*/

Matrix cartesianForce::computeLAMBDA3() { // Let's do it the Khatib way! (all refs [Khatib00])(more at [Chang99])
    // Step 0-A. Calculate I_ci. 0-4.
    // (what Khatib calls spatial inertia matrix, at link COGs) (ec.14)
    deque<Matrix> I_ci;
    Matrix _I_ci(6,6);
    double _m;
    for (int k=0; k<N; k++) {
        _I_ci.zero();
        _m = armChain->getMass(k);
        _I_ci(0,0) = _I_ci(1,1) = _I_ci(2,2) = _m;
        Matrix _bottomRight = armChain->getInertia(k);  // RChackers tell us getInertia() is at COG (COG=ci).
        _I_ci.setSubmatrix(_bottomRight,3,3);
        I_ci.push_back(_I_ci);
        printf("I_ci[%d]:\n%s\n",k,I_ci[k].toString().c_str());
    }
    // Step 0-B. Calculate X_i_ci. 0-4.
    // (spatial transformation matrices from joints to corresponding link COG) (ec.15)
    deque<Matrix> X_i_ci;
    Matrix _X_i_ci(6,6);
    Matrix _H_i_ci(4,4);
    Matrix _R_i_ci(4,4);
    Vector _l_i_ci(3);
    for (int k=0; k<N; k++) {
//        _H_i_ci = _H_i_r * _H_r_ci; // Not true!!!
        _H_i_ci = armChain->getCOM(k); // true!!
        _R_i_ci = _H_i_ci.submatrix(0,2,0,2); // Should actually be an eye3x3, put an assert??
        _X_i_ci.setSubmatrix(_R_i_ci,0,0);
        _X_i_ci.setSubmatrix(_R_i_ci,3,3);
        _l_i_ci(0) = _H_i_ci(0,3);
        _l_i_ci(1) = _H_i_ci(1,3);
        _l_i_ci(2) = _H_i_ci(2,3);
        _X_i_ci.setSubmatrix(makeCross(_l_i_ci)*_R_i_ci,3,0);
        X_i_ci.push_back(_X_i_ci);
        printf("X_i_ci[%d]:\n%s\n",k,X_i_ci[k].toString().c_str());
    }
    // Step 0-C. Calculate I_i. 0-4.
    // (spatial inertia matrix, at corresponding joints) (ec.16)
    deque<Matrix> I_i;
    for (int k=0; k<N; k++) {
        I_i.push_back(X_i_ci[k] * I_ci[k] * X_i_ci[k].transposed());
        printf("I_i[%d]:\n%s\n",k,I_i[k].toString().c_str());
    }    

    // Step 1. Outward recursion: Compute X_h_i. 0-3 to 1-4 represented as 0-3. 4-ee represented as 4.
    // (it's like a joint-to-joint PHI, more spatial transformation matrices) (ec.15)
    deque<Matrix> X_h_i;
    Matrix _X_h_i(6,6);
    Matrix _R_h_i(3,3);
    for (int k=0; k<N; k++) {
        _R_h_i = getRelR(k,k+1);
        _X_h_i.setSubmatrix(makeCross(getVector3(k,k+1))*_R_h_i,3,0);
        _X_h_i.setSubmatrix(_R_h_i,0,0);
        _X_h_i.setSubmatrix(_R_h_i,3,3);
        X_h_i.push_back(_X_h_i); // Start at 0, go up to N-1
        printf("X_h_i[%d]:\n%s\n",k,X_h_i[k].toString().c_str());
    }
    // Step 2. Inward recursion: Compute L_h_i. 4-1 to 3-0 represented as 3-0. ee-4 represented as 4.
    // (these are force propagators, work from link to parent link) (ec.21)
    deque<Matrix> L_h_i;
    L_h_i.resize(4);
    deque<Matrix> Dinv;
    Dinv.resize(5);
    deque<Matrix> I_A;  // (articulated-body inertia matrix of link i) (ec.20)
    I_A.resize(5);
    Matrix _S_flat(1,6);  // (generalized inverse of S_i) (ec.21c)
    Matrix _D(1,1);  // (projects I_A into the motion space of joint i) (ec.21c)
    I_A[4] = I_i[4];  // (ec.20)
    fprintf(stderr,"I_A[4] = I_i[4]\n");
    for (int k=N-1; k>0; k--) {
        _D = getS(k).transposed() * I_A[k] * getS(k);  // (ec.21c)
        fprintf(stderr,"D[%d]:\n%s\n",k,_D.toString().c_str());
        Dinv[k] = pinv(_D);
        fprintf(stderr,"Dinv[%d]:\n%s\n",k,Dinv[k].toString().c_str());
        _S_flat = Dinv[k] * getS(k).transposed() * I_A[k];  // (ec.21b)
        fprintf(stderr,"_S_flat:\n%s\n",_S_flat.toString().c_str());
        L_h_i[k-1] = X_h_i[k-1] * (eye6x6 - getS(k)*_S_flat).transposed();  // (ec.21a)
        fprintf(stderr,"L_h_i[%d]:\n%s\n",k-1,L_h_i[k-1].toString().c_str());
        I_A[k-1] = I_i[k-1];  // (ec.20)
        fprintf(stderr,"I_A[%d] = I_i[%d]\n",k-1,k-1);
        for(int j=k;j<N;j++) {  // (ec.20)
            fprintf(stderr,"I_A[%d] = I_A[%d] + L_h_i[%d]*I_A[%d]*L_h_i[%d].transposed()\n",k-1,k-1,j-1,j,j-1);
            I_A[k-1] = I_A[k-1] + L_h_i[j-1]*I_A[j]*L_h_i[j-1].transposed();  // (ec.20)
        }
        fprintf(stderr,"I_A[%d]:\n%s\n",k-1,I_A[k-1].toString().c_str());
    }

    // Step 3. Outward recursion: Compute block diagonal starting with OMEGArootroot
    deque<Matrix> OMEGA;
    OMEGA.push_back(zero6x6);  // OMEGA[0]
    for (int k=1; k<N; k++) {  // OMEGA[1,2,3,4]
        OMEGA.push_back(getS(k)*Dinv[k]*getS(k).transposed() + L_h_i[k-1].transposed()*OMEGA[k-1]*L_h_i[k-1]);
        printf("OMEGA[%d]:\n%s\n",k,OMEGA[k].toString().c_str());
    }

    // Step 4. Outward recursion: Compute block off-diagonal
/*    deque<Matrix> OMEGAih;
    for (int k=0; k<N; k++) {
        OMEGAih.push_back(L_h_i[k].transposed()*OMEGAii[k]);
    }*/ // Not used

    // Step 5. Spatial transformation: Compute OMEGAinv_ei_ej from OMEGAi,j.
    Matrix LAMBDAm1 = X_h_i[4].transposed()*OMEGA[4]*X_h_i[4];
    printf("LAMBDAm1:\n%s\n",LAMBDAm1.toString().c_str());

    // Step 6. Matrix inversion.
    return pinv(LAMBDAm1);  // LAMBDA is LAMBDAm1's inverse
}

Matrix cartesianForce::computeLAMBDA() { // calculates LAMBDA, M seen from EE (a.k.a. M in OpSpace)
    
    // Note: we use [Jain2011][Rodriguez87] for LambdaM, [Rodriguez1992b] for central algorithm LambdaP
    // etc, LambdaJ for what is described as H in [Rodriguez1987], and finally [Rodriguez1989] for OMEGA
    // and therefore LAMBDA.
    //
    // -------------- LambdaM first, as we need it for LambdaP for OMEGA for LAMBDA --------------
    //
    // LambdaM[k] =  [            LambdaI(k)                 Lambdam(k)*vector3_cross(k,k_com) ]
    //               [ -Lambdam(k)*vector3_cross(k,k_com)              Lambdam(k)*I            ]
    //
    deque<Matrix> LambdaM;
    LambdaM.push_back(zero6x6);
    double _Lambdam;
    Matrix _LambdaM(6,6);
    Matrix _LambdaI(3,3);
    Matrix _topRight(3,3);
    for (unsigned int k=1; k<N+1; k++) { // k=0 is not conemplated for P anyway.
        _LambdaM.eye();
        _Lambdam = armChain->getMass(N-k);  // k=1->4 ; k=2->3; k=3->2; k=4->1; k=5=N->0
        //printf("Lambdam[%d]:\n%f\n",k,_Lambdam);
        _LambdaM(3,3) = _LambdaM(4,4) = _LambdaM(5,5) = _Lambdam;
        //printf("_LambdaM[%d]:\n%s\n",k,_LambdaM.toString().c_str());
        // ----
        // Parallel axis theorem for inertias: I (x) = I (y) − m p(x) p(x) + m p(y) p(y)  // from [Jain11]
        // -> I(joint) = I(com) − _Lambam * dot()vector3(joint)*dot()vector3(joint) + dot()vector3(com)*dot()vector3(com)// from [Jain11]
        Matrix H_r_joint = armChain->getH(N-k);
        Vector vector3joint;
        vector3joint.push_back(H_r_joint(0,3));
        vector3joint.push_back(H_r_joint(1,3));
        vector3joint.push_back(H_r_joint(2,3));
        //
        Matrix H_r_com = armChain->getCOM(N-k);
        Vector vector3com;
        vector3com.push_back(H_r_com(0,3));
        vector3com.push_back(H_r_com(1,3));
        vector3com.push_back(H_r_com(2,3));
        _LambdaI = armChain->getInertia(N-k) - (_Lambdam * makeCross(vector3joint) * makeCross(vector3joint)) + (_Lambdam * makeCross(vector3com) * makeCross(vector3com));  // k=1->4 ; k=2->3; k=3->2; k=4->1; k=5=N->0
        //_LambdaI = armChain->getInertia(N-k); // k=1->4 ; k=2->3; k=3->2; k=4->1; k=5=N->0
        // ----
//        printf("_LambdaI[%d]:\n%s\n",k,_LambdaI.toString().c_str());
        _LambdaM.setSubmatrix(_LambdaI,0,0);
        _topRight = (_Lambdam)*(makeCross(getVector3com(N-k))); // k=0->5; k=1->4 ; ...; k=4->1; k=5=N->0
        //printf("_topRight[%d]:\n%s\n",k,_topRight.toString().c_str());
        _LambdaM.setSubmatrix(_topRight,0,3);
        //printf("_LambdaM[%d]:\n%s\n",k,_LambdaM.toString().c_str());
        _LambdaM.setSubmatrix(((-1)*(_topRight)),3,0);
        //printf("_LambdaM[%d]:\n%s\n",k,_LambdaM.toString().c_str());
        LambdaM.push_back(_LambdaM); // Start at 1, go up to ...
        printf("LambdaM[%d]:\n%s\n",k,LambdaM[k].toString().c_str());
    }

    // -------------- LambdaP is up next -------------- 
    deque<Matrix> LambdaP;
    deque<Matrix> LambdaG;
    deque<Matrix> LambdaD;
    deque<Matrix> LambdaDpinv;
    deque<Matrix> LambdaJ;
    LambdaP.push_back(zero6x6);  // LambdaP[0]
    LambdaG.push_back(zero6x6);  // LambdaG[0]
    LambdaD.push_back(zero6x6);  // LambdaD[0]
    LambdaDpinv.push_back(zero6x6);  // LambdaDpinv[0]
    LambdaJ.push_back(zero6x6);  // LambdaJ[0] // called H by authors
    Matrix _PSI(6,6);
    Matrix _LambdaJ(6,6);
    _LambdaJ.zero();
    for (unsigned int k=1; k<N+1; k++) {   // LambdaP[1] and so on...
//        fprintf(stderr,"Iteration [k]: %d.\n",k);
        Matrix _H_r_joint = armChain->getH(N-k);
        Matrix _r_H_joint = SE3inv(_H_r_joint);
        Matrix _Jac_joint = projectJacobian(armChain->GeoJacobian(N-k),_r_H_joint);
        _LambdaJ.setSubmatrix(_Jac_joint,0,0); // Dirty little hack to add a null row
//        _LambdaJ = getLambdaH(); // WARNING: TEST
        LambdaJ.push_back(_LambdaJ);
//        printf("LambdaJ[%d]:\n%s\n",k,LambdaJ[k].toString().c_str());
        _PSI = getPHI(N-k,N-(k-1)) * (eye6x6 - LambdaG[k-1]*LambdaJ[k-1]);  // _PSI adapted to each case
        //printf("_PSI[%d]:\n%s\n",k,_PSI.toString().c_str());
        LambdaP.push_back(_PSI*LambdaP[k-1]*_PSI.transposed() + LambdaM[k]); // should go to LambdaP[k]
        //printf("LambdaP[%d]:\n%s\n",k,LambdaP[k].toString().c_str());
        LambdaD.push_back(LambdaJ[k]*LambdaP[k]*LambdaJ[k].transposed());
        printf("LambdaD[%d]:\n%s\n",k,LambdaD[k].toString().c_str());
        LambdaDpinv.push_back(pinv(LambdaD[k]));
        LambdaG.push_back(LambdaP[k]*LambdaJ[k].transposed()*LambdaDpinv[k]);
        //printf("LambdaG[%d]:\n%s\n",k,LambdaG[k].toString().c_str());
    }

    // -------------- OMEGAs up next -------------- 
    // Now iterate OMEGAs, because LAMBDA is OMEGA[0]'s inverse
    deque<Matrix> OMEGA;
    OMEGA.push_front(zero6x6);  // Should end up at N+1 if all goes well
    for (unsigned int k=N; k>0; k--) { //algoritm says to do from N, don't worry about DRACONIAN
        //fprintf(stderr,"Iteration [k]: %d.\n",k);
        _PSI = getPHI(N-(k+1),N-k) - (eye6x6 - LambdaG[k]*LambdaJ[k]);  // _PSI adapted to each case (h-c macro)
        // now we use OMEGA[0] instead of algorithm OMEGA[k+1] to get front of deque.
        //printf("iteration[%d]:\n",k);
        //printf("OMEGA[0]:\n%s\n",OMEGA[0].toString().c_str());
        OMEGA.push_front(_PSI.transposed()*OMEGA[0]*_PSI + LambdaJ[k].transposed()*LambdaDpinv[k]*LambdaJ[k]);
        //printf("OMEGA[0]:\n%s\n\n",OMEGA[0].toString().c_str());
    }
    // now we use OMEGA[0] instead of algorithm OMEGA[1] to get front of deque.
    OMEGA.push_front(getPHI(N-1,N-0).transposed()*OMEGA[0]*getPHI(N-1,N-0));
    return pinv(OMEGA[0]);  // LAMBDA is OMEGA[0]'s inverse
    // More terrible hacks: OMEGA's right col and bottom row are null: bad pinv. Let's remove them!!
//    Matrix OMEGAcut = OMEGA[0].submatrix(0,4,0,4);
//    Matrix OMEGAcutpinv = pinv(OMEGAcut);
//    Matrix ret(6,6);
//    ret.zero();
//    ret.setSubmatrix(OMEGAcutpinv,0,0);
//    return ret;
}

// Note: not used, gives bogus results. Instead use each joint jacobian projected on joint frame.
Matrix cartesianForce::getLambdaH(const int& jointNumber) { // for now I think all rotate around Z
    Matrix ret(1,6);
    ret.zero();
    ret(0,5)=1.0;
    return ret;
}

Matrix cartesianForce::getS(const int& joint) {  // All around Z
    Matrix ret(6,1);
    ret.zero();
    ret(5,0)=1.0;
    return ret;
}

Matrix cartesianForce::getPHI(const int& first,const int& second) {  // calculate Rigid Body Transformation Matrix
    Matrix ret(6,6);
//    ret.eye();  // Now we have a 6x6 identity
    Vector l = getVector3(first,second);
//    Matrix l_cross = makeCross(l);
//    ret.setSubmatrix(l_cross,0,3);
// NEW(begin): including Rotation
    Matrix relR = getRelR(first,second);
    ret.setSubmatrix(relR,0,0);
    ret.setSubmatrix(relR,3,3);
    Matrix l_cross = makeCross(l);
    ret.setSubmatrix(l_cross*relR,0,3);
// NEW(end): including Rotation
    return ret;
}

Matrix cartesianForce::getRelR(const int& first,const int& second) {  // Rot Matrix(3,3) from "first" to "second"
    Matrix H_r_first = armChain->getH(first);
    Matrix H_first_r = SE3inv(H_r_first);
    Matrix H_r_second = armChain->getH(second);
    Matrix H_first_second = H_first_r * H_r_second;
    return H_first_second.submatrix(0,2,0,2);
}

Vector cartesianForce::getVector3(const int& first,const int& second) {  // "l" Vector(3) from "first" to "second"
    Vector ret(3);
    if((first<0)||(second<0)) {
//        fprintf(stderr,"DRACONIAN ERROR calling getVector3()\n");
        ret.zero();
        return ret;
    }
    Matrix H_r_first = armChain->getH(first);
    Matrix H_first_r = SE3inv(H_r_first);
    Matrix H_r_second = armChain->getH(second);
    Matrix H_first_second = H_first_r * H_r_second;
    ret(0) = H_first_second(0,3);
    ret(1) = H_first_second(1,3);
    ret(2) = H_first_second(2,3);
    return ret;
}

Matrix cartesianForce::getRelRcom(const int& joint) {  // Rot Matrix(3,3) from "joint" to link com
    Matrix H_r_joint = armChain->getH(joint);
    Matrix H_joint_r = SE3inv(H_r_joint);
    Matrix H_r_com = armChain->getCOM(joint);
    Matrix H_joint_com = H_joint_r * H_r_com;
    return H_joint_com.submatrix(0,2,0,2);
}

Vector cartesianForce::getVector3com(const int& joint) {  // "l" Vector(3) from "joint" to link com
    Vector ret(3);
    if(joint<0) {
//        fprintf(stderr,"DRACONIAN ERROR calling getVector3com()\n");
        ret.zero();
        return ret;
    }
    Matrix H_r_joint = armChain->getH(joint);  // To compute LAMBDA we want 0->4 (Force EE), 1->3, 4->0.
    Matrix H_joint_r = SE3inv(H_r_joint);
    Matrix H_r_com = armChain->getCOM(joint);
    Matrix H_joint_com = H_joint_r * H_r_com;
    ret(0) = H_joint_com(0,3);
    ret(1) = H_joint_com(1,3);
    ret(2) = H_joint_com(2,3);
    fprintf(stderr,"Dist from joint[%d] to joint[%d]com (std icub notation): %s\n",joint,joint,ret.toString().c_str());
    return ret;
}

Matrix cartesianForce::makeCross(const Vector& in) {  // creates skew-sym equivalent to cross product
    Matrix ret(3,3);
    ret.zero();
    ret(0,1) = -in(2);  // a     0 -c  b
    ret(0,2) =  in(1);  // b ->  c  0 -a
    ret(1,0) =  in(2);  // c    -b  a  0
    ret(1,2) = -in(0);
    ret(2,0) = -in(1);
    ret(2,1) =  in(0);
    return ret;
}

