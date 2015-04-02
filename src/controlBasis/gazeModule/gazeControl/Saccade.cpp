// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      

#include "Saccade.h"
#include <yarp/os/Random.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;
using namespace gc;

void Saccade::setVelocities() {

    string eStr = device->getDeviceName();
    cout << "Setting Saccade velocities for: " << eStr.c_str() << endl;

    if(eStr.find("head") != string::npos) {

        device->setJointReferenceSpeed(0,50);
        device->setJointReferenceSpeed(2,50);

        device->setJointVelocityDirect(0,0);
        device->setJointVelocityDirect(2,0);
        
    } else if(eStr.find("eyes-pt") != string::npos ) {

        device->setJointReferenceSpeed(3,100);
        device->setJointReferenceSpeed(4,100);

        device->setJointVelocityDirect(3,0);
        device->setJointVelocityDirect(4,0);

    } else if(eStr.find("eyes-ptv") != string::npos ) {

        device->setJointReferenceSpeed(3,100);
        device->setJointReferenceSpeed(4,100);
        device->setJointReferenceSpeed(5,100);

        device->setJointVelocityDirect(3,0);
        device->setJointVelocityDirect(4,0);
        device->setJointVelocityDirect(5,0);
    }

    potentialDot = 0.0;
    potentialStore.clear();

}

void Saccade::run() {

    Vector configData;
    Vector Vr;
    Vector Vn;
    double scaleFactor;
    double var = 0.01;

    Vector Vp(6);
    Vector Vd(6);
    Vector Va(6);
    double tmp_ds;
    double tmp_a;
    double tmp_w;

    Vector mask(6);
    mask.zero();

    // evaluate the state
    evaluateState();
    if(statePredicate == UNDEFINED) {
        return;
    }

    string eStr = device->getDeviceName();
    configData = device->getResourceData();
    
   //    yarp::dev::IEncoders *enc = device->getEncoderInterface();
 
    if(eStr.find("head") != string::npos) {

        mask[0] = 1;
        mask[2] = 1;
        mass = 10.0;

        scaleFactor = 0.8;

        device->setJointReferenceSpeed(0,50);
        device->setJointReferenceSpeed(2,50);
        
        Vr.resize(2);
        Vr[0] = configData[0] - (blobData[1]*scaleFactor);
        Vr[1] = configData[1] - (blobData[0]*scaleFactor);

        Vn.resize(2);
        Vn[0] = Random::normal(1,var);
        Vn[1] = Random::normal(1,var);

        device->setJointPositionDirect(0,Vr[0]*Vn[0]);
        device->setJointPositionDirect(2,Vr[1]*Vn[1]);
        
    } else if(eStr.find("eyes-pt") != string::npos ) {

        mask[3] = 1;
        mask[4] = 1;
        mass = 1.0;

        scaleFactor = 0.75;

        device->setJointReferenceSpeed(3,100);
        device->setJointReferenceSpeed(4,100);

        Vr.resize(2);               
        Vr[0] = configData[0] - (blobData[1]*scaleFactor);
        Vr[1] = configData[1] + (blobData[0]*scaleFactor);

        Vn.resize(2);
        Vn[0] = Random::normal(1,var);
        Vn[1] = Random::normal(1,var);

        device->setJointPositionDirect(3,Vr[0]*Vn[0]);
        device->setJointPositionDirect(4,Vr[1]*Vn[1]);
        

    } else if(eStr.find("eyes-ptv") != string::npos ) {

        mask[3] = 1;
        mask[4] = 1;
        mask[5] = 1;
        mass = 1.0;

        scaleFactor = 0.25;

        device->setJointReferenceSpeed(3,100);
        device->setJointReferenceSpeed(4,100);
        device->setJointReferenceSpeed(5,100);

        Vr.resize(3);               
        Vr[0] = configData[1] - (blobData[1]*scaleFactor);
        Vr[1] = configData[0] + (blobData[0]*scaleFactor)/2.0;
        Vr[2] = configData[2] - (blobData[0]*scaleFactor)/2.0;

        Vn.resize(3);               
        Vn[0] = Random::normal(1,var);
        Vn[1] = Random::normal(1,var);
        Vn[2] = Random::normal(1,var);

        device->setJointPositionDirect(3,Vr[0]*Vn[0]);
        device->setJointPositionDirect(4,Vr[1]*Vn[1]);
        device->setJointPositionDirect(5,Vr[2]*Vn[2]);
        
    }

    // calcuate cost
    cost = 0;
    dT = dW = 0;
    dt=0;

    for(int i=0; i<100; i++) {       
        
        device->getJointPositionsDirect(Vp);
        Vp = Vp*(M_PI/180.0);

        Time::delay(0.01);

        if(i==0) {
            t0=yarp::os::Time::now();
            t1 = t0;
            dt = dT = 0;
        }

        if(i>0) {

            t1=yarp::os::Time::now();
            dt = t1 - t0;
            t0 = t1;  
            dT += dt;

            Vd = (Vp - VpLast)*(1.0/dt);

            if(i>1) {                
                Va = (Vd - VdLast)*(1.0/dt);               

                tmp_a = 0;
                for(int k=0; k<Va.size(); k++) {
                    if(mask[k]) {
                        tmp_a += (Va[k]*Va[k]);
                    }
                }
                tmp_a = sqrt(tmp_a);
                
                tmp_ds = 0;
                for(int k=0; k<Vd.size(); k++) {
                    if(mask[k]) {
                        tmp_ds += (Vd[k]*Vd[k]);
                    }
                }
                tmp_ds = sqrt(tmp_ds);
                
                tmp_w = (mass * tmp_a * tmp_ds);

                if(tmp_w >= 1e-10) {
                    //cout << "F: " << tmp_a << ", dS: " << tmp_ds << ", dw: " << tmp_w << endl;
                    dW += tmp_w;
                }
            }
            VdLast = Vd;
        }
        VpLast = Vp;
    }

    dW /= 100000.0;
    cost = dW;
    //cout << "Saccade cost (work): " << dW << ", (time): " << dT << endl;

    // SACCADE is a 1-time thing (an open-loop burst),
    // so just set the state to converged after the 
    // movement is sent to the robot.
    statePredicate = CONVERGED;
    stop();
}

void Saccade::evaluateCost() {
    cost = 0;
}
