// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include "SmoothPursuit.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;
using namespace gc;

void SmoothPursuit::setVelocities() {

    //Vector vel(6);
    string eStr = device->getDeviceName();
    cout << "Setting SmoothPursuit velocities for: " << eStr.c_str() << endl;

    //vel.zero();
    //    device->setJointVelocityDirect(vel);

    if(eStr.find("head") != string::npos) {

        device->setJointReferenceSpeed(0,5);
        device->setJointReferenceSpeed(2,5);

        device->setJointVelocityDirect(0,0);
        device->setJointVelocityDirect(2,0);
        
    } else if(eStr.find("eyes-pt") != string::npos ) {

        device->setJointReferenceSpeed(3,5);
        device->setJointReferenceSpeed(4,5);

        device->setJointVelocityDirect(3,0);
        device->setJointVelocityDirect(4,0);

    } else if(eStr.find("eyes-ptv") != string::npos ) {

        device->setJointReferenceSpeed(3,5);
        device->setJointReferenceSpeed(4,5);
        device->setJointReferenceSpeed(5,5);

        device->setJointVelocityDirect(3,0);
        device->setJointVelocityDirect(4,0);
        device->setJointVelocityDirect(5,0);
    }

    potentialDot = 0.0;
    potentialStore.clear();
    
}


void SmoothPursuit::run() {

    Vector configData;
    Vector Vr;
    double scaleFactor = 0.5;
    double velScaleFactor = 250;

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
        //   cout << "action[" << id << "] undefined..." << endl;
        Time::delay(0.1);
        return;
    }

    string eStr = device->getDeviceName();
    configData = device->getResourceData();

    if(eStr.find("head") != string::npos) {

        mask[0] = 1;
        mask[2] = 1;
        
        mass = 10.0;

        /*
        Vr.resize(2);
        Vr[0] = configData[0] - (blobData[1]*scaleFactor);
        Vr[1] = configData[1] - (blobData[0]*scaleFactor);
        */

        Vr.resize(2);
        Vr[0] = -(blobData[1]*scaleFactor)*velScaleFactor;
        Vr[1] = -(blobData[0]*scaleFactor)*velScaleFactor;

        device->setJointVelocityDirect(0,Vr[0]);
        device->setJointVelocityDirect(2,Vr[1]);
               
    } else if(eStr.find("eyes-pt") != string::npos) {

        mask[3] = 1;
        mask[4] = 1;
        mass = 1.0;

        device->setJointReferenceSpeed(3,5);
        device->setJointReferenceSpeed(4,5);

        /*
        Vr.resize(2);               
        Vr[0] = configData[1] - blobData[1];
        Vr[1] = configData[0] + blobData[0];

        device->setJointPositionDirect(3,Vr[0]);
        device->setJointPositionDirect(4,Vr[1]);
        */

        Vr.resize(2);
        Vr[0] = -blobData[1] * velScaleFactor;
        Vr[1] =  blobData[0] * velScaleFactor;

        device->setJointVelocityDirect(3,Vr[0]);
        device->setJointVelocityDirect(4,Vr[1]);

    } else if(eStr.find("eyes-ptv") != string::npos ) {

        mask[3] = 1;
        mask[4] = 1;
        mask[5] = 1;
        mass = 1.0;

        /*
        Vr.resize(3);               
        Vr[0] = configData[1] - blobData[1];
        Vr[1] = configData[0] + blobData[0]/2.0;
        Vr[2] = configData[2] - blobData[0]/2.0;

        device->setJointPositionDirect(3,Vr[0]);
        device->setJointPositionDirect(4,Vr[1]);
        device->setJointPositionDirect(5,Vr[2]);
        */

        Vr.resize(2);
        Vr[0] = -blobData[1] * velScaleFactor;
        Vr[1] =  blobData[0]/2.0 * velScaleFactor;
        Vr[2] = -blobData[0]/2.0 * velScaleFactor;

        //cout << "setting velocity: (" << Vd[0] << "," << Vd[1] << "," << Vd[2] << ")" << endl;

        device->setJointVelocityDirect(3,Vr[0]);
        device->setJointVelocityDirect(4,Vr[1]);
        device->setJointVelocityDirect(5,Vr[2]);
        
    }

    device->getJointPositionsDirect(Vp);
    Vp = Vp*(M_PI/180.0);

    if(potentialStore.size()==1) {
        t0=yarp::os::Time::now();
        t1 = t0;
        dT = dt = 0;
    }
    
    if(potentialStore.size()>1) {

        t1=yarp::os::Time::now();
        dt = t1 - t0;
        t0 = t1;  
        dT += dt;
        
        Vd = (Vp - VpLast)*(1.0/dt);
        
        if(potentialStore.size()>2) {                
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
                
            tmp_w = (mass * tmp_a * tmp_ds)/10.0;
            
            if(tmp_w >= 1e-10) {
                //cout << "F: " << tmp_a << ", dS: " << tmp_ds << ", dw: " << tmp_w << endl;
                dW += tmp_w;
            }
        }
        VdLast = Vd;
    }
    VpLast = Vp;

    //    if(statePredicate == CONVERGED) {
        //        stop();
    cost = dW;
    //cout << "SmoothPursuit cost (work): " << dW << ", (time): " << dT << endl;
        //}

}

void SmoothPursuit::evaluateCost() {
    cost = 0;
}
