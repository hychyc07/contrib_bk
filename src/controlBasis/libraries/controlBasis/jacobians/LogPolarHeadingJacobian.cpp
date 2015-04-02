// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#include "LogPolarHeadingJacobian.h"
#include <yarp/os/Network.h>
#include <yarp/math/Math.h>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

void CB::LogPolarHeadingJacobian::startJacobian() {
    start();     // mandatory start function
}

void CB::LogPolarHeadingJacobian::stopJacobian() {
    stop();     // mandatory stop function
}

bool CB::LogPolarHeadingJacobian::updateJacobian() {

    bool ok = true;
    double offset;
    Bottle *b;

    double rho, phi;

    double DpanDrho;
    double DpanDphi;
    double DtiltDrho;
    double DtiltDphi;

    double scale_factor = 0.01;

    // if not conencted to inputs, do that now
    if(!connectedToInputs) {
        if(!connectToInputs()) {
            cout << "LogPolarHeadingJacobian couldn't connect to input ports in startJacobian()..." << endl;
            return false;
        }
    }  

    // get the current logpolar coordinate from camera
    b = logPolarPort.read();
    if(b==NULL) {
        cout << "LogPolarHeadingJacobian::update() -- having trouble reading input data!" << endl;
        return ok;
    }
    valid = (bool)(b->get(1).asInt());
    if(!valid) return ok;
    
    // get the data
    offset = 2;  
    for(int i=0; i<2; i++) {
        logPolarVals[i] = b->get(i+offset).asDouble();
    }

    phi  = logPolarVals[0];
    rho  = logPolarVals[1];

    DpanDrho =   cos(phi);
    DpanDphi =  -sin(phi);
    DtiltDrho =  sin(phi);
    DtiltDphi =  cos(phi);

    J[0][0] =  DpanDphi;
    J[0][1] =  DpanDrho;
    J[1][0] =  -DtiltDphi;  
    J[1][1] =  -DtiltDrho;
    J[2][0] =  0;
    J[2][1] =  0;

    cout << "J_logpolar:" << endl;
    for(int i=0; i<2; i++) {
        cout << "[ ";
        for(int j=0; j<2; j++) {
            cout << J[i][j] << " ";
        }
        cout << "]" << endl;
    }
    cout << endl;

    J = J*scale_factor;
    valid = true;

    return ok; 
}

bool CB::LogPolarHeadingJacobian::connectToInputs() {
    
    bool ok = true;

    if(connectedToInputs) return true;

    // configue a random ID for this jacobian instance 
    int randomID = (int)(Random::uniform()*1000.0);
    char *c = (char *)malloc(16);
    sprintf(c,"%d", randomID);
    string randomIDstr(c);

    string header = "/cb/logpolarheading";
    string sensorDevice = sensorName.substr(header.size());
    // connect to link port for configuration resource
    string logPolarOutputName = "/cb/logpolarheading" + sensorDevice + "/data:o";
    string logPolarInputName = "/cb/logpolarheadingjacobian/" + randomIDstr + sensorDevice + "/data:i";
     
    cout << "LogPolarJacobian sensor: " << sensorDevice << endl;

    cout << "LogPolarHeadingJacobian::connectToInputs() -- opening input port[" << logPolarInputName << "] for logpolar coordinates..." << endl;
    ok &= logPolarPort.open(logPolarInputName.c_str());
    if(!ok) {
        cout << "LogPolarHeadingJacobian::connectToInputs() -- could not open port for logpolar input coordinates!!" << endl;
        return ok;
    }

    cout << "LogPolarHeadingJacobian::connectToInputs() -- connecting ports for logpolar coordinates[" << logPolarOutputName << "]..." << endl;
    ok &= Network::connect(logPolarOutputName.c_str(),logPolarInputName.c_str(), "tcp");
    if(!ok) {
        cout << "LogPolarHeadingJacobian::connectToInputs() -- could not connect to logpolar input coordinates port!!" << endl;
        return ok;
    }
    
    connectedToInputs = true;   
    return true;
}
