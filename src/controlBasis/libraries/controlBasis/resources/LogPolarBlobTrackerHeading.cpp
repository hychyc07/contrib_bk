// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#include "LogPolarBlobTrackerHeading.h"
#include <yarp/os/Network.h>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void CB::LogPolarBlobTrackerHeading::startResource() {
    start();     // mandatory start function
}

void CB::LogPolarBlobTrackerHeading::stopResource() {
    stop();     // mandatory stop function
}

bool CB::LogPolarBlobTrackerHeading::updateResource() {
  
    Bottle *b;
    bool ok = true;
    Vector sample(2);
    Vector vel(2);

    // if not connected to the configuration resource, do that now
    if(!connectedToBlobTracker) {
        if(!connectToBlobTracker()) {
            cout << "LogPolarBlobTrackerHeading couldn't connect to ports in startResource()..." << endl;
            return false;
        }
    }

    // read the current configuration variables from the input port
    b = inputPort[0]->read(true);

    if(b!=NULL) {
        
        valid = (bool)(b->get(0).asInt());

        if(valid) {
            Bottle *blob = b->get(blobID+2).asList(); 
            values[0] = blob->get(0).asDouble()*TORAD - M_PI/2.0; // phi (need to convert to radians and put in [-pi,pi) range, centered at the vertical axis
            values[1] = blob->get(1).asDouble(); // rho
            headingVelocity[0] = blob->get(2).asDouble();
            headingVelocity[1] = blob->get(3).asDouble();            
        } else {            
            //cout << "No Blob Headings Found" << endl;        
        }

    } 

    return ok;
    
}

bool CB::LogPolarBlobTrackerHeading::connectToBlobTracker() {
    
    bool ok = true;

    if((int)inputPort.size() != numInputs) {
        return false;
    }    

    // connect to config port for reading config values
    string coordinatesOutputName;

    coordinatesOutputName = deviceName + "/blobs:o";       
    
    cout << "LogPolarBlobTrackerHeading::connectToBlobTracker() -- connecting ports for image coordinate values..." << endl;

    cout << "blob port: " << coordinatesOutputName << endl;
    ok &= Network::connect(coordinatesOutputName.c_str(),inputPortName[0].c_str(),"tcp");
    if(!ok) {
        cout << "LogPolarBlobTrackerHeading::connectToBlobTracker() -- connecting to image coordinates (1) port failed..." << endl;
        return ok;
    }

    connectedToBlobTracker = true;
    
    return ok;
}

yarp::sig::Vector CB::LogPolarBlobTrackerHeading::getHeadingVelocity() {
    return headingVelocity;
}
