// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#include "ParticleFilterTrackerHeading.h"
#include <yarp/os/Network.h>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void CB::ParticleFilterTrackerHeading::startResource() {
    start();     // mandatory start function
}

void CB::ParticleFilterTrackerHeading::stopResource() {
    stop();     // mandatory stop function
}

bool CB::ParticleFilterTrackerHeading::updateResource() {
  
    Bottle *b;
    bool ok = true;
    Vector sample(2);
    Vector vel(2);
    double focal_length = 0.0225;
    double meters_per_pixel = 0.0001;
    double width = 320.0;
    double height = 240.0;

    // if not connected to the configuration resource, do that now
    if(!connectedToParticleFilterTracker) {
        if(!connectToParticleFilterTracker()) {
            cout << "ParticleFilterTrackerHeading couldn't connect to ports in startResource()..." << endl;
            return false;
        }
    }

    // read the current configuration variables from the input port
    b = inputPort[0]->read(true);

    if(b!=NULL) {

        if(zdfMode==LEFT) {
            if(b->get(2).asDouble() > threshold) {
                imageCoordinates[0] = (b->get(0).asInt() - width/2.0);
                imageCoordinates[1] = (b->get(1).asInt() - height/2.0);                
                valid = true;
            } else {
                valid = false;
            }
        } else if(zdfMode==RIGHT) {
            if(b->get(5).asDouble() > threshold) {
                valid = true;
                imageCoordinates[0] = b->get(3).asInt() - width/2.0;
                imageCoordinates[1] = b->get(4).asInt() - height/2.0;                
            } else {
                valid = false;
            }
        } else {
            if((b->get(2).asDouble() > threshold) && (b->get(5).asDouble() > threshold)) {
                valid = true;
                imageCoordinates[0] = b->get(0).asInt() - width/2.0;
                imageCoordinates[1] = b->get(1).asInt() - height/2.0;                
                imageCoordinates[0] += (b->get(3).asInt() - width/2.0);
                imageCoordinates[1] += (b->get(4).asInt() - height/2.0);
                imageCoordinates[0] /= 2.0;
                imageCoordinates[1] /= 2.0;
                valid = true;
            } else {
                valid = false;
            }
        }

        if(valid) {

            vel[0] = (imageCoordinates[0] - imageCoordinatesLast[0])/dt;
            vel[1] = (imageCoordinates[1] - imageCoordinatesLast[1])/dt;

            imageCoordinatesLast[0] = imageCoordinates[0];
            imageCoordinatesLast[1] = imageCoordinates[1];

            for(int i=0; i<2; i++) {
                values[i] = atan2(imageCoordinates[i]*meters_per_pixel,focal_length);
                headingVelocity[i] = atan2(vel[i]*meters_per_pixel,focal_length);
            }
            //cout << "heading["<<deviceName<<"] = (" << values[0] << ", " << values[1] << ")" << endl;        

        } else {            
            //cout << "No Blob Headings Found" << endl;        
        }

    } 

    return ok;
    
}

bool CB::ParticleFilterTrackerHeading::connectToParticleFilterTracker() {
    
    bool ok = true;

    if((int)inputPort.size() != numInputs) {
        return false;
    }    

    // connect to config port for reading config values
    string coordinatesOutputName;

    coordinatesOutputName = "/particleMod/target:o";
    
    cout << "ParticleFilterTrackerHeading::connectToParticleFilterTracker() -- connecting ports for image coordinate values..." << endl;

    cout << "zdf particle output port: " << coordinatesOutputName << endl;
    ok &= Network::connect(coordinatesOutputName.c_str(),inputPortName[0].c_str(),"tcp");
    if(!ok) {
        cout << "ParticleFilterTrackerHeading::connectToParticleFilterTracker() -- connecting to image coordinates (1) port failed..." << endl;
        return ok;
    }

    connectedToParticleFilterTracker = true;
    
    return ok;
}

yarp::sig::Vector CB::ParticleFilterTrackerHeading::getHeadingVelocity() {
    return headingVelocity;
}
