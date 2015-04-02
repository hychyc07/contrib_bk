// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#include "YARPBlobTrackerHeading.h"
#include <yarp/os/Network.h>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void CB::YARPBlobTrackerHeading::startResource() {
    start();     // mandatory start function
}

void CB::YARPBlobTrackerHeading::stopResource() {
    stop();     // mandatory stop function
}

bool CB::YARPBlobTrackerHeading::updateResource() {
  
    Bottle *b[2];
    bool ok = true;
    Vector sample(2);
    Vector vel(2);
    double focal_length = 0.0225;
    double meters_per_pixel = 0.0001;

    // if not connected to the configuration resource, do that now
    if(!connectedToBlobTracker) {
        if(!connectToBlobTracker()) {
            cout << "YARPBlobTrackerHeading couldn't connect to ports in startResource()..." << endl;
            return false;
        }
    }

    // read the current configuration variables from the input port
    b[0] = inputPort[0]->read(true);
    if(avgMode) {
        b[1] = inputPort[1]->read(true);
    }

    if(b[0]!=NULL) {

        if(avgMode) {
            if(b[1]!=NULL) {          
                valid = (bool)(b[0]->get(0).asInt());
                valid &= (bool)(b[1]->get(0).asInt());
            } 
        } else {
            valid = (bool)(b[0]->get(0).asInt());
        }

        if(valid) {
            Bottle *blob = b[0]->get(blobID+2).asList(); 
            imageCoordinates[0] = blob->get(0).asDouble();
            imageCoordinates[1] = blob->get(1).asDouble();
            vel[0] = blob->get(2).asDouble();
            vel[1] = blob->get(3).asDouble();

            
            if(avgMode) {
                Bottle *blob_1 = b[1]->get(blobID+2).asList(); 

                //cout << "got blobs (" << imageCoordinates[0] << "," << imageCoordinates[1] << ")";
                double tmp_0 = blob_1->get(0).asDouble();
                double tmp_1 = blob_1->get(1).asDouble();
                //cout << " + (" << tmp_0 << "," << tmp_1 << ")";

                imageCoordinates[0] += tmp_0;
                imageCoordinates[1] += tmp_1;
                imageCoordinates[0] /= 2.0;
                imageCoordinates[1] /= 2.0;

                //cout << " /2 = (" << imageCoordinates[0] << "," << imageCoordinates[1] << ")" << endl;

                tmp_0 = blob_1->get(2).asDouble();
                tmp_1 = blob_1->get(3).asDouble();
                //cout << " + (" << tmp_0 << "," << tmp_1 << ")";

                vel[0] += tmp_0;
                vel[1] += tmp_1;
                vel[0] /= 2.0;
                vel[1] /= 2.0;

            }

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

bool CB::YARPBlobTrackerHeading::connectToBlobTracker() {
    
    bool ok = true;

    if((int)inputPort.size() != numInputs) {
        return false;
    }    

    // connect to config port for reading config values
    string coordinatesOutputName[2];

    coordinatesOutputName[0] = deviceNames[0] + "/blobs:o";       
    
    cout << "YARPBlobTrackerHeading::connectToBlobTracker() -- connecting ports for image coordinate values..." << endl;

    cout << "blob port: " << coordinatesOutputName[0] << endl;
    ok &= Network::connect(coordinatesOutputName[0].c_str(),inputPortName[0].c_str(),"tcp");
    if(!ok) {
        cout << "YARPBlobTrackerHeading::connectToBlobTracker() -- connecting to image coordinates (1) port failed..." << endl;
        return ok;
    }

    if(avgMode) {
        coordinatesOutputName[1] = deviceNames[1] + "/blobs:o";
        cout << "blob port: " << coordinatesOutputName[1] << endl;
        ok &= Network::connect(coordinatesOutputName[1].c_str(),inputPortName[1].c_str(),"tcp");
        if(!ok) {
            cout << "YARPBlobTrackerHeading::connectToBlobTracker() -- connecting to image coordinates (2) port failed..." << endl;
            return ok;
        }
    }

    connectedToBlobTracker = true;
    
    return ok;
}

yarp::sig::Vector CB::YARPBlobTrackerHeading::getHeadingVelocity() {
    return headingVelocity;
}
