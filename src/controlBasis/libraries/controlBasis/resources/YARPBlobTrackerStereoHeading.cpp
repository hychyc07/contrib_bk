// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#include "YARPBlobTrackerStereoHeading.h"
#include <yarp/os/Network.h>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void CB::YARPBlobTrackerStereoHeading::startResource() {
    start();     // mandatory start function
}

void CB::YARPBlobTrackerStereoHeading::stopResource() {
    stop();     // mandatory stop function
    estimate.zero();
}

bool CB::YARPBlobTrackerStereoHeading::updateResource() {
  
    Bottle *b[2];
    bool ok = true;
    Vector sample(4);
    double focal_length = 0.0225;
    //    double meters_per_pixel = 0.0001;
    
    // if not connected to the configuration resource, do that now
    if(!connectedToBlobTrackers) {
        if(!connectToBlobTrackers()) {
            cout << "YARP Atten. Mechanism couldn't connect to ports in startResource()..." << endl;
            return false;
        }
    }


    bool v[2];
    // read the current configuration variables from the input ports
    for(int i=0; i<2; i++) {            
        b[i] = inputPort[i]->read(false);
        if(b[i]!=NULL) {       
            
            v[i] = (bool)(b[i]->get(0).asInt());
            
            if(v[i]) {
                Bottle *blob = b[i]->get(2).asList(); 
                for(int k=0; k<2; k++) {            
                    imageCoordinates[i][k] = blob->get(k).asDouble();
                }
            }
        }
    }
        
    if(v[0] && v[1]) {
        // turn uv coordinates into a filtered heading
        for(int i=0; i<4; i++) {            
            sample[i] = atan2(imageCoordinates[(int)(i/2)][i%2],focal_length);
            estimate[i] = estimate[i] + alpha*sample[i];
            values[i] = estimate[i];
        }
        cout << "StereoHeading: LEFT=[" << values[0] << " " << values[1] << "]" << ", RIGHT=[" << values[2] << " " << values[3] << "]" << endl;
        valid = true;
    } else {
        valid = false;
    }

    //cout << "Blob Heading dt = " << t1 << " - " << t0 << " = " << dt << endl;
    return ok;
    
}

bool CB::YARPBlobTrackerStereoHeading::connectToBlobTrackers() {
    
    bool ok = true;

    if((int)inputPort.size() != numInputs) {
        return false;
    }    

    // connect to config port for reading config value
    string leftCoordinatesOutputName  = "/blobFinder/"+deviceName+"/left_cam/triangulation:o";
    string rightCoordinatesOutputName = "/blobFinder/"+deviceName+"/right_cam/triangulation:o";

    cout << "YARPBlobTrackerStereoHeading::connectToBlobTrackers() -- connecting ports for left image coordinates..." << endl;
    ok &= Network::connect(leftCoordinatesOutputName.c_str(),inputPortName[0].c_str(),"tcp");
    if(!ok) {
        cout << "YARPBlobTrackerStereoHeading::connectToBlobTrackers() -- connecting to left image coordinates port failed..." << endl;
        return ok;
    }

    cout << "YARPBlobTrackerStereoHeading::connectToBlobTrackers() -- connecting ports for right image coordinates..." << endl;
    ok &= Network::connect(rightCoordinatesOutputName.c_str(),inputPortName[1].c_str(),"tcp");
    if(!ok) {
        cout << "YARPBlobTrackerStereoHeading::connectToBlobTrackers() -- connecting to right image coordinates port failed..." << endl;
        return ok;
    }

    connectedToBlobTrackers = true;
   
    return ok;
}
