// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#include "iCubTriangulatedCartesianPosition.h"

#include <yarp/os/Network.h>
#include <yarp/math/Math.h>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

void CB::iCubTriangulatedCartesianPosition::startResource() {
    start();     // mandatory start function
}

void CB::iCubTriangulatedCartesianPosition::stopResource() {
    stop();     // mandatory stop function
}

bool CB::iCubTriangulatedCartesianPosition::updateResource() {
  
    Bottle *b_left;
    Bottle *b_right;
    bool ok = true;

    double focal_length = 0.0225;
    double meters_per_pixel = 0.0001;
    double img_width = 320;
    double img_height = 240;

    double imageCoordinatesLeft[2];
    double imageCoordinatesRight[2];

    // if not connected to the configuration resource, do that now
    if(!connectedToHeadings) {
        if(!connectToHeadings()) {
            cout << "iCub triangulation cartpos couldn't connect to heading ports in startResource()..." << endl;
            return false;
        }
    }

    // read the current configuration variables from the input port
    b_left = inputPort[0]->read(false);
    b_right = inputPort[1]->read(false);
    if( (b_left!=NULL) && (b_right!=NULL) ) {

        bool bl = (bool)(b_left->get(1).asInt());
        bool br = (bool)(b_right->get(1).asInt());
 
        if( !bl || !br ) {
            valid = false;
            return ok;
        } else {
            valid = true;
        }    

        // read headings
        for(int i=0; i<2; i++) {
            leftHeading[i] = b_left->get(i+2).asDouble();
            rightHeading[i] = b_right->get(i+2).asDouble();
        }

        imageCoordinatesLeft[0] = tan(leftHeading[0])*focal_length/meters_per_pixel + img_width/2.0;
        imageCoordinatesLeft[1] = tan(leftHeading[1])*focal_length/meters_per_pixel + img_height/2.0;
        imageCoordinatesRight[0] = tan(rightHeading[0])*focal_length/meters_per_pixel + img_width/2.0;
        imageCoordinatesRight[1] = tan(rightHeading[1])*focal_length/meters_per_pixel + img_height/2.0;

        /*
        cout << "left heading bottle: " << b_left->toString().c_str() << endl;
        cout << "right heading bottle: " << b_right->toString().c_str() << endl;
        */

        //cout << "heading L: (" << leftHeading[0] << "," << leftHeading[1] << "), uv: (" << (int)imageCoordinatesLeft[0] << "," << (int)imageCoordinatesLeft[1] << ")" << endl;
        //cout << "heading R: (" << rightHeading[0] << "," << rightHeading[1] << "), uv: (" << (int)imageCoordinatesRight[0] << "," << (int)imageCoordinatesRight[1] << ")" << endl;


            //send headings to eyeTriangulation module
            Bottle &b_heading_out = eyeTriangulationInputPort.prepare();
            b_heading_out.clear();
            b_heading_out.addDouble(imageCoordinatesLeft[0]);
            b_heading_out.addDouble(imageCoordinatesLeft[1]);
            b_heading_out.addDouble(imageCoordinatesRight[0]);
            b_heading_out.addDouble(imageCoordinatesRight[1]);
            eyeTriangulationInputPort.write();
            
            // read the triangulated position from the eyeTriangulation module
            Bottle *b_position_in = eyeTriangulationOutputPort.read(true);
            if(b_position_in!=NULL) {
                values[0] = b_position_in->get(0).asDouble();
                values[1] = b_position_in->get(1).asDouble();
                values[2] = b_position_in->get(2).asDouble();
            } else {
                values.zero();
            }

        cout << "Triangulated Pos=[" << values[0] << " " << values[1] << " " << values[2] << "]" << endl;
    } 

    return ok;
    
}

bool CB::iCubTriangulatedCartesianPosition::connectToHeadings() {
    
    bool ok = true;

    if((int)inputPort.size() != numInputs) {
        return false;
    }    

    if(!useLearnedModel) {
        // connect ports to eyeTriangulation module.  This module accepts iCub headings, and evaluates the resulting triangulated Cartesian Position
        string headingOutputName = "/cb/cartesianposition" + deviceName + "/eyeTriangulation/headings:o";
        string headingInputName = "/eyeTriangulation/x:i";
        
        string positionOutputName = "/eyeTriangulation/X:o";
        string positionInputName = "/cb/cartesianposition" + deviceName + "/eyeTriangulation/position:i";

        
        cout << "iCubTriangulatedCartesianPosition::connect() -- opening port to send headings to triangulation service" << endl;
        eyeTriangulationInputPort.open(headingOutputName.c_str());

        cout << "iCubTriangulatedCartesianPosition::connect() -- connecting heading ports..." << endl;
        ok &= Network::connect(headingOutputName.c_str(),headingInputName.c_str(), "tcp");
        if(!ok) {
            cout << "iCubTriangulatedCartesianPosition::connect() -- connecting heading port falied..." << endl;
            return ok;
        }

        cout << "iCubTriangulatedCartesianPosition::connect() -- opening port to get position from triangulation service" << endl;
        eyeTriangulationOutputPort.open(positionInputName.c_str());

        cout << "iCubTriangulatedCartesianPosition::connect() -- connecting position ports..." << endl;
        ok &= Network::connect(positionOutputName.c_str(),positionInputName.c_str(), "tcp");
        if(!ok) {
            cout << "iCubTriangulatedCartesianPosition::connect() -- connecting position port falied..." << endl;
            return ok;
        }
    }
    
    // connect input headings.  These headings are returned by Control Basis Heading modules.
    string headingLeftOutputName = "/cb/heading/" + leftHeadingName + "/data:o";
    string headingRightOutputName = "/cb/heading/" + rightHeadingName + "/data:o";

    cout << "trying to conenct to " << headingLeftOutputName.c_str() << endl;
    //    cout << "iCubTriangulatedCartesianPosition::connectToConfiguration() -- connecting left heading port..." << endl;
    ok &= Network::connect(headingLeftOutputName.c_str(),inputPortName[0].c_str(),"tcp");
    if(!ok) {
        cout << "iCubTriangulatedCartesianPosition::connectToConfiguration() -- connecting to left heading port falied..." << endl;
        return ok;
    }

    cout << "trying to conenct to " << headingRightOutputName.c_str() << endl;
    cout << "iCubTriangulatedCartesianPosition::connectToConfiguration() -- connecting right heading port..." << endl;
    ok &= Network::connect(headingRightOutputName.c_str(),inputPortName[1].c_str(),"tcp");
    if(!ok) {
        cout << "iCubTriangulatedCartesianPosition::connectToConfiguration() -- connecting to right heading port falied..." << endl;
        return ok;
    }

    connectedToHeadings = true;

    return ok;
}


