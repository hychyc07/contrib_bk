// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#include "HeadingJacobian.h"

using namespace std;
using namespace yarp::sig;

void CB::HeadingJacobian::startJacobian() {
    start();     // mandatory start function
}

void CB::HeadingJacobian::stopJacobian() {
    stop();     // mandatory stop function
}

bool CB::HeadingJacobian::updateJacobian() {

    // if not conencted to inputs, do that now
    if(!connectedToInputs) {
        if(!connectToInputs()) {
            cout << "HeadingJacobian couldn't connect to input ports in startJacobian()..." << endl;
            return false;
        }
    }  

    J[0][0] = -1.0;
    J[1][1] = 1.0;
    J[2][0] = 0.0;//1.0; verge effect on u coordinate
    J[2][1] = 0.0;
    valid = true;

    return true; 
}

bool CB::HeadingJacobian::connectToInputs() {
     connectedToInputs = true;   
     return true;
}
