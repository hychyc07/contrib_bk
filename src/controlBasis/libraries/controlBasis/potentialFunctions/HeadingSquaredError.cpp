// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#include "HeadingSquaredError.h"
#include <yarp/math/Math.h>
#include <yarp/os/Network.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

void CB::HeadingSquaredError::startPotentialFunction() {
    start();     // mandatory start function
}

void CB::HeadingSquaredError::stopPotentialFunction() {
    stop();     // mandatory stop function
}

bool CB::HeadingSquaredError::updatePotentialFunction() {
    
    Bottle *b[2];
    double offset;
    bool ok = true;
    Vector diff(2);

    if(!connectedToInputs) {
        if(!connectToInputs()) {
            cout << "Couldn't connect to YARP input port in startPotentialFunction()..." << endl;
            return false;
        }
    }

    //cout << "HeadingPF update" << endl; 
    // get data from ports (should be more safety checks...)
    if(inputPorts.size() != 2) {
        cout << "HeadingSquaredError::update() -- wrong number of input ports!!" << endl;
        return false;
    }
    b[0] = inputPorts[0]->read(true);
    b[1] = inputPorts[1]->read(true);

    bool b0 = (b[0]==NULL);
    bool b1 = (b[1]==NULL);

    if( b0 || b1 ) {
        return ok;
    } 

    valid = (bool)(b[0]->get(1).asInt()) && (bool)(b[1]->get(1).asInt());
    if(!valid) return ok;
    
    offset = 2;

    if(inputs[0]->size() != 2) inputs[0]->resize(2);
    if(inputs[1]->size() != 2) inputs[1]->resize(2);

    for(int i=0; i<size; i++) {
        (*inputs[0])[i] = b[0]->get(i+offset).asDouble();
        (*inputs[1])[i] = b[1]->get(i+offset).asDouble();
        diff[i] = (*inputs[1])[i] - (*inputs[0])[i];
    }

    // compute potential and gradient
    gradient = -1.0*diff;
    potential = 0.5*dot(diff,diff);
    
    cout << "dt = " << t1 << " - " << t0 << " = " << dt << endl;
    cout << "ref  -  cur  =  diff" << endl;
    for(int i=0; i<size; i++) {
        cout << (*inputs[1])[i] << "    " << (*inputs[0])[i] << "   " << diff[i] << endl;
    }
    cout << endl;

    //cout << "HeadingSquaredError Potential = " << potential << endl;
    return ok;

}

bool CB::HeadingSquaredError::connectToInputs() {
    
    bool ok = true;

    int randomID = (int)(yarp::os::Random::uniform()*1000.0);
    char *c = (char *)malloc(16);
    sprintf(c,"%d", randomID);
    std::string randomIDstr(c);

    if(inputNames.size() != 2) {
        cout << "HeadingSquaredError::connectToInputs() -- size mismatch!!" << endl;
        return false;
    }

    cout << "HeadingSquaredError::connectToInputs():\n\t" <<
        inputNames[0].c_str() << "\n\t" << inputNames[1].c_str() << endl << endl;

    string posCurName = inputNames[0] + "/data:o";
    string posRefName = inputNames[1] + "/data:o";

    string prefixStr = "/cb/" + getSpace();
    int s = prefixStr.size();
    string tmp0 = inputNames[0];
    string tmp1 = inputNames[1];
    tmp0.erase(0,s);
    tmp1.erase(0,s);

    string posCurNameIn = "/cb/heading/squared_error_pf" + tmp0 + "/" + randomIDstr + "/cur/data:i";
    string posRefNameIn = "/cb/heading/squared_error_pf" + tmp1 + "/" + randomIDstr + "/ref/data:i";

    cout << "HeadingSquaredError::connectToInputs() -- opening current input port..." << endl;
    ok &= inputPorts[0]->open(posCurNameIn.c_str());
    if(!ok) {
        cout << "HeadingSquaredError::connectToInputs() -- failed opening current input port..." << endl;
        return ok;
    }

    cout << "HeadingSquaredError::connectToInputs() -- opening reference input port..." << endl;
    ok &= inputPorts[1]->open(posRefNameIn.c_str());
    if(!ok) {
        cout << "ConfigurationSquaredError::connectToInputs() -- failed opening reference input port..." << endl;
        return ok;
    }

    cout << "HeadingSquaredError::connectToInputs() -- connecting:\n\t" << 
        posCurName.c_str() << " -> " << posCurNameIn.c_str() << "\n\t" << 
        posRefName.c_str() << " -> " << posRefNameIn.c_str() << endl << endl;

    ok &= Network::connect(posCurName.c_str(),posCurNameIn.c_str(),"tcp");
    if(!ok) {
        cout << "HeadingSquaredError::connectToInputs() -- failed connecting to current input pors...\n\n\n" << endl;
        return ok;
    }
    ok &= Network::connect(posRefName.c_str(),posRefNameIn.c_str(),"tcp");
    if(!ok) {
        cout << "HeadingSquaredError::connectToInputs() -- failed connecting to reference input pors...\n\n\n" << endl;
        return ok;
    }
    
    cout << "HeadingSquaredError done connecting to YARP input ports..." << endl;

    connectedToInputs = true;
    return ok; 
}


