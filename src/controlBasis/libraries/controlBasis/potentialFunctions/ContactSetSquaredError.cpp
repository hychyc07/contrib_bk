// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#include "ContactSetSquaredError.h"
#include <yarp/math/Math.h>
#include <yarp/os/Network.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

void CB::ContactSetSquaredError::startPotentialFunction() {
    start();     // mandatory start function
}

void CB::ContactSetSquaredError::stopPotentialFunction() {
    stop();     // mandatory stop function
}

bool CB::ContactSetSquaredError::updatePotentialFunction() {
       
    Bottle *b[2];
    double offset;
    bool ok = true;
    Vector diff;

    if(!connectedToInputs) {
        if(!connectToInputs()) {
            cout << "Couldn't connect to YARP input port in startPotentialFunction()..." << endl;
            return false;
        }
    }

    // get data from ports (should be more safety checks...)
    if(inputPorts.size() != 2) {
        cout << "ContactSetSquaredError::update() -- wrong number of input ports!!" << endl;
        return false;
    }
       
    b[0] = inputPorts[0]->read(false);
    b[1] = inputPorts[1]->read(false);
    
    if( (b[0]==NULL) || (b[1]==NULL) ) {
        // non fatal error
        //cout << "ContactSetSquaredError::update() problem reading data!!" << endl;
        //        if(size==0) {
        //    diff.resize(1);
        //    diff.zero();
        //}
        return ok;
    }

    valid = (bool)(b[0]->get(1).asInt()) && (bool)(b[1]->get(1).asInt());

    if(valid) {

        offset = 3;
        int s0 = b[0]->get(2).asInt();
        int s1 = b[1]->get(2).asInt();

        cout << "sizes: (" << s0 << "," << s1 << ")" << endl;        
 
        if(size!=s0) {
            size = s0;
            inputs[0]->resize(size);
            inputs[1]->resize(size);
            gradient.resize(size);
            cout << "ContactSetSquaredError setting size: " << size << endl;
        }

        diff.resize(size);
        for(int i=0; i<size; i++) {
            (*inputs[0])[i] = b[0]->get(i+offset).asDouble();
            (*inputs[1])[i] = b[1]->get(i+offset).asDouble();
            diff[i] = (*inputs[1])[i] - (*inputs[0])[i];
        }
        
        // compute the potential and its gradient
        gradient = -1.0*diff;
        potential = 0.5*dot(diff,diff);
        
        cout << "ref   -  cur  =  diff" << endl;
        for(int i=0; i<size; i++) {
            cout << (*inputs[1])[i] << "  " << (*inputs[0])[i] << "  " << diff[i] << endl;
        }
        cout << endl;
        
        //    cout << "ContactSetSquaredError Potential = " << potential << endl;

    }
    return ok;

}

bool CB::ContactSetSquaredError::connectToInputs() {
    
    bool ok = true;

    int randomID = (int)(yarp::os::Random::uniform()*1000.0);
    char *c = (char *)malloc(16);
    sprintf(c,"%d", randomID);
    std::string randomIDstr(c);

    if(inputNames.size() != 2) {
        cout << "ContactSetSquaredError::connectToInputs() -- size mismatch!!" << endl;
        return false;
    }
    cout << "ContactSetSquaredError::connectToInputs():\n\t" << inputNames[0].c_str() << " \n\t" << inputNames[1].c_str() << "\n\n"; 
  
    string configCurName = inputNames[0] + "/data:o";
    string configRefName = inputNames[1] + "/data:o";

    string prefixStr = "/cb/" + getSpace();
    int s = prefixStr.size();
    string tmp0 = inputNames[0];
    string tmp1 = inputNames[1];
    tmp0.erase(0,s);
    tmp1.erase(0,s);
    

    string configCurNameIn = "/cb/contactset/squared_error_pf" + tmp0 + "/" + randomIDstr + "/cur/data:i";
    string configRefNameIn = "/cb/contactset/squared_error_pf" + tmp1 + "/" + randomIDstr + "/ref/data:i";

    cout << "ContactSetSquaredError::connectToInputs() -- opening current input port..." << endl;
    ok &= inputPorts[0]->open(configCurNameIn.c_str());
    if(!ok) {
        cout << "ContactSetSquaredError::connectToInputs() -- failed opening current input port..." << endl;
        return ok;
    }
    cout << "ContactSetSquaredError::connectToInputs() -- opening reference input port..." << endl;
    ok &= inputPorts[1]->open(configRefNameIn.c_str());
    if(!ok) {
        cout << "ContactSetSquaredError::connectToInputs() -- failed opening reference input port..." << endl;
        return ok;
    }

    //    Time::delay(0.1);
    cout << "ContactSetSquaredError::connectToInputs() -- connecting:\n\t" << 
        configCurName.c_str() << " -> " << configCurNameIn.c_str() << "\n\t" << 
        configRefName.c_str() << " -> " << configRefNameIn.c_str() << endl << endl;
    
    ok &= Network::connect(configCurName.c_str(),configCurNameIn.c_str(), "tcp");
    //Time::delay(0.1);
    ok &= Network::connect(configRefName.c_str(),configRefNameIn.c_str(), "tcp");
    if(!ok) {
        cout << "ContactSetSquaredError::connectToInputs() -- failed connecting to input ports..." << endl << endl;
        return ok;
    }
    cout << "ConfiguartionSquaredError done connecting to YARP input ports..." << endl;
    connectedToInputs = true;
    return ok; 
}


