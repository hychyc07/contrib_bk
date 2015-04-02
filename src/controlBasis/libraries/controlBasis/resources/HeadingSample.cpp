// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#include "HeadingSample.h"

#include <yarp/os/Network.h>
#include <math.h>
#include <string.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void CB::HeadingSample::startResource() {
    start();     // mandatory start function
}

void CB::HeadingSample::stopResource() {
    stop();     // mandatory stop function
}

bool CB::HeadingSample::updateResource() {
  // do something? nope...
    return true;
}

bool CB::HeadingSample::drawSample() {
    bool ok = false;
    if(dist!=NULL) {      
        ok = (bool)(dist->drawSample(values));     
    } 
    if(!ok) values.zero();
    return ok;
}

bool CB::HeadingSample::loadDistributionFromFile(string f) {
    bool ok = false;
    if(dist!=NULL) {      
        ok = dist->loadDistribution(f);     
    } 
    return ok;
}

CB::HeadingSample::distributionType CB::HeadingSample::getDistributionTypeFromFile(string fName) {

    FILE *fid;
    int dims;

    if( !(fid = fopen(fName.c_str(),"r")) ) {
        cout << "HeadingSample::getDistributionTypeFromFile() -- can't open file=\'" << fName.c_str() << "\'!!" << endl;
        return (distributionType)NULL;
    }
    
    char *type;

    type = (char *)malloc(32);
    fscanf(fid,"%s %d\n", type, &dims);

    if(!strcmp(type,"Gaussian")) {
        distType = GAUSSIAN;
    } else if(!strcmp(type,"NonParameteric")) {
        distType = NONPARAMETRIC;
    } else {
        return (distributionType)NULL;
    }   

    if(dims!=2) {
        cout << "HeadingSample::getDistributionTypeFromFile() -- incorrect distribution size: " << dims << "!!" << endl;
        return (distributionType)NULL;
    }

    return distType;
}
