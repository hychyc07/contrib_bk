
#ifndef __PORTEDMACHINE_H__
#define __PORTEDMACHINE_H__

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>

#include <iCub/learningMachine/MachinePortable.h>
#include <iCub/learningMachine/TransformerPortable.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::learningmachine;

class portedMachine {
protected:
    MachinePortable mp;
    TransformerPortable tp;
public:
    portedMachine() { }
    bool init(ResourceFinder &rf);
    Vector predict(const Vector v);
};

#endif

