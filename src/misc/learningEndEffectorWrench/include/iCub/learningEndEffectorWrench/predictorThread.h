
#ifndef __PREDICTORTHREAD_H__
#define __PREDICTORTHREAD_H__

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include "sharedArea.h"
#include "portedMachine.h"

#define THREAD_RATE_INIT 20  // In ms, unmeaningful as ALWAYS gets overwritten by RF

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

class predictorThread : public RateThread {
private:
    portedMachine pMachine;
    BufferedPort<Vector> *pfcPort;  // to output the corrected output
    sharedArea *pMem;

public:
    predictorThread() : RateThread(THREAD_RATE_INIT) {}  // In ms

    void setSharedArea(sharedArea* _pMem);
    void setOutputPort(BufferedPort<Vector> *_pfcPort);
    void init(ResourceFinder &rf);
    void run();  // The periodical function
};

#endif

