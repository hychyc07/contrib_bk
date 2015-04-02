
#ifndef __LEARNINGENDEFFECTORWRENCH_H__
#define __LEARNINGENDEFFECTORWRENCH_H__

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>

#include "sharedArea.h"
#include "predictorThread.h"

#define DEFAULT_ROBOT "icub"
#define DEFAULT_PART "left_arm"

using namespace yarp::os;
using namespace yarp::sig;

class qSetPort : public BufferedPort<Vector> {
protected:
    sharedArea *pMem;
    virtual void onRead(Vector& v) {
//        printf("[Debug] Data arrived on qSetPort\n");
        pMem->setQ(v);
    }
public:
    void setSharedArea(sharedArea* _pMem) {
        pMem = _pMem;
    }
};

class fSetPort : public BufferedPort<Vector> {
protected:
    sharedArea *pMem;
    virtual void onRead(Vector& v) {
//        printf("[Debug] Data arrived on fSetPort\n");
        pMem->setF(v);
    }
public:
    void setSharedArea(sharedArea* _pMem) {
        pMem = _pMem;
    }
};

class learningEndEffectorWrench: public RFModule {
protected:    
	qSetPort qPort;  // to connect to robot arm, encoders
	fSetPort fPort;  // to connect to the FTsensor
    BufferedPort<Vector> fcPort;  // to output the corrected output
    sharedArea mem;
    predictorThread pThread;
	int period;

    double getPeriod();
    bool updateModule();
    bool interruptModule();

public:
    learningEndEffectorWrench();
    bool configure(ResourceFinder &rf);
};

#endif

