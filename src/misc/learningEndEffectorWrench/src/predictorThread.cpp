
#include "iCub/learningEndEffectorWrench/predictorThread.h"

/************************************************************************/
void predictorThread::setSharedArea(sharedArea* _pMem) {
    pMem = _pMem;
}

/************************************************************************/
void predictorThread::setOutputPort(BufferedPort<Vector> *_pfcPort) {
    pfcPort = _pfcPort;
}

/************************************************************************/
void predictorThread::init(ResourceFinder &rf) {
    pMachine.init(rf);

    while (pMem->getQ().size() == 0) {
        Time::delay(0.5);
        printf("Waiting for q...\n");
    }

    while (pMem->getF().size() == 0) {
        Time::delay(0.5);
        printf("Waiting for f...\n");
    }

    int period = rf.check("rate",20,"ms ratethread").asInt();
    this->setRate(period);
    this->start();
}

/************************************************************************/
void predictorThread::run() {
    Vector predictFerr = pMachine.predict(pMem->getQ());
    printf("predictFerr: %s\n",predictFerr.toString().c_str());
    Vector currentF = pMem->getF();
    Vector& correctF = pfcPort->prepare();
    correctF = currentF - predictFerr;
    pfcPort->write();
}

