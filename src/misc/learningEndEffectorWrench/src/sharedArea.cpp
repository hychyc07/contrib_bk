
#include "iCub/learningEndEffectorWrench/sharedArea.h"

/************************************************************************/
void sharedArea::setF(const Vector& _f) {
    fMutex.wait();
    f = _f;
    fMutex.post();
}

/************************************************************************/
void sharedArea::setQ(const Vector& _q) {
    qMutex.wait();
    q = _q;
    qMutex.post();
}

/************************************************************************/
Vector sharedArea::getQ() {
    qMutex.wait();
    Vector _q=q;
    qMutex.post();
    return _q;
}

/************************************************************************/
Vector sharedArea::getF() {
    fMutex.wait();
    Vector _f=f;
    fMutex.post();
    return _f;
}

