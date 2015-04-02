
#include "iCub/WBCdemo/sharedArea.h"

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

