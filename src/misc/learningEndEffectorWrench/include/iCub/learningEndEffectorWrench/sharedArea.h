
#ifndef __SHAREDAREA_H__
#define __SHAREDAREA_H__

#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

class sharedArea {
private:
    Vector q;
    Vector f;
    Semaphore qMutex;
    Semaphore fMutex;
public:
    void setF(const Vector& v);
    void setQ(const Vector& v);
    Vector getF();
    Vector getQ();
};

#endif

