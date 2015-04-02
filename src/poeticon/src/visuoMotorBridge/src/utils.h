

#ifndef __VISUO_MOTOR_UTILS__
#define __VISUO_MOTOR_UTILS__

#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>


#define LEFT                        0
#define RIGHT                       1
#define ARM_IN_USE                  -1
#define ARM_MOST_SUITED             -2


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


class Resource
{
private:
    Semaphore       mutex_stereo;

    Vector          stereo;

public:
    Resource()
    {}

    void setStereo(const Vector &_stereo)
    {
        mutex_stereo.wait();
        stereo=_stereo;
        mutex_stereo.post();
    }

    Vector getStereo()
    {
        Vector s;
        mutex_stereo.wait();
        s=stereo;
        stereo.clear();
        mutex_stereo.post();
        return s;
    }
};




#endif
