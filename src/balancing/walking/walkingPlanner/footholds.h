#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <iostream>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;

/* This is a class to compute robot footholds*/

class footholds : public RateThread
{
private:

public:
	footholds(yarp::os::ResourceFinder &rf);
	~footholds(void) {};
};




