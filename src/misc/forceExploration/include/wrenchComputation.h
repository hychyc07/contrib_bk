#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynTransform.h>

#include <iostream>
#include <iomanip>
#include <string.h>
#include <fstream>

#include <gsl/gsl_math.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;

using namespace std;

#define PRINT_STATUS_PER_WR        12.0    // [s]

YARP_DECLARE_DEVICES(icubmod)

//******************************************************************************
//******************************************************************************
class wrenchComputation: public RateThread
{
private:
    PolyDriver *dd;
    PolyDriver *tt;
    IEncoders  *iencs;
    IEncoders  *tencs;

    Vector encoders;
    Vector encodersT;
    
    string part;

    Vector *ft;
    BufferedPort<Vector> *port_FT;
    
    bool first;

    AWLinEstimator  *linEst;
    AWQuadEstimator *quadEst;

    int ctrlJnt;
    
    iDynLimb      *limb;
    iDynChain     *chain;
    iDynInvSensor *sens;

    //iFTransformation         *FTtoBase;
    iGenericFrame *sensor;
    int sensorLink;
    
    //ofstream outfile;

    Vector q;
    Vector dq;
    Vector d2q;

    Vector w0,dw0,d2p0,Fend,Mend;
    Vector F_measured, F_iDyn, F_offset, FT, *FT_ref;
    
    int ft_flag;

    double t,t0,t1;

    Vector evalVel(const Vector &x)
    {
        AWPolyElement el;
        el.data=x;
        el.time=Time::now();

        return linEst->estimate(el);
    }

    Vector evalAcc(const Vector &x)
    {
        AWPolyElement el;
        el.data=x;
        el.time=Time::now();

        return quadEst->estimate(el);
    }

public:
    wrenchComputation(int _rate, PolyDriver *_dd, PolyDriver *_tt,
                    const string &_part, const string &_name, Vector &_FT);

    bool threadInit();

    void run();

    void threadRelease();
    
    void printStatus();
};
