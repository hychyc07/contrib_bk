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

#define PRINT_STATUS_PER_AE     12.0    // [s]
#define MAX_TORSO_PITCH         35.0    // [deg]
#define MIN_TORSO_PITCH         -15.0   // [deg]
#define DELTAT                  4.0     // [s]
#define STOP_PER                2.0     // [s]  Time to stop if there's a contact.
#define F_AVG                   1.36
#define MU_AVG                  0.118

YARP_DECLARE_DEVICES(icubmod)

//******************************************************************************
//******************************************************************************

class armExploration: public RateThread {
protected:
    PolyDriver        *client;
    ICartesianControl *arm;

    //Desired positions and orientations
    Vector xd;
    Vector od;
    
    Vector odl;
    Vector odr;
    //Actual positions and orientations
    Vector x;
    Vector o;
    //Original positions and orientations
    Vector x0;
    Vector o0;
    //Computed Force at the end-effector
    Vector *FT;
    Vector dFT;
    
    Vector FT_f;
    Vector FT_mu;
    Vector dFT_f;
    Vector dFT_mu;

    double t;
    double t0;
    double t1;
    double t2;
    double alt;
    
    bool startTraj;
    bool done;
    
    //flag used by movements along z-axis
    int fz,fz_orig;
    
    //flag used by movements along yx-axis
    int f_cnt;
    
    string part;
    bool v;
    
    ofstream outfile;
    
    AWLinEstimator  *FTlinEst;
    
    Vector evalVel(const Vector &x)
    {
        AWPolyElement el;
        el.data=x;
        el.time=Time::now();

        return FTlinEst->estimate(el);
    }

public:
    armExploration(int _rate, PolyDriver *_dd, 
                    const string &_part, const string &_name, Vector &_FT, bool _v);
    virtual bool threadInit();
    
    virtual void run();
    virtual void threadRelease();
    void generateTarget();

    double norm(const Vector &v) { return sqrt(dot(v,v)); }
    
    bool Need_to_Stop();

    void limitTorsoPitch();
    
    void printStatus();
};
