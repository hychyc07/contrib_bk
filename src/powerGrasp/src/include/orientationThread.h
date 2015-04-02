#include <cmath>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <iCub/iKin/iKinFwd.h>

class OrientationThread : public yarp::os::RateThread
{
private:

    int context_in;
    int current_context;
    int context;
    int nAngles;
    double manipulability;
    double result;
    double limits;
    double man;
    bool done;
    bool work;
    std::string hand;

    yarp::dev::PolyDriver dCtrl;
    yarp::dev::ICartesianControl *iCtrl;
    yarp::dev::PolyDriver robotArm;
    yarp::dev::PolyDriver robotTorso;
    yarp::dev::IControlLimits *limTorso, *limArm;

    iCub::iKin::iCubArm *arm;
    iCub::iKin::iKinChain *chain;

    yarp::sig::Vector thetaMin;
    yarp::sig::Vector thetaMax;
    yarp::sig::Vector eePos;
    yarp::sig::Vector px;
    yarp::sig::Vector py;
    yarp::sig::Vector pointNormal;
    yarp::sig::Vector center;
    yarp::sig::Vector biggerAxis;
    yarp::sig::Matrix designedOrientation;
    yarp::sig::Vector od;
    yarp::sig::Vector q, q0;
    yarp::sig::Vector xdhat, odhat;
    yarp::sig::Vector ones;
    yarp::sig::Matrix Jacobian, mulJac;
    yarp::sig::Vector angles;
    yarp::sig::Matrix orientation;

    void getSampledAngles(yarp::sig::Vector &angles, int factor);
    bool normalDirection(std::string &hand, yarp::sig::Vector &normal);

public:

    OrientationThread();
    ~OrientationThread() {};

    bool open(std::string &arm, std::string &robot, int &nAngles);
    bool checkDone();
    void setInfo(yarp::sig::Vector &eePos, yarp::sig::Vector &px, yarp::sig::Vector &py, yarp::sig::Vector &pointNormal, yarp::sig::Vector &center, yarp::sig::Vector &biggerAxis);
    void getBestManipulability(double &manipulability, yarp::sig::Matrix &orientation);
    void storeContext();
    void restoreContext();
    void threadRelease();
    void close();
    void run(); 
 
};
