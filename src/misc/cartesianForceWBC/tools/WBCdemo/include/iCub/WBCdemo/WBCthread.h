
#ifndef __WBCTHREAD_H__
#define __WBCTHREAD_H__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <jspace/test/sai_util.hpp>
#include <jspace/Model.hpp>

#include <opspace/task_library.hpp>
#include <opspace/skill_library.hpp>
#include <opspace/ClassicTaskPostureController.hpp>

#include <boost/shared_ptr.hpp>
#include <err.h>

#include "sharedArea.h"


#define SEND_COMMANDS


#define THREAD_RATE_INIT 20 // IRRELEVANT: Override done by RF
#define N_TOTAL 7
#define N_FORCE 5

using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::ctrl;

class WBCthread : public RateThread {
protected:
    yarp::dev::IPositionControl *pos;
    yarp::dev::IControlMode *ictrl;
    yarp::dev::ITorqueControl *itrq;
    yarp::dev::ICartesianControl *icart;
    sharedArea *pMem;

    AWLinEstimator *linEst;

    boost::shared_ptr<jspace::Model> model;
    boost::shared_ptr<opspace::ClassicTaskPostureController> controller;
    boost::shared_ptr<opspace::GenericSkill> skill;
    boost::shared_ptr<opspace::CartPosTask> eetask;
    boost::shared_ptr<opspace::JPosTask> jtask;

    opspace::Parameter * eegoalpos;
    opspace::Parameter * eegoalvel;
    opspace::Parameter * jgoalpos;
    opspace::Parameter * jgoalvel;

    jspace::State state;
    jspace::Vector command;

    yarp::sig::Vector sig_q;
    yarp::sig::Vector sig_dq;

    yarp::sig::Vector threshold;

    yarp::sig::Vector evalVel(const yarp::sig::Vector &x);

    int mode;
    size_t prevmode;
    size_t iteration;

    bool verbose;
    
public:
    WBCthread() : RateThread(THREAD_RATE_INIT) {}  // In ms

    void setSharedArea(sharedArea* _pMem);
    void setIPositionControl(yarp::dev::IPositionControl *_pos);
    void setIControlMode(yarp::dev::IControlMode *_ictrl);
    void setITorqueControl(yarp::dev::ITorqueControl *_itrq);
    void setMode(const int _mode);
    int getMode();
    void setVerbose(const bool _mode);
    bool getVerbose();
    void init(ResourceFinder &rf);
    void run();  // The periodical function
    void threadRelease();
    void stop();  //just in case, xD
};

#endif

