
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


#define FILE_LOGGER

#ifdef FILE_LOGGER
    #include <fstream>
#endif

#define THREAD_RATE_INIT 20 // IRRELEVANT: Override done by RF
#define N_TOTAL 7
#define N_FORCE 5

using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::ctrl;


namespace tut03 {

class JTask : public opspace::Task {
public:
    JTask() : opspace::Task("tut03::JTask"), mode_(0) {}

    virtual jspace::Status init(jspace::Model const & model) {
        //////////////////////////////////////////////////
        // The Jacobian is not really required for pure jspace control,
        // but will become very important for operational-space control.

        jacobian_ = jspace::Matrix::Identity(model.getNDOF(), model.getNDOF());

        //////////////////////////////////////////////////
        // Initialize our PD parameters.

        kp_ = jspace::Vector::Zero(model.getNDOF());
        kd_ = jspace::Vector::Zero(model.getNDOF());

        kp_[0] = 0.2 * 180.0 / M_PI;
        kp_[1] = 0.1 * 180.0 / M_PI;
        kp_[2] = 0.1 * 180.0 / M_PI;
        kp_[3] = 0.05 * 180.0 / M_PI;
        kp_[4] = 0.02 * 180.0 / M_PI;

        kd_[0] = 0.01;
        kd_[1] = 0.01;
        kd_[2] = 0.01;
        kd_[3] = 0.01;
        kd_[4] = 0.01;

        kp_M_ = jspace::Vector::Zero(model.getNDOF());
        kd_M_ = jspace::Vector::Zero(model.getNDOF());

        // No NOT ever go above 2.0 in the following
        kp_M_[0] = 2.0 * 180.0 / M_PI;
        kp_M_[1] = 2.0 * 180.0 / M_PI;
        kp_M_[2] = 2.0 * 180.0 / M_PI;
        kp_M_[3] = 2.0 * 180.0 / M_PI;
        kp_M_[4] = 2.0 * 180.0 / M_PI;

        // Do NOT move the following values from 0.01
        kd_M_[0] = 0.01;
        kd_M_[1] = 0.01;
        kd_M_[2] = 0.01;
        kd_M_[3] = 0.01;
        kd_M_[4] = 0.01;

        //////////////////////////////////////////////////
        // Initialize our goal to the current configuration.

        goal1_ = model.getState().position_;
        goal1_[0] = -28.0 * M_PI / 180.0;
        goal1_[1] = 80.0 * M_PI / 180.0;
        goal1_[2] = 15.0 * M_PI / 180.0;
        goal1_[3] = 50.0 * M_PI / 180.0;
        goal1_[4] = 0.0 * M_PI / 180.0;
        goal1_[5] = 0.0 * M_PI / 180.0;
        goal1_[6] = 0.0 * M_PI / 180.0;

        goal2_ = model.getState().position_;
        goal2_[0] = -32.0 * M_PI / 180.0;
        goal2_[1] = 20.0 * M_PI / 180.0;
        goal2_[2] = 25.0 * M_PI / 180.0;
        goal2_[3] = 45.0 * M_PI / 180.0;
        goal2_[4] = 0.0 * M_PI / 180.0;
        goal2_[5] = 0.0 * M_PI / 180.0;
        goal2_[6] = 0.0 * M_PI / 180.0;

        //////////////////////////////////////////////////
        // No initialization problems to report: the default constructor
        // of jspace::Status yields an instance that signifies success.

        jspace::Status ok;
        return ok;
    }


    virtual jspace::Status update(jspace::Model const & model) {

        //////////////////////////////////////////////////
        // Update the state of our task. Again, this is not critical
        // here, but important later when we want to integrate several
        // operational space tasks into a hierarchy.

        actual_ = model.getState().position_;

        //////////////////////////////////////////////////
        // Compute PD control torques and store them in command_ for
        // later retrieval. If enabled, add the estimated effect of
        // gravity in order to make the robot behave as if was
        // weightless.

        // Just a test for now
        if ( ! model.getMassInertia(mass_inertia_)) {
            return jspace::Status(false, "failed to retrieve mass matrix");
        }

        if (mode_ == 0) {
            command_ = jspace::Vector::Zero(model.getNDOF());
        } else if (mode_ == 1) {
            command_ = kp_.cwise() * (goal1_ - actual_) - kd_.cwise() * model.getState().velocity_;
        } else if (mode_ == 2) {
            command_ = kp_.cwise() * (goal2_ - actual_) - kd_.cwise() * model.getState().velocity_;
        } else if (mode_ == 3) {
            command_ = mass_inertia_ * (kp_M_.cwise() * (goal1_ - actual_) - kd_M_.cwise() * model.getState().velocity_);
        } else if (mode_ == 4) {
            command_ = mass_inertia_ * (kp_M_.cwise() * (goal2_ - actual_) - kd_M_.cwise() * model.getState().velocity_);
        } else {
            command_ = jspace::Vector::Zero(model.getNDOF());
            printf("ERROR UNKNOWN MODE, fallback to gravity compensation\n");
        }

        jspace::Vector gg;
        if ( ! model.getGravity(gg)) {
            return jspace::Status(false, "failed to retrieve gravity torque");
        }
        command_ += gg;

        jspace::Status ok;
        return ok;
    }

    int mode_;

    jspace::Vector kp_;
    jspace::Vector kd_;

    jspace::Vector kp_M_;
    jspace::Vector kd_M_;

    jspace::Vector goal1_;
    jspace::Vector goal2_;

    jspace::Matrix mass_inertia_;

};

}


class WBCthread : public RateThread {
protected:
    yarp::dev::IPositionControl *pos;
    yarp::dev::IControlMode *ictrl;
    yarp::dev::ITorqueControl *itrq;
    yarp::dev::ICartesianControl *icart;
    sharedArea *pMem;

    AWLinEstimator *linEst;

    boost::shared_ptr<jspace::Model> model;
    boost::shared_ptr<tut03::JTask> jtask;

    jspace::State state;
    jspace::Vector command;

    yarp::sig::Vector sig_q;
    yarp::sig::Vector sig_dq;

    yarp::sig::Vector threshold;

    yarp::sig::Vector evalVel(const yarp::sig::Vector &x);


    int mode;
    size_t iteration;

    bool verbose;

#ifdef FILE_LOGGER
    std::ofstream logger;
#endif

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

