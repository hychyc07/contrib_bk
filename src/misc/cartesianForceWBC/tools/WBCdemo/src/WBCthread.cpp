
#include "iCub/WBCdemo/WBCthread.h"

/************************************************************************/
void WBCthread::setSharedArea(sharedArea* _pMem) {
    pMem = _pMem;
}

/************************************************************************/
void WBCthread::setIPositionControl(yarp::dev::IPositionControl *_pos) {
    pos = _pos;
}

/************************************************************************/
void WBCthread::setIControlMode(yarp::dev::IControlMode *_ictrl) {
    ictrl = _ictrl;
}

/************************************************************************/
void WBCthread::setITorqueControl(yarp::dev::ITorqueControl *_itrq) {
    itrq = _itrq;
}

/************************************************************************/
void WBCthread::setMode(const int _mode) {
    mode = _mode;
}

/************************************************************************/
int WBCthread::getMode() {
    return mode;
}

/************************************************************************/
void WBCthread::setVerbose(const bool _verbose) {
    verbose = _verbose;
}

/************************************************************************/
bool WBCthread::getVerbose() {
    return verbose;
}

/************************************************************************/
void WBCthread::init(ResourceFinder &rf) {

    linEst = new iCub::ctrl::AWLinEstimator(16,1.0);

    ConstString saixml=rf.findFile("saixml");
    printf("Loading SAIxml from file '%s'...\n",saixml.c_str());
    model.reset(jspace::test::parse_sai_xml_file(saixml.c_str(), true));

    threshold.resize(N_FORCE,0);
    threshold[0] = 2.5;
    threshold[1] = 2.5;
    threshold[2] = 2.5;
    threshold[3] = 1.0;
    threshold[4] = 0.5;

    while (pMem->getQ().size() == 0) {
        Time::delay(0.5);
        printf("Waiting for q...\n");
    }

    state.position_ = jspace::Vector::Zero(N_TOTAL);
    state.velocity_ = jspace::Vector::Zero(N_TOTAL);
    sig_q = pMem->getQ();
    for(int ii=0;ii<N_TOTAL;ii++)
        state.position_[ii] = CTRL_DEG2RAD * sig_q(ii);

    eetask.reset(new opspace::CartPosTask("tut06-eepos"));
    jspace::Vector kp(1), kd(1), maxvel(1), ctrlpt(3);
    kp << 30000.0;
    kd << 0.00001;
    maxvel << 30.0;
    ctrlpt << 0.0, 0.0, 0.0;  // to set extra ee tool length // was << 0.0, 0.0, -1.0;
    eetask->quickSetup(kp, kd, maxvel, "link6", ctrlpt);
    eegoalpos = eetask->lookupParameter("goalpos", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! eegoalpos) {
        errx(EXIT_FAILURE, "failed to find appropriate end-effector goalpos parameter");
    }
    eegoalvel = eetask->lookupParameter("goalvel", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! eegoalvel) {
        errx(EXIT_FAILURE, "failed to find appropriate end-effector goalvel parameter");
    }

    jtask.reset(new opspace::JPosTask("tut06-jtask"));
    kp << 4000.0;
    kd << 0.00001;
    maxvel << M_PI*6.0;
    jtask->quickSetup(kp, kd, maxvel);
    jgoalpos = jtask->lookupParameter("goalpos", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! jgoalpos) {
        errx(EXIT_FAILURE, "failed to find appropriate joint-posture goalpos parameter");
    }
    jgoalvel = jtask->lookupParameter("goalvel", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! jgoalvel) {
        errx(EXIT_FAILURE, "failed to find appropriate joint-posture goalvel parameter");
    }

    skill.reset(new opspace::GenericSkill("tut06-skill"));
    skill->appendTask(eetask);
    skill->appendTask(jtask);

    controller.reset(new opspace::ClassicTaskPostureController("tut06-ctrl"));

    mode = 0;
    prevmode = 42;
    iteration = 0;
    verbose = false;

#ifdef SEND_COMMANDS
    for (int i = 0; i < N_FORCE; i++)
        ictrl->setTorqueMode(i);
    fprintf(stderr,"(REAL) Set N_FORCE joints to TORQUE mode.\n");
#else
    fprintf(stderr,"(SIM) Set N_FORCE joints to TORQUE mode.\n");
#endif

    int period = rf.check("rate",20,"ms ratethread").asInt();
    this->setRate(period);
    this->start();
}

/************************************************************************/
void WBCthread::run() {
    // mode = toggle_count % 5;

    // a) get our updated encoder values and speeds
    sig_q = pMem->getQ();
    sig_dq = evalVel(sig_q);
    for(int ii=0;ii<N_TOTAL;ii++) {
        state.position_[ii] = CTRL_DEG2RAD * sig_q(ii);
        state.velocity_[ii] = CTRL_DEG2RAD * sig_dq(ii);
    }
    
    model->update(state);  // Update the model to reflect the current robot state.

    // b) calculate and send our torques
    
    if (0 == mode) {  // does nothing when in mode 0
        prevmode = mode;
        return;
    }

    if ((1 == mode) && (1 != prevmode)) {  // done only when first set to mode 1
        jspace::Status st(skill->init(*model));
        if ( ! st) {
            errx(EXIT_FAILURE, "skill->init() failed: %s", st.errstr.c_str());
        }
        st = controller->init(*model);
        if ( ! st) {
            errx(EXIT_FAILURE, "controller->init() failed: %s", st.errstr.c_str());
        }
    }

  // Try to maintain positions and velocities of the joint space
/*  jspace::Vector jpos(state.position_.rows());
  jpos << -25*M_PI/180.0, 20.0*M_PI/180.0, 0, 50.0*M_PI/180.0, 0.0, 0.0, 0.0;
  if ( ! jgoalpos->set(jpos)) {
    errx(EXIT_FAILURE, "failed to set joint goal position");
  }
  jspace::Vector jvel(state.velocity_.rows());
  jvel << 1.0, 1.0, 1.0, 1.0, 0.0, 0, 0;
  if ( ! jgoalvel->set(jvel)) {
    errx(EXIT_FAILURE, "failed to set joint goal velocity");
  }*/

    jspace::Vector pos(3), vel(3);
  pos << 0.1927, 0.113, -0.1606;
//    pos << -0.1036, -0.0148, 0.0;
    //pos << 0.0577, 0.1239, -0.12;
    if ( ! eegoalpos->set(pos)) {
        errx(EXIT_FAILURE, "failed to set end-effector goal position");
    }
    vel << 15.0, 15.0, 15.0;
    if ( ! eegoalvel->set(vel)) {
        errx(EXIT_FAILURE, "failed to set end-effector goal velocity");
    }

    if ( ! skill->update(*model)) {
        errx(EXIT_FAILURE, "skill update failed");
    }

    if ( ! controller->computeCommand(*model, *skill, command)) {
        errx(EXIT_FAILURE, "controller update failed");
    }

    if ((!verbose)&&(0 == (iteration % 500))) {
        controller->dbg(std::cerr, "**************************************************", "");
        for(int ii=0; ii<N_FORCE; ii++) {
            if (fabs(command[ii])>threshold[ii])
                printf("  %d too high.",ii);
        }
        printf("\n");
    }

#ifdef SEND_COMMANDS
    if (verbose) {
        fprintf(stderr, "(REAL) ");
        controller->dbg(std::cerr, "**************************************************", "");
    }
    for (int i = 0; i < N_FORCE; i++)
        if (fabs(command[i]) > threshold[i])
            fprintf(stderr, "%d not sent too high. ", i);
        else  // normal case
            itrq->setRefTorque(i, command[i]);
#else
    if (verbose) {
        fprintf(stderr, "(SIM) ");
        controller->dbg(std::cerr, "**************************************************", "");
    }
    for (int i = 0; i < N_FORCE; i++)
        if (fabs(command[i]) > threshold[i])
            fprintf(stderr, "%d not sent too high. ", i);
#endif

    ++iteration;
    prevmode = mode;

}

/************************************************************************/
void WBCthread::stop() {  // an emergency stop function
#ifdef SEND_COMMANDS
    for (int i = 0; i < N_FORCE; i++)
        ictrl->setPositionMode(i);
    fprintf(stderr,"(REAL) Set N_FORCE joints to POSITION mode.\n");
    Time::delay(1.0);
#else
    fprintf(stderr,"(SIM) Set N_FORCE joints to POSITION mode.\n");
#endif
}

/************************************************************************/
void WBCthread::threadRelease() {  // the cleaning function
#ifdef SEND_COMMANDS
    for (int i = 0; i < N_FORCE; i++)
        ictrl->setPositionMode(i);
    fprintf(stderr,"(REAL) Set N_FORCE joints to POSITION mode.\n");
    Time::delay(1.0);
#else
    fprintf(stderr,"(SIM) Set N_FORCE joints to POSITION mode.\n");
#endif
    delete linEst;
    linEst = 0;
}

/************************************************************************/
yarp::sig::Vector WBCthread::evalVel(const yarp::sig::Vector &x) {  // a nice auxiliary function
    iCub::ctrl::AWPolyElement el;
    el.data=x;
    el.time=Time::now();
    return linEst->estimate(el);
}

