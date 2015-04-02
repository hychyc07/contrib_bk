
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
    try {
        model.reset(jspace::test::parse_sai_xml_file(saixml.c_str(), true));
        jtask.reset(new tut03::JTask());
    } catch (std::runtime_error const & ee) {
        errx(EXIT_FAILURE, "%s", ee.what());
    }

#ifdef FILE_LOGGER
    Value filevalue = rf.check("out","logger.txt","checking if given an alternate file name");
    printf("Logging to: %s\n",filevalue.asString().c_str());
    logger.open(filevalue.asString(), std::ios::out);
#endif

    threshold.resize(N_FORCE,0);
    threshold[0] = 2.5;
    threshold[1] = 7.0;
    threshold[2] = 2.5;
    threshold[3] = 1.5;
    threshold[4] = 0.5;

    pos->positionMove(0,-28.0);
    pos->positionMove(1,80.0);
    pos->positionMove(2,15.0);
    pos->positionMove(3,50.0);
    bool done = false;
    while(1) {
        printf("Waiting for init position achievement...\n");
        Time::delay(0.5);
        pos->checkMotionDone(&done);
        if(done) break;
    }

    while (pMem->getQ().size() == 0) {
        Time::delay(0.5);
        printf("Waiting for q...\n");
    }

    state.position_ = jspace::Vector::Zero(N_TOTAL);
    state.velocity_ = jspace::Vector::Zero(N_TOTAL);
    sig_q = pMem->getQ();
    for(int ii=0;ii<N_TOTAL;ii++)
        state.position_[ii] = CTRL_DEG2RAD * sig_q(ii);

    model->update(state);
    jtask->init(*model);

    mode = 0;
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

    // a) get our updated encoder values and speeds
    sig_q = pMem->getQ();
    sig_dq = evalVel(sig_q);
    for(int ii=0;ii<N_TOTAL;ii++) {
        state.position_[ii] = CTRL_DEG2RAD * sig_q(ii);
        state.velocity_[ii] = CTRL_DEG2RAD * sig_dq(ii);
    }
    
    model->update(state);  // Update the model to reflect the current robot state.

    // b) calculate and send our torques

    jtask->mode_ = mode;

    jtask->update(*model);
    command = jtask->getCommand();

    if ((!verbose)&&(0 == (iteration % 20))) {
#ifdef FILE_LOGGER
    if (mode!=0)
        for (int k=0;k<5;k++) logger << state.position_[k] << " ";  // Sent to logger file
#endif

        jspace::pretty_print(state.position_, std::cerr, "  jpos", "    ");
        jspace::pretty_print(state.velocity_, std::cerr, "  jvel", "    ");
        jspace::pretty_print(command, std::cerr, "  command", "    ");
        if (mode==0) {
            std::cerr << "mode: pure gravity compensation\n";
        } else if (mode==1) {
            std::cerr << "mode: jtask with goal1\n";
            jspace::pretty_print(jtask->goal1_, std::cerr, "  goal", "    ");
#ifdef FILE_LOGGER
        for (int k=0;k<5;k++) logger << jtask->goal1_[k] << " ";  // Sent to logger file
#endif
        } else if (mode==2) {
            std::cerr << "mode: jtask with goal2\n";
            jspace::pretty_print(jtask->goal2_, std::cerr, "  goal", "    ");
#ifdef FILE_LOGGER
        for (int k=0;k<5;k++) logger << jtask->goal2_[k] << " ";  // Sent to logger file
#endif
        } else if (mode==3) {
            std::cerr << "mode: jtask with goal1 using M\n";
            jspace::pretty_print(jtask->goal1_, std::cerr, "  goal", "    ");
            jspace::pretty_print(jtask->mass_inertia_, std::cerr, "  mass matrix", "    ");
#ifdef FILE_LOGGER
        for (int k=0;k<5;k++) logger << jtask->goal1_[k] << " ";  // Sent to logger file
#endif
        } else if (mode==4) {
            std::cerr << "mode: jtask with goal2 using M\n";
            jspace::pretty_print(jtask->goal2_, std::cerr, "  goal", "    ");
            jspace::pretty_print(jtask->mass_inertia_, std::cerr, "  mass matrix", "    ");
#ifdef FILE_LOGGER
        for (int k=0;k<5;k++) logger << jtask->goal2_[k] << " ";  // Sent to logger file
#endif
        } else 
            std::cerr << "INVALID MODE\n";

        for(int ii=0; ii<N_FORCE; ii++) {
            if (fabs(command[ii])>threshold[ii])
               fprintf(stderr, "%d not sent (%f). ", ii,command[ii]);
        }
        printf("\n");
#ifdef FILE_LOGGER
    if (mode!=0)
        logger << std::endl;  // Sent to logger file
#endif
    }

#ifdef SEND_COMMANDS
    if (verbose) {
        fprintf(stderr, "(REAL) ");
        jspace::pretty_print(state.position_, std::cerr, "  jpos", "    ");
        jspace::pretty_print(state.velocity_, std::cerr, "  jvel", "    ");
        jspace::pretty_print(command, std::cerr, "  command", "    ");
        if (mode==0) {
            std::cerr << "mode: pure gravity compensation\n";
        } else if (mode==1) {
            std::cerr << "mode: jtask with goal1\n";
            jspace::pretty_print(jtask->goal1_, std::cerr, "  goal", "    ");
        } else if (mode==2) {
            std::cerr << "mode: jtask with goal2\n";
            jspace::pretty_print(jtask->goal2_, std::cerr, "  goal", "    ");
        } else if (mode==3) {
            std::cerr << "mode: jtask with goal1 using M\n";
            jspace::pretty_print(jtask->goal1_, std::cerr, "  goal", "    ");
            jspace::pretty_print(jtask->mass_inertia_, std::cerr, "  mass matrix", "    ");
        } else if (mode==4) {
            std::cerr << "mode: jtask with goal2 using M\n";
            jspace::pretty_print(jtask->goal2_, std::cerr, "  goal", "    ");
            jspace::pretty_print(jtask->mass_inertia_, std::cerr, "  mass matrix", "    ");
        } else 
            std::cerr << "INVALID MODE\n";
    }
    for (int i = 0; i < N_FORCE; i++)
        if (command[i] > threshold[i]) {
            fprintf(stderr, "%d saturated (%f). ", i,command[i]);
            itrq->setRefTorque(i, threshold[i]);
        } else if (command[i] < (-threshold[i])) {
            fprintf(stderr, "%d saturated (%f). ", i,command[i]);
            itrq->setRefTorque(i, (-threshold[i]));
        } else  // normal case
            itrq->setRefTorque(i, command[i]);
#else
    if (verbose) {
        fprintf(stderr, "(SIM) ");
        jspace::pretty_print(state.position_, std::cerr, "  jpos", "    ");
        jspace::pretty_print(state.velocity_, std::cerr, "  jvel", "    ");
        jspace::pretty_print(command, std::cerr, "  command", "    ");
        if (mode==0) {
            std::cerr << "mode: pure gravity compensation\n";
        } else if (mode==1) {
            std::cerr << "mode: jtask with goal1\n";
            jspace::pretty_print(jtask->goal1_, std::cerr, "  goal", "    ");
        } else if (mode==2) {
            std::cerr << "mode: jtask with goal2\n";
            jspace::pretty_print(jtask->goal2_, std::cerr, "  goal", "    ");
        } else if (mode==3) {
            std::cerr << "mode: jtask with goal1 using M\n";
            jspace::pretty_print(jtask->goal1_, std::cerr, "  goal", "    ");
        } else if (mode==4) {
            std::cerr << "mode: jtask with goal2 using M\n";
            jspace::pretty_print(jtask->goal2_, std::cerr, "  goal", "    ");
        } else 
            std::cerr << "INVALID MODE\n";
    }
    for (int i = 0; i < N_FORCE; i++)
        if (fabs(command[i]) > threshold[i])
            fprintf(stderr, "%d not sent too high. ", i);
#endif

    ++iteration;

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

#ifdef FILE_LOGGER
    logger.close();
#endif

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

