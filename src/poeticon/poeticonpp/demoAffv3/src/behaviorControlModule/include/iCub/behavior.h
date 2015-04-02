// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Luis Montesano lmontesano@isr.ist.utl.pt
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __DABEHAVIOR__
#define __DABEHAVIOR__


#ifndef M_PI_2
#define M_PI_2	((float)(asin(1.0)))
#endif
#ifndef M_PI
#define M_PI	((float)(2*M_PI_2))
#endif

 // std
#include <stdio.h>
#include <string>
#include <iostream>

// yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

//#include "../../lib/attentionSystem/src/attentionSelection/include/iCub/RemoteAttentionSelection.h"
#include <iCub/RemoteAttentionSelection.h>
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace iCub {
     namespace contrib {
         class demoAff;
     }
}



using namespace iCub::contrib;

/**
 *
 * affordance demo Module class
 *
 * \see icub_demoaff
 *
 */
class Behavior : public RFModule {

private:

  // ports
  BufferedPort<Bottle> port_gaze;
  BufferedPort<Bottle> port_aff_in;
  BufferedPort<Bottle> port_aff_out;
  BufferedPort<Bottle> port_in; 
  
  
  int state; // Current state of the FSM
  int next_state;

  float cg_az_max, cg_az_min;
  float cg_el_max, cg_el_min;

  double min_attention_time;
  double attention_time;

  int OK_MSG;
  int nMoves;

  bool respond(const Bottle &command,Bottle &reply);
  bool readDetectorInput(Bottle *Input, string name, double *cx, double *cy);

public:

    Behavior();
    virtual ~Behavior();
    
    virtual bool configure(ResourceFinder &config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();

private:
    RemoteAttentionSelection remoteAtt;
};


#endif
