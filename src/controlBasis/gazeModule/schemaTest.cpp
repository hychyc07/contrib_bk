// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include <iostream>
#include <string>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <Schema.h>

#include <PotentialFunctionRegister.h>
#include <JacobianRegister.h>

using namespace yarp::os;
using namespace std;
using namespace CB;

int main(int argc, char *argv[]) {
    
  //    registerPotentialFunctions();
  // registerJacobians();

    Schema schema;

    /*
    RunnableControlLaw controlLaw;
    controlLaw.useTranspose(true);

    string sen = "/cb/configuration/icubSim/right_arm";
    string eff = "/cb/configuration/icubSim/right_arm";
    string pf = "/cb/configuration/cosfield_pf";
    controlLaw.addController(sen,pf,eff,10.0);    

    controlLaw.startAction();

    double p, pdot;
    string s_str = controlLaw.getControllerStateString(0);
    while(s_str!="1") {
        p = controlLaw.getControllerPotential(0);
        pdot = controlLaw.getControllerPotentialDot(0);
        cout << "state: " << s_str.c_str() << ", p: " << p << ", pdot: " << pdot << endl;
        s_str = controlLaw.getControllerStateString(0);
        Time::delay(0.5);
    }
    cout << "state: " << s_str.c_str() << ", p: " << p << ", pdot: " << pdot << endl;

    controlLaw.stopAction();
    */
    return 1;
}
