#include<iostream>
//#include "CartesianImpedance.hpp"

#include "JointImpedance.hpp"
#include "controller.hpp"

#include "gazeController.hpp"

#include <yarp/os/Network.h>
//#include
#include <yarp/os/Time.h>


using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;


//#include "TorqueCalc.h"

using namespace HandSome;
int main()
{
    std::cout<<"Booah main began";

    Network yarp;

//    options.put("device", "remote_controlboard");
    Property option1;
    option1.put("device", "remote_controlboard");
    option1.put("remote","/icubSim/left_arm");
    option1.put("local","/local_client/left_arm");

    Property option2;
    option2.put("device", "remote_controlboard");
    option2.put("remote","/icubSim/right_arm");
    option2.put("local","/local_client/right_arm");

    PolyDriver *client_left = NULL, *client_right = NULL;

    client_left=new PolyDriver;
    if (!client_left->open(option1))
    {
        delete client_left;
//        return false;
        return 0;
    }

    client_right = new PolyDriver;
    if (!client_right->open(option2))
    {
        delete client_right;
//        return false;
        return 0;
    }

    Property optionGaze;

    optionGaze.put("device","gazecontrollerclient");
     optionGaze.put("remote","/iKinGazeCtrl");
 	optionGaze.put("local","/handsome/gaze");

 	PolyDriver *head_neck;

 	head_neck = new PolyDriver(optionGaze);

 	if (!head_neck->isValid()) {
         printf(" Hand/Neck device not available.\n");
         return false;
     }
//     head_neck->limb = new iCubEyeNeckRefDyn ();

//    controllerLeftArm = new PositionImpedanceThd(
//    			client_left, (double) REFSPEED, (double) REFACC, jointImpedance,
//    			(double) PCONTROLLER);

    ControllerThd * ctr = NULL;

	Vector joints;
	joints.resize(7);
	joints[0] = 0;
	joints[1] = 0;
	joints[2] = 0;
	joints[3] = 0;
	joints[4] = 0;
	joints[5] = 0;
	joints[6] = 0;

	//I need to know the size of the arms
	GazeControlThd* gaze = new GazeControlThd(head_neck,joints,0.2);

    JointImpedance *jointImpedance = new JointImpedance(client_left, client_right, ctr,gaze,0.1,0.01);


    if (!jointImpedance->start())
     {
         delete jointImpedance;
     }

    Time::delay(25);

    jointImpedance->stop();
    delete jointImpedance;
    return 0;



}
