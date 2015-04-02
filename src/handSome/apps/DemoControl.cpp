#include <stdio.h>
#include <string>
#include <math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include "impController.hpp"
#include "robot.hpp"

#define PCONTROLLER	0.3

#define REFSPEED	5	
#define REFACC		5

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

using namespace HandSome;

void firstPhase();
void secondPhase();
void thirdPhase();
void fourthPhase();

Vector jointsLeft, jointsRight, jointImpedance;

int main(int argc, char *argv[]) {
	
	Network yarp;
	Property params;
	
	Robot * robot;
	PositionImpedanceThd* controllerLeftArm;
	PositionImpedanceThd* controllerRightArm;
	
	//Vector jointImpedance;
	int joint,nJoints;
	
	
	// Create context
	params.fromCommand(argc, argv);
	if (!params.check("robot")) {
		fprintf(stderr, "Please specify the name of the robot\n");
		fprintf(stderr, "  --robot name (e.g. icub, icubSim)\n");
		return -1;
	}
	 
	//Initialize the Robot
	robot = new Robot(0);
	robot->init(params.find("robot").asString().c_str());

	// Stiffness vector
	nJoints = robot->get_dof(LEFT_ARM);
	jointImpedance.resize(nJoints);
	
	// Default stiffness
	for (joint = 0; joint < nJoints; ++joint) {
		jointImpedance[joint] = 0;
	}
	
	//Controller threads
	controllerLeftArm = new PositionImpedanceThd(
			robot->get_driver(LEFT_ARM), (double) REFSPEED, (double) REFACC, jointImpedance,
			(double) PCONTROLLER);
	controllerRightArm = new PositionImpedanceThd(
			robot->get_driver(RIGHT_ARM), (double) REFSPEED, (double) REFACC, jointImpedance,
			(double) PCONTROLLER);

	// Starting the controllers threads
	if (!controllerLeftArm->start()) {return 0;}
	if (!controllerRightArm->start()) {return 0;}
	sleep(1);
	

	//SetPoints Vector
	bool doneLeft, doneRight;
	jointsLeft.resize(nJoints);
	jointsRight.resize(nJoints);
	
	
	/*
	 * Phase 1 - Initial position
	 *
	 * Stiff movement to the initial arm position.
	 */
	printf("\n+++ Phase 1.\n");
	firstPhase();
	controllerLeftArm->enableImpedance(jointImpedance);
	controllerLeftArm->setJoints(jointsLeft);
	controllerRightArm->setJoints(jointsRight);
	sleep(3);
	printf("+++ Finished phase 1.\n");
	
		
	/*
	 * Phase 2 - Two handed grasping
	 *
	 * Compliant movement to grasp an object between the arms.
	 */
	printf("\n+++ Phase 2.\n");
	secondPhase();
	controllerLeftArm->enableImpedance(jointImpedance);
	controllerLeftArm->setJoints(jointsLeft);
	controllerRightArm->setJoints(jointsRight);
	sleep(3);
	printf("+++ Finished phase 2.\n");


	/*
	 * Phase 3 - Two-Handed Object Tracking
	 *
	 * Following the object
	 */
	// Some simulated behaviour
	/*
	unsigned int step;
	double prec = 20.0;
	for (step = 0; step < 200; ++step) {

		// Up and down
		jointsLeft[0] = -40 + 10 * sin(step / prec * 3.142);
		jointsRight[0] = -40 + 10 * sin(step / prec * 3.142);
		jointsLeft[1] = 10 + 5 * cos(step / prec * 3.142);
		jointsRight[1] = 10 + 5 * cos(step / prec * 3.142);

		// Left and right
		jointsLeft[2] = 23 + 5 * cos(step / prec * 3.142);
		jointsRight[2] = 23 + 5 * cos(step / prec * 3.142);

		// Hand orientation
		jointsLeft[5] = -27 + 5 * cos(step / prec * 3.142);
		jointsRight[5] = -27 + 5 * cos(step / prec * 3.142);

		controllerLeftArm->setJoints(jointsLeft);
		controllerRightArm->setJoints(jointsRight);

		usleep(100000);
	}
	*/
	
	while(getchar() != 'a'){}

	/*
	 * Tear down
	 *
	 * Releasing the object, deconstructing
	 */
	// Back to initial arm position
	printf("\n+++ Phase 4.\n");
	fourthPhase();
	controllerLeftArm->setJoints(jointsLeft);
	controllerRightArm->setJoints(jointsRight);
	sleep(3);
	printf("+++ Finished phase 4.\n");
	
	printf("\n+++ Released the object, Demo finished.\n");


	controllerLeftArm->stop();
	controllerRightArm->stop();
	
	delete robot;
	delete controllerLeftArm;
	delete controllerRightArm;

	return 0;
}

void init(){}

void firstPhase()
{

	int joint = 0;	
	
	for (joint = 0; joint < jointsLeft.size(); ++joint) {
		jointsLeft[joint] = 0;
		jointsRight[joint] = 0;
	}

	// Stiff movement in the first phase
	for (joint = 0; joint < jointsLeft.size(); ++joint) {
		jointImpedance[joint] = 0;
	}
	
	// Setting initial arm position
	jointsLeft[0] = -40;
	jointsLeft[1] = 20;
	jointsLeft[2] = -15;
	jointsLeft[3] = 50;
	jointsLeft[4] = 0;
	jointsLeft[5] = 0;
	
	jointsRight[0] = -40;
	jointsRight[1] = 20;
	jointsRight[2] = -15;
	jointsRight[3] = 50;
	jointsRight[4] = 0;
	jointsRight[5] = 0;

	// Waiting until movement is done
	/*
	doneLeft = false;
	doneRight = false;
	while (!(doneLeft && doneRight)) {
		controllerLeftArm->checkMotionDone(&doneLeft);
		controllerRightArm->checkMotionDone(&doneRight);
		Time::delay(0.1);
	}
	*/
	
}


void secondPhase()
{
	int joint = 0;	
	
	for (joint = 0; joint < jointsLeft.size(); ++joint) {
		jointsLeft[joint] = 0;
		jointsRight[joint] = 0;
	}

	// Stiff movement in the first phase
	for (joint = 0; joint < jointsLeft.size(); ++joint) {
		jointImpedance[joint] = 1;
	}

	// Grasping position
	jointsLeft[0] = -40;
	jointsLeft[1] = 30;
	jointsLeft[2] = 23;
	jointsLeft[3] = 70;
	jointsLeft[4] = -10;
	jointsLeft[5] = -27;
	
	jointsRight[0] = -40;
	jointsRight[1] = 30;
	jointsRight[2] = 23;
	jointsRight[3] = 70;
	jointsRight[4] = -10;
	jointsRight[5] = -27;


	// Waiting until movement is done
	/*
	doneLeft = false;
	doneRight = false;
	while (!(doneLeft && doneRight)) {
		controllerLeftArm->checkMotionDone(&doneLeft);
		controllerRightArm->checkMotionDone(&doneRight);
		Time::delay(0.1);
	}
	*/

}

void thirdPhase(){}

void fourthPhase()
{
	jointsLeft[0] = -40;
	jointsLeft[1] = 15;
	jointsLeft[2] = -10;
	jointsLeft[3] = 50;
	jointsLeft[5] = -15;
	
	jointsRight[0] = -40;
	jointsRight[1] = 15;
	jointsRight[2] = -10;
	jointsRight[3] = 50;
	jointsRight[5] = -15;
	
	// Waiting until movement is done
	/*
	doneLeft = false;
	doneRight = false;
	while (!(doneLeft && doneRight)) {
		controllerLeftArm->checkMotionDone(&doneLeft);
		controllerRightArm->checkMotionDone(&doneRight);
		Time::delay(0.1);
	}
	*/
	
}
