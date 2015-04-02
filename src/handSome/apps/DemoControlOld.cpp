#include <stdio.h>
#include <string>
#include <math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include "impController.hpp"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

using namespace HandSome;

int main(int argc, char *argv[]) {
	Network yarp;

	Property params;
	params.fromCommand(argc, argv);

	// Create context
	if (!params.check("robot")) {
		fprintf(stderr, "Please specify the name of the robot\n");
		fprintf(stderr, "  --robot name (e.g. icub, icubSim)\n");
		return -1;
	}

	/*
	 * Setting up
	 *
	 * Setting everything up
	 */
	unsigned int joint;
	unsigned int nJoints = 7;
	std::string robotName = params.find("robot").asString().c_str();
	std::string rightArmRemote = "/";
	rightArmRemote += robotName;
	rightArmRemote += "/right_arm";
	std::string rightArmLocal = "/test/clientRightArm";
	std::string leftArmRemote = "/";
	leftArmRemote += robotName;
	leftArmRemote += "/left_arm";
	std::string leftArmLocal = "/test/clientLeftArm";

	Property options;
	options.put("device", "remote_controlboard");

	// Device for right arm
	options.put("local", rightArmLocal.c_str()); //local port names
	options.put("remote", rightArmRemote.c_str()); //where we connect to
	PolyDriver* rightArmDevice = new PolyDriver;
	rightArmDevice->open(options);

	// Device for left arm
	options.put("local", leftArmLocal.c_str()); //local port names
	options.put("remote", leftArmRemote.c_str()); //where we connect to
	PolyDriver* leftArmDevice = new PolyDriver;
	leftArmDevice->open(options);

	// Stiffness vector
	Vector jointImpedance;
	jointImpedance.resize(nJoints);
	// Setting to stiff per default
	for (joint = 0; joint < nJoints; ++joint) {
		jointImpedance[joint] = 0;
	}

	// Controller threads
	PositionImpedanceThd* controllerLeftArm = new PositionImpedanceThd(
			leftArmDevice, (double) 10, (double) 50, jointImpedance,
			(double) 0.3);
	PositionImpedanceThd* controllerRightArm = new PositionImpedanceThd(
			rightArmDevice, (double) 10, (double) 50, jointImpedance,
			(double) 0.3);

	// Starting the controllers threads
	if (!controllerLeftArm->start()) {
		return 0;
	}
	if (!controllerRightArm->start()) {
		return 0;
	}
	sleep(2);

	/*
	 * Phase 1 - Two handed grasping
	 *
	 * Stiff movement to the initial arm position.
	 */
	bool doneLeft, doneRight;
	Vector jointsLeft, jointsRight;
	jointsLeft.resize(nJoints);
	jointsRight.resize(nJoints);
	for (joint = 0; joint < nJoints; ++joint) {
		jointsLeft[joint] = 0;
		jointsRight[joint] = 0;
	}

	// Stiff movement in the first phase
	for (joint = 0; joint < nJoints; ++joint) {
		jointImpedance[joint] = 0;
	}
	controllerLeftArm->enableImpedance(jointImpedance);

	// Setting initial arm position
	jointsLeft[0] = -40;
	jointsLeft[1] = 15;
	jointsLeft[2] = -10;
	jointsLeft[3] = 50;
	jointsRight[0] = -40;
	jointsRight[1] = 15;
	jointsRight[2] = -10;
	jointsRight[3] = 50;
	controllerLeftArm->setJoints(jointsLeft);
	controllerRightArm->setJoints(jointsRight);

	// Waiting until movement is done
	doneLeft = false;
	doneRight = false;
	while (!(doneLeft && doneRight)) {
		controllerLeftArm->checkMotionDone(&doneLeft);
		controllerRightArm->checkMotionDone(&doneRight);
		Time::delay(0.1);
	}
	sleep(3);
	printf("\n+++ Finished phase 1.\n");

	/*
	 * Phase 2 - Two handed grasping
	 *
	 * Compliant movement to grasp an object between the arms.
	 */
	for (joint = 0; joint < nJoints; ++joint) {
		jointsLeft[joint] = 0;
		jointsRight[joint] = 0;
	}

	// Stiff movement in the first phase
	for (joint = 0; joint < nJoints; ++joint) {
		jointImpedance[joint] = 1;
	}
	controllerLeftArm->enableImpedance(jointImpedance);

	// Setting initial arm position
	jointsLeft[0] = -40;
	jointsLeft[1] = -10;
	jointsLeft[2] = 10;
	jointsLeft[3] = 50;
	jointsLeft[5] = -17;
	jointsRight[0] = -40;
	jointsRight[1] = -10;
	jointsRight[2] = 10;
	jointsRight[3] = 50;
	jointsRight[5] = -17;
	controllerLeftArm->setJoints(jointsLeft);
	controllerRightArm->setJoints(jointsRight);

	// Waiting until movement is done
	doneLeft = false;
	doneRight = false;
	while (!(doneLeft && doneRight)) {
		controllerLeftArm->checkMotionDone(&doneLeft);
		controllerRightArm->checkMotionDone(&doneRight);
		Time::delay(0.1);
	}
	sleep(3);
	printf("\n+++ Finished phase 2.\n");

	/*
	 * Phase 3 - Two-Handed Object Tracking
	 *
	 * Following the object
	 */

	// Some simulated behaviour
	unsigned int step;
	double prec = 20.0;
	for (step = 0; step < 200; ++step) {


		// Up and down
		jointsLeft[0] = -40 + 10 * sin(step / prec * 3.142);
		jointsRight[0] = -40 + 10 * sin(step / prec * 3.142);
		jointsLeft[1] = -10 + 5 * cos(step / prec * 3.142);
		jointsRight[1] = -10 + 5 * cos(step / prec * 3.142);

		// Left and right
		jointsLeft[2] = 10 + 5 * cos(step / prec * 3.142);
		jointsRight[2] = 10 - 5 * cos(step / prec * 3.142);

		// Hand orientation
		jointsLeft[5] = -17 - 10 * cos(step / prec * 3.142);
		jointsRight[5] = -17 + 10 * cos(step / prec * 3.142);

		controllerLeftArm->setJoints(jointsLeft);
		controllerRightArm->setJoints(jointsRight);

		usleep(100000);
	}

	/*
	 * Tear down
	 *
	 * Releasing the object, deconstructing
	 */
	// Back to initial arm position
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
	controllerLeftArm->setJoints(jointsLeft);
	controllerRightArm->setJoints(jointsRight);

	// Waiting until movement is done
	doneLeft = false;
	doneRight = false;
	while (!(doneLeft && doneRight)) {
		controllerLeftArm->checkMotionDone(&doneLeft);
		controllerRightArm->checkMotionDone(&doneRight);
		Time::delay(0.1);
	}
	sleep(2);
	printf("\n+++ Released the object, Demo finished.\n");

	controllerLeftArm->stop();
	controllerRightArm->stop();

	return 0;
}
