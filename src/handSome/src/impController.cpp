//Author: 	Cristiano Alessandro
//email:	alessandro@ifi.uzh.ch

#include "impController.hpp"
#include <yarp/os/Time.h>

using namespace HandSome;

/*
 Input Parameters:
 Polydriver p 	- device to control
 refSpeed		- reference speed at each joint
 refAcc			- reference acceleration at each joint
 compJoints		- the j-th element is 1 if the corresponding joint has to be controlled in impedance
 the j-th element is 0 if the corresponding joint has to be controlled in position
 period			- period of the thread
 */
PositionImpedanceThd::PositionImpedanceThd(PolyDriver* p, double refSpeed,
		double refAcc, Vector& compJoints, double period) :
	ControllerThd(double(period)) {
	this->client = p;
	this->refSpeed = refSpeed;
	this->refAcc = refAcc;
	this->compJ = compJoints;
	this->flag = false;
}

bool PositionImpedanceThd::threadInit() {
	if (!client)
		return false;

	// get the interfaces
	client->view(imp);
	client->view(pos);
	client->view(mode);

	int nDOF = compJ.length();
	if (nDOF) {
		setReferences(nDOF);
		enableImpedance(compJ);

		cout << "Position Control Thread Initialized!" << endl;
		return true;
	} else {
		cout << "Problem Initializing the Position Control Thread!" << endl;
		return false;
	}

}

/*
 The setpoint vector cannot be read and written at the same time.
 This is controlled by a mutex mechanism.
 */
void PositionImpedanceThd::run() {
	if (setPoints.size()) {
		pos->checkMotionDone(&flag);
		if (flag)
		{
			jointMutex->wait(); //Lock the semaphore
			pos->positionMove(setPoints.data());
			jointMutex->post(); //Unlock the semaphore
		}
	}
}

void PositionImpedanceThd::threadRelease() {
	pos->stop();
	cout << "Impedance control thread released!" << endl;
}

bool PositionImpedanceThd::enableImpedance(Vector& joints) {
	bool ok = true;
	int i = 0;
	for (i = 0; i < joints.length(); i++) {
		if (joints[i]) {
			ok = ok && mode->setImpedancePositionMode(i);
			ok = ok && imp->setImpedance(i, STIFF, DAMP, 0);
		}
	}

	if (ok && i) {
		cout << "Impedance set at each joint!" << endl;
		return true;
	} else {
		cout << "Problem setting the impedance!" << endl;
		return false;
	}
}

void PositionImpedanceThd::setReferences(int njs) {
	Vector tmp;
	tmp.resize(njs);
	for (int i = 0; i < njs; i++) {
		pos->setRefSpeed(i, this->refAcc);
		pos->setRefAcceleration(i, this->refAcc);
	}
}

bool PositionImpedanceThd::checkMotionDone(bool* flag) {
	return pos->checkMotionDone(flag);
}
