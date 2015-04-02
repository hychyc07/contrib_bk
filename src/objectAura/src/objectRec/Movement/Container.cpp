/*
 * Container.cpp
 *
 *  Created on: Aug 12, 2010
 *      Author: alemme
 */

#include "Container.h"

Container::Container(string what) {

	if (!connectIKin(what)) {
		cout << "could not connect to Solver" << what.c_str() << endl;
	}

}

Container::~Container() {
	// TODO Auto-generated destructor stub
}

EndeffectorPos Container::getActualCoord() {
	return EE;
}

void Container::setActualCoord(double x, double y, double z, double a,
		double b, double c) {
	EE = EndeffectorPos(x, y, z, a, b, c);
}

void Container::setActualCoord(EndeffectorPos e) {
	EE = e;
}


int Container::connectIKin(string whatArm) {
	Bottle cmd, replyX, replyQ;

	// prepare ports
	vout_kin.open(("/v" + whatArm + ":o").c_str());
	xout_kin.open(("/x" + whatArm + ":o").c_str());
	Network::connect(("/iKinArmCtrl/" + whatArm + "_arm/x:o").c_str(),
			xout_kin.getName().c_str());
	Network::connect(("/iKinArmCtrl/" + whatArm + "_arm/v:o").c_str(),
			vout_kin.getName().c_str());

	xout_kin.read(replyX);
	cout << "x       =" << replyX.toString() << endl;

	this->setActualCoord(replyX.get(0).asDouble(), replyX.get(1).asDouble(),
			replyX.get(2).asDouble(), replyX.get(3).asDouble(),
			replyX.get(4).asDouble(), replyX.get(5).asDouble());
	return 1;
}

int Container::listener() {
	Bottle cmd, replyX, replyQ;
	xout_kin.read(replyX);


	this->setActualCoord(replyX.get(0).asDouble(), replyX.get(1).asDouble(),
			replyX.get(2).asDouble(), replyX.get(3).asDouble(),
			replyX.get(4).asDouble(), replyX.get(5).asDouble());
	cout << "getActualCoord Eigenmodel: " << this->getActualCoord() << endl;
	return 1;
}

int Container::disconnectIKin() {
	// close up
	xout_kin.close();
	vout_kin.close();

	return 0;
}

void Container::save(ofstream* out) {
	EndeffectorPos ep = getActualCoord();
	*out << ep.x << ' ' << ep.y << ' ' << ep.z << ' ' << endl;
}
