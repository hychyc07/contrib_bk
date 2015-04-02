#include "Container.h"
#include <fstream>
#include <yarp/os/Network.h>
#include <iostream>
#include <stdio.h>
#include <yarp/dev/Drivers.h>

YARP_DECLARE_DEVICES( icubmod);

using namespace yarp::os;
using namespace std;

void createBottle(Bottle* b, double x, double y, double z) {
	cout << "create bottle" << endl;
	b->clear();
	b->addDouble(x);
	b->addDouble(y);
	b->addDouble(z);
	b->addDouble(0.0);
	b->addDouble(-1.0);
	b->addDouble(0.0);
	b->addDouble(0.0);
	cout << b->toString() << endl;
}

int main(int argc, char * argv[]) {
	// initialization of the yarp network instance.
	Network yarp;
	if (!yarp.checkNetwork())
		return -1;

	Port pl, pr; // Create a port.
	pl.open("/MTLeft"); // Give it a name on the network.
	pr.open("/MTRight"); // Give it a name on the network.
	Network::connect("/MTLeft", "/iKinArmCtrl/left_arm/xd:i");
	Network::connect("/MTRight", "/iKinArmCtrl/right_arm/xd:i");

	ofstream* out;
	int k = 0;
	double omega = 2 * M_PI / 400.0;
	bool next = true;
	double delta = 0.005;
	double x, y, z, yy;
	Bottle* b;
	b = new Bottle();
	bool use_left_hand;

	Container* lcontainer = new Container("left");
	Container* rcontainer = new Container("right");


	while (k < 901) {
		lcontainer->listener();
		rcontainer->listener();
		if (next) {
			x = -0.25;//-0.08 * (0.5 * sin(omega * k)) - 0.2;
			y = 0.15 * (cos(6 * omega * k));
			z = 0.15 * (sin(6 * omega * k)) + 0.05;

			use_left_hand = (y < 0);
			if (use_left_hand) {
				y -= 0.13;
				createBottle(b, x, y, z);
				pl.write(*b); //send data
				yy = 0.13;
				createBottle(b, x, yy, z);
				pr.write(*b); //send data
			} else {
				y += 0.13;
				createBottle(b, x, y, z);
				pr.write(*b); //send data
				yy = -0.13;
				createBottle(b, x, yy, z);
				pl.write(*b); //send data
			}

			next = false;
			k++;
		}

		sleep(0.8);

		EndeffectorPos ep;
		if (use_left_hand) {
			cout << "move left arm" << endl;
			ep = lcontainer->getActualCoord();
		} else {
			cout << "move right arm" << endl;
			ep = rcontainer->getActualCoord();
		}
		double tmp = (ep.x - x) * (ep.x - x) + (ep.y - y) * (ep.y - y) + (ep.z
				- z) * (ep.z - z);
		if (tmp < delta) {
			next = true;
			//container->save(out);
			cout << "position reached" << endl << endl;
		} else {
			cout << "diff to big (" << fabs(tmp) << ")" << endl << endl;
		}
	}

	delete lcontainer;
	delete rcontainer;
	delete out;
	delete b;
	return 0;
}
