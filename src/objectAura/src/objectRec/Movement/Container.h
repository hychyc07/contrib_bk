/*
 * Container.h
 *
 *  Created on: Aug 12, 2010
 *      Author: alemme
 */

#ifndef CONTAINER_H_
#define CONTAINER_H_
#include "EndeffectorPos.h"
#include <fstream>
#include <string>
//#include "ModelSender.h"
#include <yarp/os/Network.h>
#include <iCub/iKin/iKinVocabs.h>
#include <iCub/iKin/iKinHlp.h>
#include <iCub/iKin/iKinSlv.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;

class Container {
public:
	Container(string what);
	virtual ~Container();
	EndeffectorPos getActualCoord();
	void setActualCoord(EndeffectorPos e);
	void setActualCoord(double x, double y, double z, double a, double b,
			double c);
	void save(ofstream* out);

	int listener();
	int connectIKin(string whatArm);
	int disconnectIKin();

protected:

	EndeffectorPos EE;
	Port xout_kin, vout_kin; /**iKin port to handle messages*/
};

#endif /* Container_H_ */
