/*
 * CartesianImpedance.cpp
 *
 *  Created on: Jul 23, 2010
 *      Author: naveenoid
 */

#include "CartesianImpedance.hpp"


/*
 * Project Defines
 */

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>


/*
 * Additional Defines
 */
//#include<vector>

/*
 * Namespaces
 */

//using std::vector;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;



namespace HandSome {

CartesianImpedance::CartesianImpedance(PolyDriver *pd1,PolyDriver *pd2, double period) : RateThread(int(period*1000.0)){
	driver1 = pd1;
	driver2 = pd2;
}


void setSpring(Vector pos,Vector orient)
{
//	springPos = pos;
//	springOrient = orient;
}


//CartesianImpedance::~CartesianImpedance() {
//	// TODO Auto-generated destructor stub
//}



}
