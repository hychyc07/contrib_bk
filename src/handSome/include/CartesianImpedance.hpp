/*
 * CartesianImpedance.hpp
 *
 *  Created on: Jul 23, 2010
 *      Author: naveenoid
 *
 *      Abstract Parent Class for Cartesian Impedance Functionality
 *
 */

#ifndef CARTESIANIMPEDANCE_HPP_
#define CARTESIANIMPEDANCE_HPP_


/*
 * Project Defines
 */

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/math/Math.h>
#include <yarp/os/RateThread.h>
#include <iCub/iDyn/iDyn.h>

#include "controller.hpp"

#include <yarp/sig/Vector.h>
/*
 * Additional Defines
 */
//#include<vector>

/*
 * Namespaces
 */

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iDyn;

namespace HandSome {

class CartesianImpedance : public RateThread
{
public:

	/* Constructor sets both the polydriver and the cartesian control interface */
	CartesianImpedance(PolyDriver *pd1 = NULL,PolyDriver *pd2 = NULL, double period = 0);


    /* Thread Stuff*/

	virtual bool threadInit() = 0;
    virtual void run()  = 0;
    virtual void threadRelease() = 0;

protected:

	ControllerThd * ctrl;

	/* obtaining the cartesian ee position */
    virtual Vector getCartesian(IEncoders *, iCubArmDyn *) = 0;

    /* obtaining the desired ee position/virtual spring forces */
    virtual void computeDesired() = 0;

    /* obtaining the compute jointspace position/torques */
    virtual void computeJointSpace() = 0;

	/* x,y,z of the desired spring equilibrium position */
//	Vector springPos;
	/*a,b,c of the desired spring equilibrium orientation */
//	Vector springOrient;


	/* Arm1 */
	/* x,y,z of the current end effector position */
	Vector currentPose1;
	/*a,b,c of the current end effector orientation */
//	Vector currentOrient1;

	/* EE current translational velocity */
	Vector currentTransVel1;

	/* EE current rotational velocity */
	Vector currentRotVel1;

	/* Arm2 */
	/* x,y,z of the current end effector position */
	Vector currentPose2;
	/*a,b,c of the current end effector orientation */
//	Vector currentOrient2;

	/* EE current translational velocity */
	Vector currentTransVel2;

	/* EE current rotational velocity */
	Vector currentRotVel2;


    PolyDriver *driver1;
    PolyDriver *driver2;

};



}

#endif /* CARTESIANIMPEDANCE_HPP_ */
