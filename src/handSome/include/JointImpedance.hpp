/*
 * JointImpedance.hpp
 *
 *  Created on: Jul 23, 2010
 *      Author: naveenoid
 */

#ifndef JOINTIMPEDANCE_HPP_
#define JOINTIMPEDANCE_HPP_

/*
 * Local Header
 */
#include "CartesianImpedance.hpp"
#include "controller.hpp"
#include "gazeController.hpp"
/*
 * Project Headers
 */

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include<yarp/math/Math.h>
#include<yarp/os/RateThread.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iKin/iKinIpOpt.h>

//#include

#include <yarp/sig/Vector.h>

/*
 * Additional Headers
 */



/*
 * Namespaces
 */

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iDyn;
using namespace iCub::iKin;


namespace HandSome {


class JointImpedance :  public CartesianImpedance //, public CartesianImpedance
{

public:

	/* Constructor sets both the polydriver and the cartesian control interface */
	JointImpedance(PolyDriver* = NULL, PolyDriver* = NULL, ControllerThd * = NULL, GazeControlThd * = NULL,double = 0.0, double = 0.01);
			//

	/* Thread Stuff */
	virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();

	virtual ~JointImpedance();

private :

	GazeControlThd *gct;
	/* All 3 of the functionality below are for 1 arm only */

	/* obtaining the cartesian ee position */
    virtual Vector getCartesian(IEncoders *, iCubArmDyn *);

    /* obtaining the desired ee position */
    virtual void computeDesired(void);

virtual void computeDesiredVariable(void);

    /* obtaining the compute jointspace position*/
    virtual void computeJointSpace();

    Vector desiredPose1;
    Vector desiredPose2;

    iCubArmDyn *arm1 ;
    iCubArmDyn *arm2;

    IEncoders *ienc1;
    IEncoders *ienc2;

	iKinChain *chain1;
	iKinChain *chain2;

	double restLength;

};

}
#endif /* JOINTIMPEDANCE_H_ */
