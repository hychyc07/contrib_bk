

#ifndef MODULE_H_
#define MODULE_H_

//yarp includes
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

//local includes
#include <iCub/piSquare/policyImprovementManager/policyImprovementManager.h>

using namespace pi2;
using namespace library;
using namespace yarp::os;
using namespace yarp::sig;


class Module : public PolicyImprovementManager
{

public:

    Module(ResourceFinder* rf, double movementDuration, Vector& start, Vector& goal);
    virtual ~Module();

private:

    bool computeCost(dmp::Trajectory&, Vector&);

};


#endif /* MODULE_H_ */
