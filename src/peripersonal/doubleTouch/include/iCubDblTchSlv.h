#ifndef __ICUBCUSTOMLIMB_H__
#define __ICUBCUSTOMLIMB_H__

#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinIpOpt.h>
#include <iCub/iKin/iKinSlv.h>

#include "iKinFwdMod.h"

using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace std;

/****************************************************************/
/****************************************************************/
struct iCubDoubleTouch_Problem
{
    iCubCustomLimb limb;
    iCubFinger     index;
    iKinLinIneqConstr *RLIC;
    iKinLinIneqConstr *LLIC;

    int nJoints;
    int nVars;

    iCubDoubleTouch_Problem(const iCubDoubleTouch_Problem &_p) : limb(_p.limb), index(_p.index),
                     nJoints(_p.nJoints), nVars(_p.nVars), RLIC(_p.RLIC), LLIC(_p.LLIC) {}

    iCubDoubleTouch_Problem(std::string _type);

    std::string        getType()  { return limb.getType(); }
    int                getNVars() { return nVars; }
    iKinLinIneqConstr* getRLIC()  { return RLIC; }
    iKinLinIneqConstr* getLLIC()  { return LLIC; }
};

/****************************************************************/
/****************************************************************/
struct iCubDoubleTouch_Variables
{    
    yarp::sig::Vector ee;
    yarp::sig::Vector joints;

    yarp::sig::Matrix H;
    yarp::sig::Matrix H_0;

    yarp::sig::Vector z_hat;
    yarp::sig::Vector x_hat;

    double dot;

    /**
    * Constructor. 
    * @param dim is the number of links of the chain to be dealt with
    */
    iCubDoubleTouch_Variables(int dim);

    /**
    * Prints the state of the variable
    */
    void print();

    /**
    * Copy Operator
    */
    iCubDoubleTouch_Variables &operator=(const iCubDoubleTouch_Variables &v);

    /**
    * Clone Function
    */
    void clone (const iCubDoubleTouch_Variables &v);
};

/****************************************************************/
/****************************************************************/
class iCubDoubleTouch_Solver
{
protected:
    iCubDoubleTouch_Problem   &problem;
    iCubDoubleTouch_Variables  guess;

public:
    iCubDoubleTouch_Solver(iCubDoubleTouch_Problem &_problem) : problem(_problem), guess(_problem.getNVars()) { };

    bool solve(iCubDoubleTouch_Variables &solution);

    void setInitialGuess(const iCubDoubleTouch_Variables &g){ guess=g; };
};

#endif
