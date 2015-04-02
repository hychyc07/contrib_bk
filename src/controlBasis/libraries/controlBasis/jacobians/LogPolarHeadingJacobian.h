// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
 * \defgroup icub_controlbasis_libraries Control Basis Libraries
 * @ingroup icub_controlbasis_libraries
 *
 */
#ifndef _LOGPOLAR_HEADING_JACOBIAN__H_
#define _LOGPOLAR_HEADING_JACOBIAN__H_

#include "ControlBasisJacobian.h"

namespace CB {

    /**
     * This class implements the generic Jacobian class for a simple 2-DOF LogPolarHeading Jacobian.
     * This Jacobian is a simple 2x2 matrix that maps headings to pan/tilt angles (it, therefore
     * does not consiser vergence angles...)
     **/
    class LogPolarHeadingJacobian : public ControlBasisJacobian {

    protected:

        yarp::sig::Vector logPolarVals;

        /**
         * Port to get the logpolar information of the device
         **/
        yarp::os::BufferedPort<yarp::os::Bottle> logPolarPort;

    public:

        /**
         * Constructor.
         **/        
        LogPolarHeadingJacobian() :
            ControlBasisJacobian("logpolarheading", "configuration", 2, 3)
        {                      
            J.resize(outputSize,inputSize);
            J.zero();
            logPolarVals.resize(2);
        }
        
        /**
         * destructor
         **/
        ~LogPolarHeadingJacobian() {  }

        /**
         * inherited update function
         **/
        virtual bool updateJacobian();

        /**
         * inherited start function
         **/
        virtual void startJacobian();

        /**
         * inherited stop function
         **/
        virtual void stopJacobian();

        /**
         * inherited connect to inputs function
         **/
        virtual bool connectToInputs();

    };
    
}

#endif
