// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
 * \defgroup icub_controlbasis_libraries Control Basis Libraries
 * @ingroup icub_controlbasis_libraries
 *
 */
#ifndef _MANIPULABILITY_FIELD__H_
#define _MANIPULABILITY_FIELD__H_

#include <deque>

#include <iCub/iKin/iKinFwd.h>

#include "cb.h"
#include "ControlBasisPotentialFunction.h"

namespace CB {

    /**
     * This potential function implements Yoshikawa's Manipulability field for a manipulator.
     * This field is defined as \phi= sqrt(det(J*J^T)), where J is the manipulator jacobian.
     **/
    class ManipulabilityField : public ControlBasisPotentialFunction {
        
    protected:

        /**
         * The iKin Limb
         **/
        iCub::iKin::iKinLimb kinLimb;

        /**
         * The pionter to the iKin Chain
         **/
        iCub::iKin::iKinChain *kinChain;

        /** 
         * storage for DH parameters
         **/
        yarp::sig::Matrix DHParams;

        /**
         * storage for link type information
         **/
        yarp::sig::Vector LinkTypes;

        /**
         * port to get DH parameter and link information
         **/
        yarp::os::BufferedPort<yarp::os::Bottle> paramsInputPort;

        /**
         * indicator flag concerning whether DH parameter info has been set
         **/
        bool paramsSet;

        /**
         * linked list to the kinematic links
         **/
        std::deque<iCub::iKin::iKinLink*> linkList;

    public:

        /**
         * Empty Constructor, needs to set configuration info
         **/
        ManipulabilityField() :
            ControlBasisPotentialFunction("manipulability_pf","configuration",false)
        {

            inputs.resize(1);
            DHParams.resize(1,1);
            LinkTypes.resize(1);
            gradient.resize(1);

            std::cout << "Created new ManipulabilityField... " << std::endl;
        }
        
        /**
         * Destructor
         **/
        ~ManipulabilityField() { }

        /**
         * inherited update function
         **/
        virtual bool updatePotentialFunction();

        /**
         * inherited start function
         **/
        virtual void startPotentialFunction();

        /**
         * inherited stop function
         **/
        virtual void stopPotentialFunction();

        /**
         * inherited connect function
         **/
        virtual bool connectToInputs();

        /**
         * gets the manipulability metric for the specified Jacobian matrix
         **/
        double getManipulability(yarp::sig::Matrix J);


    };
    
}

#endif
