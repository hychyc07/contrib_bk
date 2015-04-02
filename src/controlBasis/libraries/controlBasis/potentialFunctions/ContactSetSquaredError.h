// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
 * \defgroup icub_controlbasis_libraries Control Basis Libraries
 * @ingroup icub_controlbasis_libraries
 *
 */
#ifndef _CONTACT_SET_SQUARED_ERROR__H_
#define _CONTACT_SET_SQUARED_ERROR__H_

#include "ControlBasisPotentialFunction.h"

namespace CB {

    /**
     * This class implements a Squared Error quadratic potential function
     * for Cartesian Position resources, s.t., \phi = (1/2)*x^T*X.  
     * Implemented as x_reference - x_current.
     **/
    class ContactSetSquaredError : public ControlBasisPotentialFunction {
        
    public:
        
        /**
         * Empty Constructor, needs to set configuration info
         **/
        ContactSetSquaredError() :
            ControlBasisPotentialFunction("squared_error_pf", "contactset", true)        
        {                        
            size = 1;
            gradient.resize(size);           
        }

        /**
         * Destructor
         **/        
        ~ContactSetSquaredError() { }

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
         * timing store
         **/
        double t0;
    };
    
}

#endif
