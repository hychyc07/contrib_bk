// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
 * \defgroup icub_controlbasis_libraries Control Basis Libraries
 * @ingroup icub_controlbasis_libraries
 *
 */
#ifndef _LOGPOLAR_HEADING_SQUARED_ERROR__H_
#define _LOGPOLAR_HEADING_SQUARED_ERROR__H_

#include "ControlBasisPotentialFunction.h"

namespace CB {

    /**
     * This class implements a Squared Error quadratic potential function
     * for a LogPolarHeading resources, s.t., \phi = (1/2)*gamma^T*gamma.  
     * Implemented as gamma_reference - gamma_current.
     **/
    class LogPolarHeadingSquaredError : public ControlBasisPotentialFunction {
        
    public:
        
        /**
         * Empty Constructor, needs to set configuration info
         **/
        LogPolarHeadingSquaredError() :
            ControlBasisPotentialFunction("squared_error_pf", "logpolarheading", true)        
        {                        
            size = 2;
            gradient.resize(size);           
        }

        /**
         * Destructor
         **/        
        ~LogPolarHeadingSquaredError() { }

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


    };
    
}

#endif
