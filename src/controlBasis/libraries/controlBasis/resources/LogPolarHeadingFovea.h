// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _LOGPOLAR_HEADING_FOVEA__H_
#define _LOGPOLAR_HEADING_FOVEA__H_

#include "LogPolarHeading.h"
#include <vector>

namespace CB {
    
    /**
     * Implements the LogPolarHeading abstract interface to return the 
     * fovea positon (i.e., [0 0])
     **/
    class LogPolarHeadingFovea : public LogPolarHeading {
       
        
    public:
        
        /**
         * Constructor
         */
        LogPolarHeadingFovea(std::string name="") {
            deviceName=name+"/fovea";
            // set fovea position ([0 0])
            values.zero();            
        }
        
        /** 
         * Destructor
         **/
        ~LogPolarHeadingFovea() {  }
        
        /**
         * Inherited update function.
         * \returns success on update
         **/
        bool updateResource() { return true; }

        /**
         * Inherited start function.
         **/
        void startResource() { start(); }

        /**
         * Inherited stop function.
         **/
        void stopResource() { stop(); }

        
    };
    
}

#endif
