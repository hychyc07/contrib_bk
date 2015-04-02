// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _CARTESIAN_POSITION_REFERENCE__H_
#define _CARTESIAN_POSITION_REFERENCE__H_

#include "CartesianPosition.h"
#include <string>
#include <iostream>

namespace CB {
    
    class CartesianPositionReference : public CartesianPosition {
        
    public:
        
        /**
         * Constructor
         **/        
        CartesianPositionReference(std::string name) {            
            deviceName = name + "/ref";
            std::cout << "Creating new CartesianPositionReference, name=" << deviceName.c_str() << std::endl;            
        }
        
        /**
         * Destructor
         **/        
        ~CartesianPositionReference() { }

        // functions from ControlBasisResource
        bool updateResource() { 
            double dt = getRate() / 1000.0; 
            return true; 
        }

        void startResource() { start(); }
        void stopResource() { stop(); }
        
        // new functions
        void setVals(yarp::sig::Vector ref) {
            if(ref.size() != values.size()) {
                fprintf(stderr, "Couldn't set reference position for %s!!\n", resourceName.c_str());
                return;
            } else {
                values = ref;
            }
        }

    };
    
}

#endif
