// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _CARTESIAN_ORIENTATION_REFERENCE__H_
#define _CARTESIAN_ORIENTATION_REFERENCE__H_

#include "CartesianOrientation.h"
#include <string>
#include <iostream>

namespace CB {
    
    class CartesianOrientationReference : public CartesianOrientation {
        
    public:
        
        /**
         * Constructor
         **/        
        CartesianOrientationReference(std::string name) {
            deviceName = name + "/ref";
            std::cout << "Creating new CartesianOrientationReference, name=" << deviceName.c_str() << std::endl;            
        }
        
        /**
         * Destructor
         **/        
        ~CartesianOrientationReference() { }
       
        // functions from ControlBasisResource
        bool updateResource() { return true; }
        void startResource() { start(); }
        void stopResource() { stop(); }
        
        // new functions
        void setVals(yarp::sig::Vector ref) {
            if(ref.size() != values.size()) {
                fprintf(stderr, "Couldn't set reference orientation for %s!!\n", resourceName.c_str());
                return;
            } else {
                values = ref;
            }
        }

    };
    
}

#endif
