// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _LOGPOLAR_HEADING__H_
#define _LOGPOLAR_HEADING__H_

#include "ControlBasisResource.h"

namespace CB {

    /**
     * This class instantiates the abstract ControlBasisResource class for a LogPolarHeading 
     * type resource. This type of resource provides interesting log polar image coordinates to 
     * control actions.  It is still abstract, in that it doesn't implement a runnable resource. 
     * This class must be extended for specific heading sensor device that implements the 
     * start, update, and stop functions.
     **/
    class LogPolarHeading : public ControlBasisResource {       
        
    public:
        
        /**
         * returns the horizontal angle of the image coordinate of the position
         **/    
        double get_gamma_rho() { return values[0]; }

        /**
         * returns the vertical angle of the image coordinate of the position
         **/    
        double get_gamma_phi() { return values[1]; }

        /**
         * Constructor
         **/
        LogPolarHeading() :
            ControlBasisResource("logpolarheading", 0, 1) 
        {        
            std::cout << "setting type of LogPolarHeading to " << type.c_str() << std::endl;            
            size = 2;
            values.resize(size);         
            outputName.push_back("data");            
        }   
 
        /**
         * Destructor
         **/        
        ~LogPolarHeading() { };      
 
       /**
         * This is the function that posts the resource data to the output port.
         * it is type specific, so it is defined here.  it is automatically called 
         * after the update() function in the main thread loop.
         **/
        void postData() {
            yarp::os::Bottle &b = outputPort[0]->prepare();
            b.clear();
            b.addString(resourceName.c_str());
            b.addInt((int)valid);
            b.addDouble(get_gamma_rho());
            b.addDouble(get_gamma_phi());
            outputPort[0]->write();
        }

        /**   
         * getInputData() function. 
         */
        virtual void getInputData() { }
        
    };
    
}

#endif


