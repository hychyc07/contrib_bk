// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _STEREO_HEADING__H_
#define _STEREO_HEADING__H_

#include "ControlBasisResource.h"

namespace CB {

    /**
     * This class instantiates the abstract ControlBasisResource class for a Stereo Heading 
     * type resource. This type of resource provides interesting pairs of image coordinates to 
     * control actions.  It is still abstract, in that it doesn't implement a runnable resource. 
     * This class must be extended for a specific stereo heading device that implements the 
     * start, update, and stop functions.
     **/
    class StereoHeading : public ControlBasisResource {       
        
    public:
        
        /**
         * returns the horizontal angle of the left image coordinate of the position
         **/    
        double get_gamma_u_left() { return values[0]; }

        /**
         * returns the vertical angle of the left image coordinate of the position
         **/    
        double get_gamma_v_left() { return values[1]; }

        /**
         * returns the horizontal angle of the right image coordinate of the position
         **/    
        double get_gamma_u_right() { return values[2]; }

        /**
         * returns the vertical angle of the right image coordinate of the position
         **/    
        double get_gamma_v_right() { return values[3]; }

        /**
         * Constructor
         **/
        StereoHeading() :
            ControlBasisResource("stereoheading", 0, 1) 
        {        
            std::cout << "setting type of StereoHeading to " << type.c_str() << std::endl;            
            size = 4;
            values.resize(size);         
            outputName.push_back("data");            
        }   
 
        /**
         * Destructor
         **/        
        ~StereoHeading() { };      
 
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
            b.addDouble(get_gamma_u_left());
            b.addDouble(get_gamma_v_left());
            b.addDouble(get_gamma_u_right());
            b.addDouble(get_gamma_v_right());
            outputPort[0]->write();
        }

        /**   
         * getInputData() function. 
         */
        virtual void getInputData() { }
        
    };
    
}

#endif


