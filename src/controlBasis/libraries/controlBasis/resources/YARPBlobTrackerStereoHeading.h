// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _YARP_BLOB_TRACKER_STEREO_HEADING__H_
#define _YARP_BLOB_TRACKER_STEREO_HEADING__H_

#include "StereoHeading.h"
#include <vector>
#include <yarp/sig/Matrix.h>

namespace CB {
    
    /**
     * Implements the StereoHeading abstract interface to return the 
     * coordinates of an image feature evaluated through the 
     * YARP Attention Mechanism modules
     **/
    class YARPBlobTrackerStereoHeading : public StereoHeading {
        
    protected:
        
        /**
         * flag for whether the resource is connected to a configuration resource.
         **/
        bool connectedToBlobTrackers;
        
        /*
         * the image coordinates
         **/
        yarp::sig::Matrix imageCoordinates;

        /**
         * a vector to store the estimate
         **/
        yarp::sig::Vector estimate;


        /** 
         * the filter parameter
         **/
        double alpha;

    public:
        
        /**
         * Constructor
         */
        YARPBlobTrackerStereoHeading(std::string name) 
            : connectedToBlobTrackers(false),
              imageCoordinates(2,2),
              estimate(4)
        {

            deviceName=name;
            numInputs = 2;
            inputName.push_back("left_image_coordinates");
            inputName.push_back("right_image_coordinates");
            //            updateDelay=0.01;

            // filiter initialization params
            alpha=0;
            estimate.zero();
                        
        }
        
        /** 
         * Destructor
         **/
        ~YARPBlobTrackerStereoHeading() { }
        
        /**
         * Inherited update function.
         * \returns success on update
         **/
        bool updateResource();

        /**
         * Inherited start function.
         **/
        void startResource();

        /**
         * Inherited stop function.
         **/
        void stopResource();
        
        /**
         * A functon that connects the EndEffector resource to 
         * the CB configuration device resource device it represents.
         **/
        bool connectToBlobTrackers();
        
    };
    
}

#endif
