// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _YARP_BLOB_TRACKER_HEADING__H_
#define _YARP_BLOB_TRACKER_HEADING__H_

#include "Heading.h"
#include <vector>
#include <deque>

namespace CB {
    
    /**
     * Implements the Heading abstract interface to return the 
     * coordinates of an image feature evaluated through the 
     * YARP BlobTracker modules
     **/
    class YARPBlobTrackerHeading : public Heading {
        
    protected:
        
        /**
         * flag for whether the resource is connected to a configuration resource.
         **/
        bool connectedToBlobTracker;
        
        /*
         * the image coordinates
         **/
        yarp::sig::Vector imageCoordinates;

        /*
         * the heading  velocitoes
         **/
        yarp::sig::Vector headingVelocity;

        /**
         * the blob ID for the blobTacker.  
         * -1 gives the average (centroid) of all blobs
         **/
        int blobID;

        /**
         * if this flag is set, the resource will
         * report the average heading from two blobTracker
         * resources.  
         **/
        bool avgMode;
        
        /**
         * the second blobTracker name (if in average mode)
         **/
        std::string deviceNames[2];
        

    public:
        
        /**
         * Constructor
         */
        YARPBlobTrackerHeading(std::string name_0, std::string name_1="", int blob=-1) 
            : connectedToBlobTracker(false),
              imageCoordinates(2),
              blobID(blob)
        {

            deviceNames[0] = name_0;
            deviceNames[1] = name_1;

            if(name_1 != "") {
                deviceName = "/blobTracker/avg";
                deviceNames[0] = name_0;
                deviceNames[1] = name_1;
                avgMode = true;
                numInputs=2;
                inputName.push_back("blobData_0");
                inputName.push_back("blobData_1");
            } else {
                deviceName = name_0; 
                avgMode = false;
                numInputs=1;
                inputName.push_back("blobData");
            }

            //            updateDelay=0.01;
            valid=false;
            values.resize(2);

            if(blobID < -1) {
                std::cout << "YARPBlobTracker -- blobID invalid!!" << std::endl;
                return;
            }


            headingVelocity.resize(2);
            headingVelocity.zero();
                       
        }
        
        /** 
         * Destructor
         **/
        ~YARPBlobTrackerHeading() { 
        }
        
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
        bool connectToBlobTracker();

        /**
         * returns the heading velocities
         **/
        yarp::sig::Vector getHeadingVelocity();
        
    };
}

#endif
