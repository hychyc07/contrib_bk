// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _LOGPOLAR_BLOB_TRACKER_HEADING__H_
#define _LOGPOLAR_BLOB_TRACKER_HEADING__H_

#include "LogPolarHeading.h"
#include <vector>
#include <deque>

namespace CB {
    
    /**
     * Implements the Heading abstract interface to return the 
     * coordinates of an image feature evaluated through the 
     * LOGPOLAR BlobTracker modules
     **/
    class LogPolarBlobTrackerHeading : public LogPolarHeading {
        
    protected:
        
        /**
         * flag for whether the resource is connected to a configuration resource.
         **/
        bool connectedToBlobTracker;
        
        /*
         * the heading  velocitoes
         **/
        yarp::sig::Vector headingVelocity;

        /**
         * the blob ID for the blobTacker.  
         * -1 gives the average of all blobs
         **/
        int blobID;
                

    public:
        
        /**
         * Constructor
         */
        LogPolarBlobTrackerHeading(std::string name, int blob=-1) 
            : connectedToBlobTracker(false),
              blobID(blob)
        {

            std::cout << "starting logpolar blobtracker heading with name: " << name << std::endl;
            deviceName = name;
            numInputs=1;
            inputName.push_back("logPolarBlobData");
            
            //            updateDelay=0.01;
            valid=false;
            values.resize(2);

            if(blobID < -1) {
                std::cout << "LogPolarBlobTracker -- blobID invalid!!" << std::endl;
                return;
            }

            headingVelocity.resize(2);
            headingVelocity.zero();
                       
        }
        
        /** 
         * Destructor
         **/
        ~LogPolarBlobTrackerHeading() { 
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
         * A functon that connects the resource to 
         * the CB blobTracker device it represents.
         **/
        bool connectToBlobTracker();

        /**
         * returns the heading velocities
         **/
        yarp::sig::Vector getHeadingVelocity();
        
    };
}

#endif
