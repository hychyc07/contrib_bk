// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _PARTICLE_FILTER_TRACKER_HEADING__H_
#define _PARTICLE_FILTER_TRACKER_HEADING__H_

#include "Heading.h"
#include <vector>
#include <deque>

namespace CB {
    
    /**
     * Implements the Heading abstract interface to return the 
     * coordinates of an image feature evaluated through the 
     * YARP ParticleFilter modules
     **/
    class ParticleFilterTrackerHeading : public Heading {
        
    public:

        /**
         * modes for just hte left or just the right
         * image data, or the average of both.
         **/
        enum ParticleFilterMode {
            AVG=0,
            LEFT,
            RIGHT
        };


    protected:
        
        /**
         * flag for whether the resource is connected to a configuration resource.
         **/
        bool connectedToParticleFilterTracker;
        
        /*
         * the image coordinates
         **/
        yarp::sig::Vector imageCoordinates;

        /*
         * the last image coordinates (for vel comp)
         **/
        yarp::sig::Vector imageCoordinatesLast;

        /*
         * the heading  velocitoes
         **/
        yarp::sig::Vector headingVelocity;

        /**
         * if this flag is set, the resource will
         * report the average heading from two blobTracker
         * resources.  
         **/
        bool avgMode;

        /**
         * which mode
         **/
        ParticleFilterMode zdfMode;

        /**
         * threshold for valid particle
         **/
        double threshold;


    public:
        
        /**
         * Constructor
         */
        ParticleFilterTrackerHeading(ParticleFilterMode mode, double thresh=0.5) 
            : connectedToParticleFilterTracker(false),
              imageCoordinates(2),
              imageCoordinatesLast(2),
              zdfMode(mode),
              threshold(thresh)
        {
            
            deviceName = "/zdfTracker/";
            if(mode == AVG) {
                deviceName += "avg";
                avgMode = true;
            } else if(mode == LEFT) {
                deviceName += "left";
                avgMode = false;
            } else if(mode == RIGHT) {
                deviceName += "right";
                avgMode = false;
            }

            inputName.push_back("zdfData");

            //updateDelay=0.01;
            valid=false;
            values.resize(2);

            headingVelocity.resize(2);
            headingVelocity.zero();
            imageCoordinates.zero();
            imageCoordinatesLast.zero();
                       
        }
        
        /** 
         * Destructor
         **/
        ~ParticleFilterTrackerHeading() { 
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
        bool connectToParticleFilterTracker();

        /**
         * returns the heading velocities
         **/
        yarp::sig::Vector getHeadingVelocity();
        
    };
    
}

#endif
