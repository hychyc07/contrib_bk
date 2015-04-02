// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _ICUB_TRIANGULATED_CARTESIANPOSITION__H_
#define _ICUB_TRIANGULATED_CARTESIANPOSITION__H_

#include "CartesianPosition.h"

#include <vector>

namespace CB {
    
    /**
     * Implements the CartesianPosition abstract interface to retun the 
     * iCubTriangulated position of visual features perceived by the iCub.
     * It either "outsources" the triangulation process to the iCub iKinHead 
     * module or uses a learned model obtained by the learningMachine.
     **/
    class iCubTriangulatedCartesianPosition : public CartesianPosition {
        
    protected:
        
        /**
         * flag for whether the resource is connected to the left and right heading resources
         **/
        bool connectedToHeadings;

        /**
         * storage for left heading value
         **/
        yarp::sig::Vector leftHeading;

        /**
         * storage for right heading value
         **/
        yarp::sig::Vector rightHeading;

        /**
         * name of the left heading resource type
         **/
        std::string leftHeadingName;

        /**
         * name of the right heading resource type
         **/
        std::string rightHeadingName;

        /**
         * Port to send headings to the iKinHead module.
         **/
        yarp::os::BufferedPort<yarp::os::Bottle> eyeTriangulationInputPort;
        
        /**
         * Port to read triangulated position from the iKinHead module
         */
        yarp::os::BufferedPort<yarp::os::Bottle> eyeTriangulationOutputPort;

        /**
         * flag for whether using iKin or learningMachine model
         **/
        bool useLearnedModel;

    public:
        
        /**
         * Constructor
         */
        iCubTriangulatedCartesianPosition(std::string devName, std::string left_heading="left_eye/attentionMechanism", std::string right_heading="right_eye/attentionMechanism", bool useLearnedModel=true) 
            : connectedToHeadings(false),
              leftHeading(2),
              rightHeading(2),
              leftHeadingName(left_heading),
              rightHeadingName(right_heading),
              useLearnedModel(useLearnedModel)
        {

            deviceName=devName + "/blobTracker";

            numInputs = 2;
            inputName.push_back(leftHeadingName + "/heading");
            inputName.push_back(rightHeadingName + "/heading");

        }
        
        /** 
         * Destructor
         **/
        ~iCubTriangulatedCartesianPosition() { }
        
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
         * A functon that connects the iCubTriangulated resource to 
         * the two CB heading resources it needs as input
         **/
        bool connectToHeadings();

        
    };
    
}

#endif
