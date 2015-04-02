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
     * It "outsources" the triangulation process to the iCub iKinHead module.
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

    public:
        
        /**
         * Constructor
         */
        iCubTriangulatedCartesianPosition(std::string devName, std::string left_heading="left_eye/attentionMechanism", std::string right_heading="right_eye/attentionMechanism") 
            : connectedToHeadings(false),
              leftHeadingName(left_heading),
              rightHeadingName(right_heading),
              leftHeading(2),
              rightHeading(2)
        {

            deviceName=devName;

            numInputs = 2;
            inputName.push_back(leftHeadingName + "/heading");
            inputName.push_back(rightHeadingName + "/heading");

            updateDelay=0.01;

            // mandatory inherit function
            initPorts();

            std::cout << "iCubTriangulatedPosition input[0]: " << inputName[0] << std::endl;
            std::cout << "iCubTriangulatedPosition input[1]: " << inputName[1] << std::endl;
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
