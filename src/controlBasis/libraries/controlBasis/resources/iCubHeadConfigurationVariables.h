// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _ICUB_HEAD_CONFIGURATION_VARIABLES__H_
#define _ICUB_HEAD_CONFIGURATION_VARIABLES__H_

#include "YARPConfigurationVariables.h"

namespace CB {
    
    /**
     * This class extendes the YARPConfigurationVariables resource type for
     * the iCub head subsystem, remapping to (pan, tilt, yaw) from (tilt, yaw, pan)
     **/    
    class iCubHeadConfigurationVariables : public YARPConfigurationVariables {

    private:

        int iterations;

    protected:
        
        /**
         * use iCub simulator flag
         **/
        bool simulationMode;

        /**
         * Matrix to map virutal input angles to head p/t/y from t/y/p
         **/
        yarp::sig::Matrix mapping;

        /**
         * Input Vector (in p/t/y)
         **/
        yarp::sig::Vector virtualInputData;

        /**
         * use yaw
         **/
        bool usingYaw;

    public:
        
        /**
         * Constructor.  
         **/
        iCubHeadConfigurationVariables(bool simMode=false, bool yaw=false)  :           
            iterations(0),
            mapping(1,1),
            virtualInputData(1),
            usingYaw(yaw)
        {
                        
            std::cout << "iCubHeadConfigurationVariables created" << std::endl;  

            connectedToDevice=false;
            velocityControlMode=false;              
            velocityGain=10;
            simulationMode=simMode;

            mapping.resize(3,2+(int)usingYaw);   
            virtualInputData.resize(2+(int)usingYaw);           
            numDOFs = 2+(int)usingYaw; 
             
            numLinks = numDOFs;
            size=numDOFs;
            moveable = true;
            lock = true;
            maxSetVal=10;

            std::string robot_prefix;
            if(simulationMode) {
                robot_prefix = "/icubSim";
            } else {
                robot_prefix = "/icub";
            }            
            yarpDeviceName = robot_prefix+"/head";

            deviceName = yarpDeviceName;

            localDevPort = yarpDeviceName;
            remoteDevPort = yarpDeviceName;

            // mask must deal with full 6-DOF head system
            mask.resize(6);     

            // set the mask values to allow only the DOFs we want to control
            mask[0] = 1; // tilt
            mask[1] = (int)usingYaw; // yaw
            mask[2] = 1; // pan
            mask[3] = 0; // eyes[0]
            mask[4] = 0; // eyes[1]
            mask[5] = 0; // eyes[2]

            std::cout << "mask[1]: " << mask[1] << std::endl;

            // virtual DOFs
            minLimits.resize(2+(int)usingYaw);
            maxLimits.resize(2+(int)usingYaw);
            DHParameters.resize(4,numLinks);
            LinkTypes.resize(numLinks);

            // 3-DOF t/y/p commands
            values.resize(numDOFs);
            desiredValues.resize(numDOFs);
            
            // zero data
            values.zero();
            desiredValues.zero();
            DHParameters.zero();
            virtualInputData.zero();
            mapping.zero();
            minLimits.zero();
            maxLimits.zero();

            for(int i=0; i<LinkTypes.size(); i++) {
                LinkTypes[i] = (int)LINK_TYPE_REVOLUTE;
            }

        }
        
        /**
         * Destructor
         **/
        ~iCubHeadConfigurationVariables() { }
        
        virtual void startResource();

        virtual bool updateResource();

        virtual void postData();

        virtual void getInputData();

        virtual bool connectToDevice();
    };
    
}

#endif
