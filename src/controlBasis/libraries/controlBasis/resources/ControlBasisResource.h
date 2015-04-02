// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _CONTROL_BASIS_RESOURCE__H_
#define _CONTROL_BASIS_RESOURCE__H_

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>
#include <vector>
#include <iostream>
#include <controlBasis/cb.h>

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

#ifndef TODEG
#define TODEG 180.0/M_PI
#endif

#ifndef TORAD
#define TORAD M_PI/180.0
#endif


namespace CB {
    
    /**
     * The Abstract Control Basis Sensory and Motor Resource Class.  
     */
    class ControlBasisResource : public yarp::os::RateThread {
        
    protected:
        
        /**
         * the name of the device
         **/
        std::string deviceName;
        
        /**
         * the name of the resource
         **/
        std::string resourceName;
        
        /** 
         * The formal type of the resource (e.g., CartesianPosition, ConfigurationVariable, etc.)
         **/
        std::string type;

        /**
         * The resource data vals
         **/
        yarp::sig::Vector values;

        /**
         * the size of the data values
         **/
        int size;
        
        /**
         * The Yarp output ports
         **/
        std::vector<yarp::os::BufferedPort<yarp::os::Bottle> *> outputPort;
        
        /**
         * The Yarp input Ports
         **/
        std::vector<yarp::os::BufferedPort<yarp::os::Bottle> *> inputPort;
        
        /**
         * num output ports
         **/
        int numOutputs;
        
        /** 
         * num input ports
         **/
        int numInputs;
        
        /**
         * The Yarp output port names
         **/
        std::vector<std::string> outputPortName;
        
        /** 
         * The YARP input port name
         **/
        std::vector<std::string> inputPortName;

        /**
         * The output names
         **/
        std::vector<std::string> outputName;

        /**
         * The input names
         **/
        std::vector<std::string> inputName;
        
        /**
         * Lock flag to prevent writing vals to hardware
         **/
        bool lock;
        
        /**
         * valid flag
         **/
        bool valid;

        /**
         * update estimate (in ms)
         **/
        double dt;


    public:
        
        /** 
         * Getter for the resource name
         * \return the name
         **/
        std::string getResourceName() { return resourceName; }  
        
        /** 
         * Getter for the device name
         * \return the name
         **/
        std::string getDeviceName() { return deviceName; }  
        
        /** 
         * Getter for the resource type
         * \return the type
         **/
        std::string getResourceType() { return type; }

        /**
         * Getter for the resource data
         **/
        yarp::sig::Vector getResourceData() { return values; }

        /**
         * Getter for the resource data size
         **/
        int getResourceDataSize() { return size; }
        
        /**
         * Setter for updated delay
         * \param delay (in milliseconds)
         **/
        void setUpdateDelay(int p) { setRate(p); }

        /**
         * Setter for locking/unlocking the ability to set 
         * output variables.
         * \param lock
         **/
        void setLock(bool l) { lock = l; }
        
        /** 
         * virtual update function
         **/
        virtual bool updateResource()=0;

        /**
         * virtual start function
         **/
        virtual void startResource()=0;

        /**
         * virtual stop function
         **/
        virtual void stopResource()=0;
        
        /**
         * virtual post data function to be filled in by instantiation
         **/
        virtual void postData()=0;

        /**
         * virtual set data function to be filled in by instantiation
         **/
        virtual void getInputData()=0;
        
        /**
         * returns whether resource value is valid.
         **/
        bool isValid() { return valid; }

        bool threadInit() {

            // set up port names
            resourceName = "/cb/" + type + deviceName;
            
            std::cout << "ControlBasisResource::init() name=" << resourceName.c_str() << std::endl;
            
            //std::cout << "configuring " << numOutputs << " outputs for " << resourceName.c_str() << std::endl;
            outputPort.clear();
            outputPortName.clear();
            for(int i=0; i<numOutputs; i++) {
                outputPortName.push_back(resourceName + "/" + outputName[i] +  ":o");
                outputPort.push_back(new yarp::os::BufferedPort<yarp::os::Bottle>);
                outputPort[i]->open(outputPortName[i].c_str());
            }

            //std::cout << "configuring " << numInputs << " inputs for " << resourceName.c_str() << std::endl;
            inputPort.clear();
            inputPortName.clear();
            for(int i=0; i<numInputs; i++) {
                inputPortName.push_back(resourceName + "/" + inputName[i] +  ":i");
                inputPort.push_back(new yarp::os::BufferedPort<yarp::os::Bottle>);
                inputPort[i]->open(inputPortName[i].c_str());
            }

            //std::cout << "ControlBasisResource::init() initialized" << std::endl;
            
            return true;
        }

        /**
         * main run function for resource (instatiates for thread)
         **/
        void run() {

            // update resource values
            if(!updateResource()) {                                        
                std::cout << "Problem updating resource: " << resourceName.c_str() << "!!" << std::endl;
                return;
            }
            
            // post input/output data to ports
            if(numOutputs > 0) postData();
            if(numInputs > 0) getInputData();

            // calculate period (in seconds)
            dt = (double)getEstPeriod()/1000.0;

        }


        void threadRelease() {

            std::cout << "ControlBasisResource::threadRelease()" << std::endl;

            // close ports
            for(unsigned int i=0; i<inputPort.size(); i++) {
                inputPort[i]->close(); 
            }
            for(unsigned int i=0; i<outputPort.size(); i++) {
                outputPort[i]->close(); 
            }
            std::cout << "ControlBasisResource::threadRelease() -- closed ports..." << std::endl;

        }
      
        /**
         * Constructor
         **/
        ControlBasisResource(std::string type, int numInputs, int numOutputs) :
            yarp::os::RateThread(10),
            type(type),
            values(1),
            numOutputs(numOutputs),
            numInputs(numInputs),
            lock(true),            
            valid(true)
        {
        }

        /** 
         * Destructor
         **/
        ~ControlBasisResource() {          

            for(unsigned int i=0; i<inputPort.size(); i++) {
                inputPort[i]->close(); 
                delete inputPort[i];
            }

            for(unsigned int i=0; i<outputPort.size(); i++) {
                outputPort[i]->close(); 
                delete outputPort[i];
            }

        }
        
    };
    
}

#endif
