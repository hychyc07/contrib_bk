// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _CONTROL_BASIS_ACTION__H_
#define _CONTROL_BASIS_ACTION__H_

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>

#include <string>
#include <iostream>
#include <vector>

/**
 * This will provide an abstract interface to control basis actions. Actions can be 
 * control primitives, multi-objective (prioritized) control laws, or programs of 
 * other actions.  All actions will use a 4-predicate dynamic state logic:
 * 
 * p \in {X,-,0,1}, where
 *     X  -- unknown (action is not being run)
 *     -  -- undefined (there is no reference input, or the action is outside its admissible state space)
 *     0  -- unconverged (transient response as action is progressing towards attractor state
 *     1  -- converged (or, more accurately 'quiescence,' capturing when the change in potential/value is stops changing)
 *
 *
 **/
namespace CB {
    
    /*
     * Dynamic State enum
     **/
    enum ActionState {
        UNKNOWN=0,
        UNDEFINED,
        UNCONVERGED,
        CONVERGED
    };

    /**
     * The Abstract Control Basis Action Class
     */
    class ControlBasisAction : public yarp::os::RateThread {
        
    protected:

        /**
         * The dynamic state of the action
         **/
        ActionState dynamicState;

        /**
         * output ports
         **/
        std::vector<yarp::os::BufferedPort<yarp::os::Bottle> *> outputPorts;
        
        /**
         * output port names
         **/
        std::vector<std::string> outputPortNames;

        /**
         * output names
         **/
        std::vector<std::string> outputNames;

        /**
         * num output ports
         **/
        int numOutputs;

        /**
         * the name of the action
         **/ 
        std::string actionName;

        /**
         * started flag (it takes a while to start and connect all the pieces of each controller/schema)
         **/
        bool started;
        
    public:
        
        /** 
         * Getter for the action name
         * \return the name
         **/
        std::string getActionName() { return actionName; } 

        /** 
         * Getter for action state
         * \return the state
         **/
        ActionState getState() { return dynamicState; }

        bool hasStarted() {
            return started;
        }
                
        /** 
         * virtual update function
         **/
        virtual bool updateAction()=0;

        /**
         * virtual start function
         **/
        virtual void startAction()=0;

        /**
         * virtual stop function
         **/
        virtual void stopAction()=0;

        /**
         * virtual pause function
         **/
        virtual void pauseAction()=0;

        /**
         * virtual post data function
         **/
        virtual void postData()=0;

        /**
         * Initiallization function for starting ports
         **/
        bool threadInit() {
            std::cout << "ControlBasisAction::init()" << std::endl;
            outputPorts.clear();
            outputPortNames.clear();
            for(int i=0; i<numOutputs; i++) {
                outputPortNames.push_back(actionName + "/" + outputNames[i] +  ":o");
                outputPorts.push_back(new yarp::os::BufferedPort<yarp::os::Bottle>);
                outputPorts[i]->open(outputPortNames[i].c_str());
            }
            return true;
        }
               
        /** 
         * release function.  closes and clears the ports.
         **/
        void threadRelease() {
            for(unsigned int i=0; i<outputPorts.size(); i++) {
                outputPorts[i]->close();
            }
            outputPorts.clear();
            outputPortNames.clear();
            outputNames.clear();
            dynamicState=UNKNOWN;
        }

        /**
         * run function
         **/
        void run() {

            if(!updateAction()) {
                std::cout << "Problem updating control action!!" << std::endl;
                return;
            }

            // post input/output data to ports
            if(numOutputs > 0) postData();
            //if(numInputs > 0) getInputData();
            
        }

      
        /**
         * Constructor
         **/
        ControlBasisAction() :
            RateThread(10),
            dynamicState(UNKNOWN),
            numOutputs(0),
            started(false)
        {
        }

        /**
         * Destructor
         **/
        ~ControlBasisAction() {  
            if(isRunning()) {
                stop();
            }
        }

    };
    
}

#endif
