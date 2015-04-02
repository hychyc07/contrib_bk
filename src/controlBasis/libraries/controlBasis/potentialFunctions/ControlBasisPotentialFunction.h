// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
 * \defgroup icub_controlbasis_libraries Control Basis Libraries
 * @ingroup icub_controlbasis_libraries
 *
 */
#ifndef _CONTROL_BASIS_POTENTIAL_FUNCTION__H_
#define _CONTROL_BASIS_POTENTIAL_FUNCTION__H_

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Random.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/SVD.h>

#include <string>
#include <vector>
#include <iostream>

namespace CB {

    /**
     * The Abstract ControlBasis Potential Function class.  Returns potential 
     * and gradient based on input. Implmenters of this class will be required to
     * compute a potential functon evaluated at a certain point, as well as the 
     * gradient of the function at that point.  The type (i.e., cartesianposition) 
     * of the PF must be specified, as well as if this PF needs a reference to
     * compute its current value.
     */
    class ControlBasisPotentialFunction : public yarp::os::RateThread {
        
    protected:
        
        /**
         * the name of the potential function
         **/
        std::string pfName;

        /**
         * the name of the type of function
         **/
        std::string pfType;

        /**
         * the space of function
         **/
        std::string pfSpace;

        /**
         * whether the PF needs a reference
         **/
        bool hasReference;

        /**
         * running flag
         **/
        //bool running;
        
        /**
         * The Yarp output port
         **/
        yarp::os::BufferedPort<yarp::os::Bottle> outputPort;
        
        /**
         * The Yarp output port name
         **/
        std::string outputPortName;
        
        /**
         * update delay (in seconds)
         **/
        //double updateDelay;
        
        /**
         * the potential value
         **/
        double potential;
        
        /**
         * the gradient of the potential wrt the input space
         **/
        yarp::sig::Vector gradient;
        
        /**
         * the input vector
         **/
        std::vector<yarp::sig::Vector *> inputs;

        /**
         * the name of the input resources
         **/
        std::vector<std::string> inputNames;
        
        /**
         * the input ports
         **/
        std::vector< yarp::os::BufferedPort<yarp::os::Bottle> *> inputPorts;
        
        /**
         * the size of the input space
         **/
        int size;
        
        /**
         * connected to inputs flag
         **/
        bool connectedToInputs;       

        /**
         * set inputs flag
         **/
        bool inputsSet;       

        /**
         * valid flag
         **/
        bool valid;

        /**
         * update estimate
         **/
        double dt;

        /**
         * time store
         **/
        double t0,t1;

    public:
        
       /**
         * Constructor
         **/
        ControlBasisPotentialFunction(std::string type, std::string space, bool hasRef) :
            RateThread(10),
            pfType(type),
            pfSpace(space),
            hasReference(hasRef),
            //running(false),
            //updateDelay(0.01),
            potential(0),
            size(1),
            connectedToInputs(false),
            inputsSet(false),
            valid(true)
        {            
        }


        /** 
         * Getter for the potential function name
         * \return the name
         **/
        std::string getPotentialName() { return pfName; }  
        
        /** 
         * Getter for the potential
         * \return the potential
         **/
        double getPotential() { return potential; }  
        
        /** 
         * Getter for the potential gradient
         * \return the gradient
         **/
        yarp::sig::Vector getPotentialGradient() { return gradient; }
        
        /** 
         * is the potential function running?
         * \return running
         **/
        //bool isPotentialRunning() { return running; }
        
        /** 
         * Getter for the potential size
         * \return the size
         **/
        int getInputSize() { return size; }  
        
        /**
         * Setter for updated delay
         * \param delay (in milliseconds)
         **/
        void setUpdateDelay(int p) { 
            setRate(p);
        }      

        /**
         * Get's wether PF can be evaluated or not
         **/
        bool isValid() { return valid; }
        /**
         * connect to input function
         **/
        virtual bool connectToInputs()=0;
        
        /** 
         * virtual update function
         **/
        virtual bool updatePotentialFunction()=0;

        /**
         * virtual start function
         **/
        virtual void startPotentialFunction()=0;
        
        /**
         * virtual stop function
         **/
        virtual void stopPotentialFunction()=0;
        
        /**
         * virtual post data function to be filled in by abstract interface
         **/
        void postData() {

            yarp::os::Bottle &b = outputPort.prepare();
            b.clear();

            b.addString(pfName.c_str());
            b.addInt((int)valid);
            b.addDouble(potential);
            
            for(int i = 0; i < gradient.size(); i++) {
                b.addDouble(gradient[i]);
            }
           
            outputPort.write();
        }
        
        /**
         * main run function for pf (instatiates for thread)
         **/
        void run() {

            if(!inputsSet) {
                std::cout << "PotentialFunction inputs not set, can't run...." << std::endl;
                return;
            }

            dt = (double)getEstPeriod() / 1000.0;
            if(!updatePotentialFunction()) {
                std::cout << "Problem updating potential function: " << pfName.c_str() << std::endl;
                return;
            }
            postData();

        }

        /**
         * Function to set input names
         * \param inNames vector of input names
         **/
        void setInputs(std::vector<std::string> inNames)
        {

            int randomID = (int)(yarp::os::Random::uniform()*1000.0);
            char *c = (char *)malloc(16);
            sprintf(c,"%d", randomID);
            std::string randomIDstr(c);

            // clear our residual names
            if(inputsSet) {
                inputNames.clear();
            }
            inputNames = inNames;

            // set up pf name: /cb/<space>/<pf_type>/<inputName[1]>/.../<inputName[N]>
            pfName = "/cb/" + pfSpace + "/" + randomIDstr + "/" + pfType;
            std::string prefixStr = "/cb/" + pfSpace;
            std::string tmp;
            int s;
            for(unsigned int i=0; i<inputNames.size(); i++) {
                tmp = inputNames[i];
                s = prefixStr.size();           
                tmp.erase(0,s);
                pfName += tmp;
            }
            
            // clear out any old input vectors or ports
            for(unsigned int i=0; i<inputPorts.size(); i++)
                delete inputPorts[i];
            inputPorts.clear();
            
            for(unsigned int i=0; i<inputs.size(); i++)
                delete inputs[i];
            inputs.clear();

            // create new input pointers
            for(unsigned int i=0; i<inputNames.size(); i++) {
                inputs.push_back(new yarp::sig::Vector(1));
                inputPorts.push_back(new yarp::os::BufferedPort<yarp::os::Bottle>());
            }                       

            // set flag
            inputsSet = true;

            // set outputs
            outputPortName = pfName + "data:o";            
            outputPort.close();
            outputPort.open(outputPortName.c_str());

            std::cout << "ControlBasisPotentialFunction::setInputs() name=" << pfName.c_str() << ", size: " << inputs.size() << std::endl;

        }
        
        /**
         * Destructor
         **/
        ~ControlBasisPotentialFunction() 
        { 

            std::cout << "ControlBasisPotentialFunction destructor..." << std::endl;

            inputNames.clear();
            inputsSet = false;

            // clear ports and data
            for(unsigned int i=0; i<inputPorts.size(); i++) {
                inputPorts[i]->close();
                delete inputPorts[i];
            }
            inputPorts.clear();

            for(unsigned int i=0; i<inputs.size(); i++){
                delete inputs[i];
            }
            inputs.clear();
            outputPort.close(); 

        }

        /**
         * returns whether the PF needs a reference input
         **/
        bool needsReference() { return hasReference; }

        /**
         * gets the space of the potential function
         **/
        std::string getSpace() { return pfSpace; }
        
        /**
         * gets the type of the potential function (the name representing the type of function. e.g., squared_error_pf)
         **/
        std::string getType() { return pfType; }
       
    };


    /**
     * a helper class that stores the identification information concerning 
     * control basis potential functions.
     **/
    class PotentialFunctionInfo {

    public:

        PotentialFunctionInfo() { }

        PotentialFunctionInfo(std::string name, std::string space, bool hasReference) :
            name(name),
            space(space),
            hasReference(hasReference)
        {
        }
        
        ~PotentialFunctionInfo() {}

        /**
         * the name of the PF
         **/
        std::string name;

        /**
         * the space the PF computeds its value in
         **/
        std::string space;

        /**
         * whether the PF needs a reference (to compute an error from)
         **/
        bool hasReference;

    };

    
}

#endif
