// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
/**
 * This is a simple helper class that enumertes the resource names to be used
 * in a control basis controller.  The resources include a sensor value,
 * a reference value (that is compared to the sensor value), a potential function
 * that evaluates a positive scalar "potential" value based on the sensor and
 * reference signals, and an effector that can be moved to reduce the potential
 * value.
 **/
#ifndef _CONTROLLER_PARAMETERS__H_
#define _CONTROLLER_PARAMETERS__H_

#include <iostream>

namespace CB {
    
    class ControllerParameters {
    
    public:
        
        /**
         * the sensor string
         **/
        std::string Sensor;
        
        /**
         * the reference string (leave as "" if there is no reference)
         **/
        std::string Reference;
        
        /**
         * the effector string
         **/
        std::string Effector;
        
        /**
         * the potential function string
         **/
        std::string PotentialFunction;
        
        /**
         * a (proportional) gain for the controller output
         **/
        double Gain_P;

        /**
         * an (integral) gain for the controller output
         **/
        double Gain_I;

        /**
         * a (derivative) gain for the controller output
         **/
        double Gain_D;
        
        /**
         * Constructor
         **/
        ControllerParameters() {
            Sensor = "";
            Effector = "";
            Reference = "";
            PotentialFunction = "";
            Gain_P = 1.0;
            Gain_I = 0.0;
            Gain_D = 0.0;
        }
        
        ControllerParameters(std::string s, std::string r, std::string pf, std::string e, double gain_p=1.0, double gain_i=0.0, double gain_d=0.0) {
            Sensor = s;
            Effector = e;
            Reference = r;
            PotentialFunction = pf;
            Gain_P = gain_p;
            Gain_I = gain_i;
            Gain_D = gain_d;
        }

        ControllerParameters(std::string s, std::string pf, std::string e, double gain_p=1.0, double gain_i=0.0, double gain_d=0.0) {
            Sensor = s;
            Effector = e;
            Reference = "";
            PotentialFunction = pf;
            Gain_P = gain_p;
            Gain_I = gain_i;
            Gain_D = gain_d;
        }
        
        /** 
         * Destructor
         **/
        ~ControllerParameters() { }
        
        /**
         * display function
         **/
        void display() {
            std::cout << std::endl << std::endl;
            std::cout << "Controller Info:" << std::endl << "------------------" << std::endl;
            std::cout << "Sensor: " << Sensor.c_str() << std::endl;
            if(Reference != "")  std::cout << "Reference: " << Reference.c_str() << std::endl;
            std::cout << "Potential Function: " << PotentialFunction.c_str() << std::endl;
            std::cout << "Effector: " << Effector.c_str() << std::endl;
            std::cout << "Gains: (KP=" << Gain_P << ", KI=" << Gain_I << ", KD=" << Gain_D << ")" << std::endl << std::endl;
        }
        
    };
    
}

#endif
