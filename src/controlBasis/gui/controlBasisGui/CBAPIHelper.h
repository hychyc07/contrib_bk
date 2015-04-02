// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CBAPI_HELPER__H_
#define _CBAPI_HELPER__H_

#include <controlBasis/RunnableControlLaw.h>
#include <controlBasis/ControllerParameters.h>

namespace CB {

    /**
     * This class is a helper class for the CBAPI that stores the current control law.
     * You can start, stop, add, delete, and run controllers (adding them to a law in 
     * order of decreasing priority.  Can get their state and dyanmics too.
     **/ 
    class CBAPIHelper {

    protected:
        
        /**
         * The runnable control law class to add controllers to.
         **/
        RunnableControlLaw * controlLaw;

        /**
         * The runnable control law class to add controllers to (for the sequence).
         **/
        RunnableControlLaw * sequenceControlLaw;
        
        /**
         * The number of controllers in the law currently.
         **/
        int numControllersInLaw;

        /**
         * The number of controllers in the sequence
         **/
        int numControllersInSequence;

        /**
         * the list of controllers in the sequence
         **/
        std::vector<ControllerParameters *> sequenceControllerParameters[2];
        
        /**
         * the index into the sequence that is currently running
         **/
        int sequenceIndex;

        /**
         * determines whether a derivative term is used in the control laws
         **/
        bool useDerivativeTerm;

        /**
         * determines whether an integral term is used in the control laws
         **/
        bool useIntegralTerm;

    public:
        
        /**
         * Constructor
         **/

        CBAPIHelper();

        /**
         * Destuctor
         **/
        ~CBAPIHelper() { 
            clearControlLaw();
            clearSequence();

            delete controlLaw;
            delete sequenceControlLaw;
        }
      
        /**
         * Adds a controller to the law with the resources specified.
         * \param sen the sensor name
         * \param ref the reference (target) name, if there is one
         * \param pf the name of the potential function
         * \param eff the effector name
         * \param useTranspose  if true, uses J^T in the contrl law, if false, uses J^# (moore-penrose pseudoinverse)
         * \param gain_p the P-gain for the controller
         * \param gain_i the I-gain for the controller
         * \param gain-d the D-gain for the controller
         **/
        void addControllerToLaw(std::string sen, std::string ref, std::string pf, std::string eff, bool useTranspose, double gain_p=1, double gain_i=0, double gain_d=0);

        /**
         * Adds a controller to the sequence with the resources specified.
         * \param sen the sensor name
         * \param ref the reference (target) name, if there is one
         * \param pf the name of the potential function
         * \param eff the effector name
         * \param useTranspose  if true, uses J^T in the contrl law, if false, uses J^# (moore-penrose pseudoinverse)
         * \param gain_p the P-gain for the controller
         * \param gain_i the I-gain for the controller
         * \param gain-d the D-gain for the controller
         **/
        void addControllerToSequence(std::string sen, std::string ref, std::string pf, std::string eff, bool useTranspose, double gain_p=1, double gain_i=0, double gain_d=0);

        /**
         * Adds a lower-priority (secondary) controller to the sequence with the resources specified.
         * \param sen the sensor name
         * \param ref the reference (target) name, if there is one
         * \param pf the name of the potential function
         * \param eff the effector name
         * \param useTranspose  if true, uses J^T in the contrl law, if false, uses J^# (moore-penrose pseudoinverse)
         * \param gain_p the P-gain for the controller
         * \param gain_i the I-gain for the controller
         * \param gain-d the D-gain for the controller
         * \return returns whether it was added (only two objectives are allowed)
         **/
        bool addSecondaryControllerToSequence(std::string sen, std::string ref, std::string pf, std::string eff, bool useTranspose, double gain_p=1, double gain_i=0, double gain_d=0);

        /**
         * clears the control law
         **/
        void clearControlLaw();

        /**
         * clears the controller sequence
         **/
        void clearSequence();

        /**
         * stops the control law
         **/
        void stopControlLaw();

        /**
         * pause the control law
         **/
        void pauseControlLaw();

        /**
         * stops the controller sequence
         **/
        void stopSequence();

        /**
         * pause the controller sequence
         **/
        void pauseSequence();

        /**
         * runs the control law
         **/
        void runControlLaw();

        /**
         * runs the controller sequence
         **/
        void runSequence();
        
        /**
         * gets the number of controllers in the law of sequence
         **/
        int getNumControllers(bool sequence=false) { 
            if(sequence) {
                return numControllersInSequence; 
            } else {
                return numControllersInLaw; 
            }
        }
        
        /**
         * gets the potential of controller n
         **/
        double getPotential(int n, bool sequence);

        /**
         * gets the estimated change in potential of controller dot
         **/
        double getPotentialDot(int n, bool sequence);

        /**
         * gets the state of controller n
         **/
        int getState(int n, bool sequence);
        
        /**
         * sets whether the controller is using the transpose of the Jacobian or the Moore-Penros pseudoinverse
         **/
        void useTranspose(bool b);

        /**
         * sets whether the controller will be a PD controller or a P controller 
         **/
        void usePDControl(bool b);

        /**
         * sets whether the controller will be a PID controller or a P controller 
         **/
        void usePIDControl(bool b);
      
        /**
         * gets the current sequence controller id if that is running
         **/
        int getSequenceControllerID();

        /**
         * gets the number of controllers (objectives) in the current sequence step
         **/
        int getNumberOfControllersInSequenceStep();

        /**
         * starts the next controller in the sequence
         **/
        void goToNextControllerInSequence();

        /**
         * starts the previous controller in the sequence
         **/
        void goToPreviousControllerInSequence();

        /**
         * saves the controller output to a file
         **/
        void saveControllerOutput(std::string f);

        /**
         * sets the controller gains
         **/
        void setGains(double kp, double ki, double kd);

        /**
         * sets the nullspace scaling factor
         **/
        void setNullspaceFactor(double n);

        /**
         * returns whether any of the controllers in the law have finished starting
        **/
        bool controlLawStarted(bool sequence);


  };

}

#endif
