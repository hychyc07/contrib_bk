// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _CONTROLLER__H_
#define _CONTROLLER__H_

#include "ControlBasisAction.h"
#include "ControlBasisResource.h"
#include "ControlBasisJacobian.h"
#include "ControlBasisPotentialFunction.h"

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Port.h>
#include <vector>

#include "JacobianFactory.h"
#include "PotentialFunctionFactory.h"

/**
 * This class instantiates the abstract ControlBasisAction class 
 * for a primitive Controller.  It implements the control law
 * \delta \tau = J^# \phi(\sigma) \kappa, where \tau is the effector
 * resource (e.g., motor variables), \sigma is the input sensor 
 * resource information, and \phi is the control (potential) function.
 * The task Jacobian J captures sensitive of the potential evaluated
 * at the sensory values w.r.t., the output variables. that is,
 * J = \frac{ \partial \phi(\sigma) }{ \partial \tau}.  Also supports 
 * using J^T in place of J^# and using Intergral and Derivative terms
 * in addition to just the Proportional term.
 *
 * Controllers are strongly typed entities.  The input sensory space will
 * have a type (or domain, such as "Cartesian Position," etc.) that must
 * match the space of the potential function.  Furthermore, there must
 * be a Jacobian that maps the input space to the output (effector) space, 
 * if they are not the same.  It is possible to have output spaces that are
 * not directly controllabe degrees of freedom.  If this is the case,
 * the RunnableControlLaw will have to turn this abstract output into an
 * actual motor command. See the RunnableControlLaw class for more 
 * information considering this particular case.
 *
 * A controller can be run in either "local" ir "distributed" mode. In local mode,
 * it connects directly to ControlBasisResource classes specified in the
 * constructor.  If it runs in distributed mode, it connects to these resources
 * over YARP ports, and the names of these ports must be specified in the
 * constructor.
 **/
namespace CB {
    
    class Controller : public ControlBasisAction {
        
    protected:
    
        /**
         * inputSpace
         **/
        std::string inputSpace;
        
        /**
         * outputSpace
         **/
        std::string outputSpace;
        
        /**
         * potentialFunction
         **/
        std::string pfType;
    
        /**
         * control output
         **/
        yarp::sig::Vector Vout;

        /**
         * current effector value
         **/
        yarp::sig::Vector Veff;

        /**
         * Potential
         **/
        double potential;

        /**
         * potential (last)
         **/
        double potentialLast;
    
        /**
         * Potential Dot
         **/
        double potentialDot;
        
        /**
         * whether an internal jacobian is necessary
         **/
        bool needsJacobian;

        /**
         * whether the inverse of the internal jacobian is needed
         **/
        bool needsJacobianInverse;
        
        /**
         * whether there is a reference sensory signal
         **/
        bool hasReference;

        /**
         * Controller (Task) Jacobian
         **/
        yarp::sig::Matrix Jc;
        
        /**
         * Controller Jacobian Inverse
         **/
        yarp::sig::Matrix JcInv;

        /**
         * a pointer to the sensor resource
         **/
        ControlBasisResource *sensor;

        /**
         * a pointer to the reference sensor resource (if necessary)
         **/
        ControlBasisResource *reference;

        /**
         * a pointer to the effector resource
         **/
        ControlBasisResource *effector;

        /**
         * a pointer to the potential function
         **/
        ControlBasisPotentialFunction *potentialFunction;

        /**
         * a pointer to the Jacobian
         **/
        ControlBasisJacobian *jacobian;

        /**
         * control gain (proportional)
         **/
        double gain_p;    

        /**
         * control gain (integral)
         **/
        double gain_i;    

        /**
         * control gain (derivative)
         **/
        double gain_d;    

        /**
         *  the name of the sensor
         **/
        std::string sensorName;
        
        /**
         *  the name of the sensor reference
         **/
        std::string referenceName;

        /**
         *  the name of the effector 
         **/
        std::string effectorName;

        /**
         * the name of the potential function
         **/
        std::string pfName;

        /**
         * flag for distributed sensor mode
         */
        bool distributedMode;

        /**
         * the name of the device the controller will control
         **/
        std::string deviceName;

        /**
         * storage for the potential as the controller runs
         */
        std::vector<double> potentialStore;

        /**
         * storage for the estimated potential derivative as the controller runs
         */
        std::vector<double> potentialDotStore;       

        /**
         * using transpose of Jacobian, rather than pseudoinverse, s.t. \delta \tau = -J^T \phi(\sigma)
         **/
        bool useJacobianTranspose;        

        /**
         * using a derivative term in the control law (in addition to a proportional term)
         **/
        bool useDerivativeTerm;

        /**
         * using an integral term in the control law (in addition to a proportional  term)
         **/
        bool useIntegralTerm;

        /**
         * stores the bias towards convergence based on history
         **/
        double convergenceStore;

        /**
         * stores the previous convergence state
         **/
        double convergenceStoreLast;

        /**
         * stores the start time of the controller
         **/
        double t0;

        /**
         * stores the time of the last iteration of the controller
         **/
        double t_last;

        /**
         * stores an estimate of dt
         **/
        double dt;

        /**
         * stores the estimated integral 
         **/
        double errInt;

    public:
        
        /** 
         * constructor for local set up (with reference)
         **/
        Controller(ControlBasisResource *sen,
                   ControlBasisResource *ref,
                   ControlBasisPotentialFunction *pf,
                   ControlBasisResource *eff);

        /** 
         * constructor for local set up (without reference)
         **/
        Controller(ControlBasisResource *sen,
                   ControlBasisPotentialFunction *pf,
                   ControlBasisResource *eff);

        /**
         * constructor for remote set up (with reference sensor)
         **/
        Controller(std::string sen, std::string ref, std::string pf, std::string eff);

        /**
         * constructor for remote set up (without reference)
         **/
        Controller(std::string sen, std::string pf, std::string eff);
    
        /** 
         * destructor
         **/
        ~Controller() { resetController(); }       
        
        /**
         * Inherited update fuction from ControlBasisAction class
         **/
        virtual bool updateAction();

        /**
         * Inherited start fuction from ControlBasisAction class
         **/
        virtual void startAction();

        /**
         * Inherited stop fuction from ControlBasisAction class
         **/
        virtual void stopAction();

        /**
         * Inherited pause fuction from ControlBasisAction class
         **/
        virtual void pauseAction();

        /**
         * Inherited postData fuction from ControlBasisAction class
         **/
        virtual void postData();  

        /**
         * reset funtion
         **/
        void resetController();  

        /**
         * gets the device name of the effector this controller moves
         * \returns deviceName
         **/
        std::string getOutputDeviceName() { return deviceName; }

        /**
         * gets the space of the effector 
         * \returns outputSpace
         **/
        std::string getOutputSpace() { return outputSpace; }

        /**
         * gets the size of the effector signal 
         * \returns outputSize
         **/
        int getOutputSize() { return Vout.size(); }

        /**
         * gets the controller output \delta \tau = J^# \phi(\sigma)*\kappa         
         * \returns Vout the delta output
         **/
        yarp::sig::Vector getControlOutput() { return Vout; }

        /**
         * gets the current effector (\tau) value
         * \returns \tau
         **/
        yarp::sig::Vector getEffectorValue();

        /**
         * gets the controller's Task jacobian 
         * J = \frac{\partial \phi(\sigma)}{\partial \tau}         
         * \returns Jc the task jacobian
         **/
        yarp::sig::Matrix getTaskJacobian() { return Jc; }

        /**
         * determines if controller will be running by connecting
         * to resources over the YARP ports (distributed mode),
         * or not (local mode), where pointers to resources are 
         * maintained.
         * \return distributedMode in distributed mode
         **/
        bool inDistributedMode() { return distributedMode; }

        /**
         * sets the controller gain         
         * \param kappa_p the P gain value
         **/
        void setGainP(double kappa_p) { gain_p = kappa_p; }

        /**
         * sets the controller gain         
         * \param kappa_i the I gain value
         **/
        void setGainI(double kappa_i) { gain_i = kappa_i; }

        /**
         * sets the controller gain         
         * \param kappa_d the D gain value
         **/
        void setGainD(double kappa_d) { gain_d = kappa_d; }

        /**
         * sets the controller gain s         
         * \param kappa_p the P gain value
         * \param kappa_i the I gain value
         * \param kappa_d the D gain value
         **/
        void setGains(double kappa_p, double kappa_i, double kappa_d) { 
            gain_p = kappa_p; 
            gain_i = kappa_i; 
            gain_d = kappa_d; 
        }

        /**
         * returns current controller potential
         */
        double getControllerPotential() { return potential; }

        /**
         * returns current controller potential
         */
        double getControllerPotentialDot() { return potentialDot; }

        /** 
         * returns whether using Jacobian transpose mode in control
         **/
        bool usingJacobianTranspose() { return useJacobianTranspose; }

        /**
         * sets whether using jacobian transpose or not
         **/
        void useTranspose(bool b) { useJacobianTranspose = b; }

        /**
         * sets whether this controller is a P(I)D or a P(I) Controller 
         **/
        void useDerivativeControl(bool b) { useDerivativeTerm = b; }

        /**
         * sets whether this controller is a PI(D) or a P(D) Controller 
         **/
        void useIntegralControl(bool b) { useIntegralTerm = b; }

        /**
         * provides the phi and phi_dot for the specified index
         * &returns the size of the potential store
         **/
        int getStoredPotentialInfo(int idx, double &pot, double &pot_dot) {

            int sp = potentialStore.size();
            int sd = potentialDotStore.size();

            //std::cout << "sizes=("<<sp<<","<<sd<<")"<<std::endl;
            if(sp < sd) {
                sd = sp; 
            } else {
                sp = sd; 
            }

            if(sp==0) {
                pot = pot_dot = 0;
            } else {  
                if(idx >= sp) {
                    return -1;
                }
                pot = potentialStore[idx];
                pot_dot = potentialDotStore[idx];
            }
            return sp;

        }


    protected:

        /**
         * creates a potential function of the specified type
         **/ 
        bool createPotentialFunction(std::string pf);

        /**
         * creates the necessary Jacobian to transform potential changes to effector changes.
         **/
        bool createJacobian();

        /**
         * determines the output device and space of effector resource
         **/
        void parseOutputResource();

        /**
         * deletes the resource pointers
         **/
        void deleteResources();

        /**
         * stops the resources 
         **/
        void stopResources();
  };
 

}

#endif
