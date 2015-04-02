/* 
 * Copyright (C) 2012 Istituto Italiano di Tecnologia
 * Author:  Elena Ceseracciu
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef _DMP_INTERFACE_H_

#define _DMP_INTERFACE_H_

#include <vector>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Bottle.h>
#include <iostream>

#include <iCub/DMP/Trajectory.h>
#define DMP_EPSILON 1e-16

yarp::sig::Vector derive(const yarp::sig::Vector& vec, const yarp::sig::Vector& time);
namespace dmp{
    
    class DMP; //forward declaration


class DMPCanonicalState
{
public:
    //DMPCanonicalState(){};
    virtual ~DMPCanonicalState();
     /**
     * Reset to initial value
     */
    virtual void reset()=0;
    virtual DMPCanonicalState* clone() const = 0;  
};

class DMPCanonical
{
protected:
    DMPCanonicalState *state_;
    virtual void print(std::ostream& os) const =0;
public:
    //DMPCanonical(){};
    virtual ~DMPCanonical();
    virtual DMPCanonical* clone() const = 0;
    
    /**
     * Integrate the canonical function to update internal canonical state
     * @param[in] dt integration step
     * @return true if successful
     */
    virtual bool integrate(double dt)=0;
    
     /**
     * Reset the internal canonical state to initial value
     * @return true if successful
     */
    virtual void reset()=0;
    /**
     * Ask the pointer to the internal canonical state (non-const version)
     * @return pointer to internal canonical state
     */
    DMPCanonicalState* getCanonicalState() {return state_;};
    
     /**
     * Ask the pointer to the internal canonical state (const version)
     * @return const pointer to internal canonical state
     */
    const DMPCanonicalState* getCanonicalState() const {return dynamic_cast<const DMPCanonicalState*>(state_);};
   // virtual bool askStateAtTime(double time, dmp::DMPCanonicalState* state) const =0;
    
     /**
     * Query the canonical function for the canonical state that corresponds to a given time instant 
     * @param[in] time time instant for which the state is queried
     * @return pointer to newly created canonical state (must be deallocated by caller function)
     */
    virtual  dmp::DMPCanonicalState* askStateAtTime(double time) const =0;
    
     /**
     * Estimate time-scaling parameters that define the canonical function from an example trajectory
     * @param[in] trajectory example trajectory
     * @return true if successful
     */
    virtual bool estimate_time_constants(const Trajectory& trajectory)=0;
    
    friend std::ostream& operator<<(std::ostream& os, const DMPCanonical& myDMPCan)
    {
        myDMPCan.print(os);
        return os;
    };
  //  virtual bool isValid() const =0;
    /**
   * Ask the constant parameters that describe the equations for this canonical system (i.e., time constants)
   * @return vector of parameters
   */
    virtual yarp::sig::Vector get_constants() const = 0;
    
    /**
   * Ask the numerosity of constant parameters that describe the equations for this canonical system (i.e., time constants)
   * @return number of constant parameters
   */
    virtual size_t get_number_of_constants() const = 0;
    
    /**
   * Set the constant parameters that describe the equations for this canonical system (i.e., time constants)
   * @param[in] newConsts constants vector
   * @return true if successful
   */
    virtual bool set_constants(const yarp::sig::Vector& newConsts)=0;
    /**
   * Ask the numerosity of variable parameters that identify a particular canonical system (i.e., expected time duration of the trajectory)
   * @return number of variable parameters
   */
    virtual size_t get_number_of_variables() const = 0;
    
    /**
   * Ask the variable parameters that identify a particular canonical system (i.e., expected time duration of the trajectory)
   * @return vector of variable parameters
   */
    virtual yarp::sig::Vector get_variables() const = 0;
   
   /**
   * Set the variable parameters that identify a particular canonical system (i.e., expected time duration of the trajectory)
   * @param[in] newVars variables vector
   * @return true if successful
   */
    virtual bool set_variables(const yarp::sig::Vector& newVars)=0;

};

class BasisFunction
{
protected:
    virtual void print(std::ostream& os) const =0;
public:
    virtual BasisFunction* clone() const = 0;
    /** 
    * Calculate value of basis function at a given state of the canonical system
    * @param[in] canonicalState pointer to the DMPCanonicalState that 
    */
    virtual double calculate(const DMPCanonicalState* canonicalState)=0;
    friend std::ostream& operator<<(std::ostream& os, const BasisFunction& myBF)
    {
        myBF.print(os);
        return os;
    };
};

class TransformationSystem
{   
protected:
    double y; //position
    double z; //scaled velocity
    double attractor;
    std::vector<BasisFunction*> basisFunctions;
    
  /**
   * Estimate the attractor from an example trajectory
   * @param[in] trajectory example trajectory (only first degree of freedom is considered)
   * @return true if successful
   */
    virtual bool estimate_attractor(const Trajectory& trajectory)=0;
    
  /**
   * Helper function for parameters estimation - calculates the value of the "known" part of the perturbation function at the state calculated with a given canonical function for a given time instant.
   * The values X thus found can be used to calculate the weights w for the DMP, by solving X*w=f_arg, where the values f_arg can be found with the calculate_output function.
   * @param[in] time time instant for which basis function values must be calculated
   * @param[in] canonicalFunction function that allows to convert the time instant into phase space.
   * @return true if successful
   */
    virtual yarp::sig::Vector calculate_all_basis_functions_at_time(double time, const DMPCanonical* canonicalFunction);
    
  /**
   * Helper function for parameters estimation - calculates the value of the unperturbed second-order differential equation for measured position, velocity, acceleration.
   * The value f_arg thus found can be used to calculate the weights w for the DMP, by solving X*w=f_arg, where the values of X can be found with the calculate_all_basis_functions_at_time function.
   * @param[in] position measured position for a trajectory sample
   * @param[in] velocity measured velocity for a trajectory sample
   * @param[in] acceleration measured acceleration for a trajectory sample
   * @return value of the unperturbed second-order differential equation 
   */
    virtual double calculate_output(double position, double velocity, double acceleration)=0;
   
   /**
   * Function that allows to optionally modify the value of the perturbation function, starting from the weighted sum of basis functions
   * @param[in,out] fx value of the perturbation function
   * @param[in] canonicalState pointer to the canonical state at which the perturbation function is being calculated
   * @return value of the unperturbed second-order differential equation 
   */
    virtual bool normalize_fx(double& fx, const DMPCanonicalState* canonicalState){return true;};
    
   /**
   * Function that allows to optionally modify the value of the perturbation function, starting from the weighted sum of basis functions
   * @param[in,out] fx value of the perturbation function
   * @param[in] canonicalState pointer to the canonical state at which the perturbation function is being calculated
   * @return value of the unperturbed second-order differential equation 
   */
    virtual bool scale(double& fx){return true;};
    
    /**
   * Integrate the differential equations, using the provided value for the perturbation function.
   * @param[in] fx value of the perturbation function
   * @param[in] dt integration step 
   * @return true if internal position and velocity have been successfully updated
   */
    virtual bool integrate_output_system(double dt, double fx)=0;
    virtual void print(std::ostream& os) const =0;   
    friend std::ostream& operator<<(std::ostream& os, const TransformationSystem& myTS)
    {
        myTS.print(os);
        return os;
    };
public:
    /** 
    * Default constructor
    */
    TransformationSystem();
    virtual ~TransformationSystem();
    /** 
    * Default constructor
    */
    TransformationSystem(const TransformationSystem& original);
    virtual TransformationSystem* clone() const = 0;
    
    /** 
    * Ask current value for the position
    * @return position
    */
    virtual double get_position() const;
    
    /** 
    * Set current value for the position
    * @param[in] newPos new value for position
    */
    virtual void set_position(double newPos);
    
    /** 
    * Ask current value for the velocity
    * @return velocity
    */
    virtual double get_velocity() const;
    
    /** 
    * Set current value for the velocity
    * @param[in] newVel new value for velocity
    */
    virtual void set_velocity(double newVel);
    
    /** 
    * Ask value of attractor
    * @return attractor
    */
    virtual double get_attractor() const;
    
    /** 
    * Set current value for the attractor
    * @param[in] newAttr new value for attractor
    */
    virtual void set_attractor(double newAttr);

    /** 
    *  Get the number of basis functions employed in the perturbation function
    * @return number of banewAttr new value for attractor
    */
    virtual size_t get_number_of_basis_functions() const =0;
    /** 
    * If needed, set TransformationSystem's own time constants based on those of the canonical function
    * @param[in] canonicalFunction pointer to the canonical function that can provide the time constants
    */
    virtual bool set_time_constants(const DMPCanonical* canonicalFunction)=0;
        
    /**
   * Integrate the dynamical equation by a step dt (Euler integration), calculating the perturbation function from the provided canonical state
   * @param[in] canonicalStateFunction pointer to the canonicalState that allows to calculate the perturbation function
   * @param[in] dt time lenght of the integration step (a single step is performed)
   * @return true if successful
   */
    virtual bool integrate(const DMPCanonicalState* canonicalState, double dt);
    
   /**
   * Estimate DMP parameters from an example trajectory, resorting to the provided canonical function to convert the temporal dimension to phase space.
   * @param[in] trajectory example trajectory (only first degree of freedom is considered)
   * @param[in] canonicalFunction pointer to canonicalFunction that can be queried to obtain the state the corresponds to each time instant of the original trajectory
   * @return true if successful
   */
    virtual bool learn(const dmp::Trajectory& trajectory, const DMPCanonical* canonicalFunction);
    
   /**
   * Ask the numerosity of constant parameters that describe the equations for this transformation system (i.e., time constants)
   * @return number of constant parameters
   */
    virtual size_t get_number_of_constants() const = 0;
    
   /**
   * Ask the constant parameters that describe the equations for this transformation system (i.e., time constants)
   * @return vector of parameters
   */
    virtual yarp::sig::Vector get_constants() const = 0;
    
   /**
   * Set the constant parameters that describe the equations for this transformation system  (i.e., time constants)
   * @param[in] newConsts constants vector
   * @return true if successful
   */
    virtual bool set_constants(const yarp::sig::Vector& newConsts)=0;
    
   /**
   * Ask the numerosity of variable parameters that identify a particular transformation system (i.e., weights & attractor)
   * @return number of variable parameters
   */
    virtual size_t get_number_of_variables() const = 0;
    
    /**
   * Ask the variable parameters that identify a particular transformation system (i.e., weights & attractor)
   * @return vector of variable parameters
   */
    virtual yarp::sig::Vector get_variables() const = 0;
   
   /**
   * Set the variable parameters that identify a particular transformation system (i.e., weights & attractor)
   * @param[in] newVars variables vector
   * @return true if successful
   */
    virtual bool set_variables(const yarp::sig::Vector& newVars)=0;
    
   /**
   * Set the weights that characterize the perturbation function for this transformation system
   * @param[in] weights new weights for the perturbation function (length must correspond to the number of basis functions)
   * @return true if successful
   */
    virtual bool set_weights(const yarp::sig::Vector& weights)=0;
    
    /**
   * Ask the weights that characterize the perturbation function for this transformation system
   * @return vector of weights (length is the number of basis functions)
   */
    virtual yarp::sig::Vector get_weights() const =0;
    
    /**
   * Ask the weight that characterize the contribution of a particular basis function to the perturbation function for this transformation system
   * @param[in] indBF index of basis function for which the weight is queried
   * @return weight for a basis function
   */
    virtual double get_weight(size_t indBF) const
    {
        if (indBF >=get_number_of_basis_functions())
            return 0.0;
        return get_weights()[indBF];
    };
    
  //  friend class dmp::DMP;
};

/**
 * Abstract class that defines Dynamic Movement Primitives (DMP) as the conjunction of a canonical system and a set of transformation systems.
 * The first describes the evolution of the state of the DMP in time, while the latter relate this state to the value for each degree of freedom of the represented trajectory.
 * 
 */
class DMP
{
protected:
    bool valid;
    bool verbose;

    double integrationStep;
    dmp::DMPCanonical *canonicalSystem_;

    size_t dof_;
    std::vector<dmp::TransformationSystem*> transformations_;

    virtual void print(std::ostream& os) const =0;
    
    DMP(size_t dof=0);
    
    /**
    * Set the weights for the perturbation function of one degree of freedom
    * @param[in] indTS index of the degree of freedom (transformation system) to be modified
    * @param[in] newWeights weights vector (lenght is the number of basis functions)
    * @return true if successful
    */
    bool set_weights(size_t indTS, const yarp::sig::Vector& newWeights);
    
    /**
    * Get the weights for the perturbation function of one degree of freedom
    * @param[in] indTS index of the degree of freedom (transformation system) to access
    * @return weights vector (lenght is the number of basis functions)
    */
    yarp::sig::Vector get_weights(size_t indTS) const;
    
public:
    
    DMP(const DMP& original);
    virtual ~DMP();
    virtual DMP* clone() const = 0;

   /**
   * Integrate the dynamical equations.
   * @param[in] dt length of integration window (may correspond to multiple actual integration step)
   * @return true if successful
   */
    bool integrate(double dt);
    
   /**
   * Estimate DMP parameters from an example trajectory
   * @param[in] trajectory example trajectory
   * @return true if successful
   */
    bool estimate_from_trajectory(const Trajectory& trajectory);
    
   /**
   * Change verbosity option
   * @param[in] on true to enable verbosity
   */
    void setVerbose(bool on){verbose=on;};
    
   /**
   * Reset the state of the canonical system of the DMP 
   * @param[in] on true to enable verbosity
   */
    void resetState();
    
   /**
   * Ask the number of degrees of freedom (transformation systems) that are represented 
   * @return number of degrees of freedom
   */
    size_t get_number_of_dofs() const;
    
   /**
   * Return the current positions
   * @return positions vector (lenght is the number of degrees of freedom)
   */
    yarp::sig::Vector get_positions() const;
    
   /**
   * Return the current velocities
   * @return velocities vector (lenght is the number of degrees of freedom)
   */
    yarp::sig::Vector get_velocities() const;
    
   /**
   * Return the attractor
   * @return attractor vector (lenght is the number of degrees of freedom)
   */
    yarp::sig::Vector get_attractor() const;
    
   /**
   * Set the current positions
   * @param[in] newPos positions vector (lenght is the number of degrees of freedom)
   * @return true if successful (lenght of provided vector was consistent with the DMP number of degrees of freedom)
   */
    bool set_positions(const yarp::sig::Vector& newPos);
    
   /**
   * Set the attractor
   * @param[in] newAttractor attractor vector (lenght is the number of degrees of freedom)
   * @return true if successful (lenght of provided vector was consistent with the DMP number of degrees of freedom)
   */
    bool set_attractor(const yarp::sig::Vector& newAttractor);
    
   /**
   * Set the current velocities
   * @param[in] newVel velocities vector (lenght is the number of degrees of freedom)
   * @return true if successful (lenght of provided vector was consistent with the DMP number of degrees of freedom)
   */
    bool set_velocities(const yarp::sig::Vector& newVel);
    
   /**
   * Ask the constant parameters that describe the DMP equations (i.e., time constants)
   * @return vector of parameters
   */
    virtual yarp::sig::Vector get_DMP_constants() const;
    
   /**
   * Set the constant parameters that describe the DMP equations (i.e., time constants)
   * @param[in] newConsts constants vector
   * @return true if successful
   */
    virtual bool set_DMP_constants(const yarp::sig::Vector & newConsts);
   /**
   * Ask the variable parameters that identify a particular DMP (i.e., weights & attractor)
   * @return vector of parameters
   */
    virtual yarp::sig::Vector get_DMP_variables() const;
    
   /**
   * Set the variable parameters  that identify a particular DMP (i.e., weights & attractor)
   * @param[in] newVars variables vector
   * @return true if successful
   */
    virtual bool set_DMP_variables(const yarp::sig::Vector& newVars);
    
   /**
   * Ask the type name of the DMP
   * @return name of DMP type
   */
    virtual std::string getType() const =0;
    
   /**
   * Ask the length of actual integration step
   * @return length of integration step
   */
    double get_integration_step() const
    {
        return integrationStep;
    }
    
    /**
   * Set the length of actual integration step
   * @param[in] integrationStep new length of integration step 
   */
    void set_integration_step(double integrationStep)
    {
        this->integrationStep=integrationStep;
    }
    
   /**
   * Check whether the DMP is valid (construction performed correctly; useful when constructing from Searchable)
   * @return true if DMP is valid
   */
    virtual bool isValid() const {return valid;};
    
   /**
   * Serialize DMP as a YARP Bottle
   * @return bottle describing the DMP
   */
    virtual yarp::os::Bottle toBottle() const =0;
    
   /**
   * Serialize DMP as a YARP Bottle
   * @return bottle describing the DMP
   */
    virtual bool fromBottle(const yarp::os::Bottle& bot)=0;
    
    friend std::ostream& operator<<(std::ostream& os, const DMP& myDmp)
    {
        myDmp.print(os);
        return os;
    };
    
};
}

#endif