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

#ifndef _DMP_PERIODIC_H_

#define _DMP_PERIODIC_H_

#include <string>

#include <fstream>
#include <sstream>
#include <iostream>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include "iCub/DMP/DMP.h"

 double average(const yarp::sig::Vector& vector);
//#define DMP_CANONICAL_THRESHOLD 1e-5

class DMPCanonicalStatePhi: public dmp::DMPCanonicalState
{
    double phi_;
public:
    DMPCanonicalStatePhi();
    DMPCanonicalStatePhi(double phi);
    DMPCanonicalStatePhi(const DMPCanonicalStatePhi& original);
    virtual DMPCanonicalStatePhi* clone() const;
    virtual void reset();
    double get_phi() const;
    void set_phi(double phi);
};

class DMPCanonicalPeriodic : public dmp::DMPCanonical{
protected:
    DMPCanonicalStatePhi phase_;
    double omega_;
    
    virtual void print(std::ostream& os) const;
    
public:
    DMPCanonicalPeriodic();
    DMPCanonicalPeriodic(double omega);
    ~DMPCanonicalPeriodic() {};
    DMPCanonicalPeriodic(const DMPCanonicalPeriodic& original);
    virtual DMPCanonicalPeriodic* clone() const;
    void set_omega(double omega);
    double get_omega() const;
    double get_phase() const;
    virtual dmp::DMPCanonicalState* askStateAtTime(double time) const;
    virtual bool estimate_time_constants(const dmp::Trajectory& trajectory);
    virtual bool integrate(double dt);
    virtual void reset();
    
    virtual size_t get_number_of_constants() const;
    
    virtual yarp::sig::Vector get_constants() const;
    virtual bool set_constants(const yarp::sig::Vector& newConsts);
    
    virtual size_t get_number_of_variables() const;
    virtual yarp::sig::Vector get_variables() const;
    virtual bool set_variables(const yarp::sig::Vector& newVars);
};

class PeriodicBasisFunction : public dmp::BasisFunction
{
protected:
    double c; //center
    double sigma2; //variance
    virtual void print(std::ostream& os) const
    {
        os << "Center: " << c << ", variance: " << sigma2 << ".";
    };
public:
    PeriodicBasisFunction(double center, double variance): c(center), sigma2(variance)
    {};
    PeriodicBasisFunction(const PeriodicBasisFunction& original): c(original.c), sigma2(original.sigma2) {};
    virtual PeriodicBasisFunction* clone() const
    {
        return new PeriodicBasisFunction(*this);
    };
    double get_center() const {return c;}
    double get_variance() const {return sigma2;}
    virtual double calculate(const dmp::DMPCanonicalState* canonicalState);
    
};

class TransformationSystemPeriodic : public dmp::TransformationSystem 
{
//     double z;
//     double y;
    
    double alpha_z;
    double beta_z;

    double omega;
    yarp::sig::Vector w;

    size_t N;
   // double attractor;
    bool scaling;
   // double epsilon;
    
    void define_basis_functions(double factor);
    
    
    virtual bool estimate_attractor(const dmp::Trajectory& trajectory);

    virtual bool normalize_fx(double& fx, const dmp::DMPCanonicalState* canonicalState);
    virtual bool scale(double& fx);
    virtual bool integrate_output_system(double dt, double fx);
    
    virtual size_t get_number_of_constants() const;
    
    virtual yarp::sig::Vector get_constants() const;
    virtual bool set_constants(const yarp::sig::Vector& newConsts);
    
    virtual size_t get_number_of_variables() const;
    virtual yarp::sig::Vector get_variables() const;
    virtual bool set_variables(const yarp::sig::Vector& newVars);

    virtual double calculate_output(double position, double velocity, double acceleration);
    virtual bool set_weights(const yarp::sig::Vector& weights);

    virtual yarp::sig::Vector get_weights() const;
    
    virtual void print(std::ostream& os) const;

public:
    TransformationSystemPeriodic(){};
     ~TransformationSystemPeriodic(){};
    TransformationSystemPeriodic(yarp::os::Property properties);
    TransformationSystemPeriodic( double alpha_z, double beta_z, int N);
    
    TransformationSystemPeriodic(const TransformationSystemPeriodic& original);
    virtual TransformationSystemPeriodic* clone() const;
        
    double get_alphaz() const;
    double get_betaz() const;
    virtual bool set_time_constants(const dmp::DMPCanonical* canonicalFunction);

    size_t get_number_of_basis_functions() const;    

};

/**
* Dynamic Motion Primitive representation for a trajectory
*/
class DMPPeriodic: public dmp::DMP {
protected:
   // bool valid;
    virtual void print(std::ostream& os) const;
    virtual bool fromSearchable(const yarp::os::Searchable* property);
public:
    //DMPPeriodic();
    /**
    * Constructor.
    *
    * @param[in] dof degrees of freedom of original trajectories 
    * @param[in] N number of basis functions
    * @param[in] alpha_z alpha_x time constant in DMP dynamic system
    * @param[in] beta_z beta_z time constant in DMP dynamic system
    * 
    */
    DMPPeriodic(int dof, int N, double alpha_z, double beta_z);
    DMPPeriodic(const yarp::os::Property& properties);
    DMPPeriodic(const DMPPeriodic& original);
    virtual ~DMPPeriodic();
    
    virtual DMPPeriodic* clone() const;
    
    void set_omega(double omega);
    
    virtual std::string getType() const {return "DMPPeriodic";};
    
    virtual yarp::os::Bottle toBottle() const;
    virtual bool fromBottle(const yarp::os::Bottle& bot);
 
};

#endif
