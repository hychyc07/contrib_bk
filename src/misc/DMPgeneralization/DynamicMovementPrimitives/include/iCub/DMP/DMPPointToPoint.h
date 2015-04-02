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

/*
 * 
 */

#ifndef _DMP_POINT_TO_POINT_H_

#define _DMP_POINT_TO_POINT_H_

#include <string>

#include <fstream>
#include <sstream>
#include <iostream>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include "iCub/DMP/DMP.h"


#include <cmath>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

class DMPCanonicalStateX: public dmp::DMPCanonicalState
{
        double x_;
public:
    DMPCanonicalStateX();
    DMPCanonicalStateX(double x);
    DMPCanonicalStateX(const DMPCanonicalStateX& original);
    virtual DMPCanonicalStateX* clone() const;

    void reset();
    double get_x() const;
    void set_x(double x);
};



class DMPCanonicalPointToPoint : public dmp::DMPCanonical{
   DMPCanonicalStateX phase_;
    double alpha_x_;
    double tau_;
    double threshold;
    
public:
    DMPCanonicalPointToPoint();
    DMPCanonicalPointToPoint(double alpha_x);
    DMPCanonicalPointToPoint(const DMPCanonicalPointToPoint& original);
    virtual DMPCanonicalPointToPoint* clone() const;
    ~DMPCanonicalPointToPoint(){};
    void set_alphax(double alpha_x);
    double get_alphax() const;
    void set_tau(double tau);
    double get_tau() const;
    double const get_phase();
    bool integrate(double dt);
    dmp::DMPCanonicalState* askStateAtTime(double time) const;
    bool estimate_time_constants(const dmp::Trajectory& trajectory);
    void reset();
    virtual size_t get_number_of_constants() const;
    
    virtual yarp::sig::Vector get_constants() const;
    virtual bool set_constants(const yarp::sig::Vector& newConsts);
    
    virtual size_t get_number_of_variables() const;
        
    virtual yarp::sig::Vector get_variables() const;
    
    virtual bool set_variables(const yarp::sig::Vector& newVars); 
    
    void print(std::ostream& os) const;
    
};

class PointToPointBasisFunction : public dmp::BasisFunction
{
protected:
    double c; //center
    double sigma2; //variance
    void print(std::ostream& os) const
    {
        os << "Center: " << c << ", variance: " << sigma2 << ".";
    };
public:
    PointToPointBasisFunction(double center, double variance): c(center), sigma2(variance)
    {};
    PointToPointBasisFunction(const PointToPointBasisFunction& original): c(original.c), sigma2(original.sigma2) {};
    virtual PointToPointBasisFunction* clone() const
    {
        return new PointToPointBasisFunction(*this);
    };
    double get_center() const {return c;}
    double get_variance() const {return sigma2;}
    virtual double calculate(const dmp::DMPCanonicalState* canonicalState)
    {
        const DMPCanonicalStateX* myCanStateX= dynamic_cast<const DMPCanonicalStateX*>(canonicalState);
        if (!myCanStateX)
            return 0.0;
        return exp(-0.5* pow(myCanStateX->get_x() - c, 2) / sigma2);
    }
    
};

class TransformationSystemPointToPoint : public dmp::TransformationSystem 
{

    double alpha_z;
    double beta_z;
    double alpha_x;
    double tau;
    yarp::sig::Vector w;
    double lastC;
    size_t N;
   // double attractor;
    bool scaling;
    
    void define_basis_functions(double alpha_x, double factor);
    
    
    bool estimate_attractor(const dmp::Trajectory& trajectory);
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
    TransformationSystemPointToPoint();
    ~TransformationSystemPointToPoint(){};
    TransformationSystemPointToPoint(yarp::os::Property properties);
    TransformationSystemPointToPoint(double alpha_z, double beta_z, double alpha_x, int N);

    TransformationSystemPointToPoint(const TransformationSystemPointToPoint& original);
    virtual TransformationSystemPointToPoint* clone() const;
    
    virtual size_t get_number_of_basis_functions() const;    

    double get_alphaz() const;
    double get_betaz() const;

    virtual bool set_time_constants(const dmp::DMPCanonical* canonicalFunction);

    
};

/**
* Dynamic Motion Primitive representation of a point-to-point (i.e., non periodic) movement.
*/
class DMPPointToPoint: public dmp::DMP 
{
protected:
    virtual void print(std::ostream& os) const;

virtual bool fromSearchable(const yarp::os::Searchable* properties);

public:
    /**
    * Constructor.
    *
    * @param[in] dof degrees of freedom of original trajectories 
    * @param[in] N number of basis functions
    * @param[in] alpha_x alpha_x time constant in DMP dynamic system
    * @param[in] alpha_z alpha_x time constant in DMP dynamic system
    * @param[in] beta_z beta_z time constant in DMP dynamic system
    * 
    */
    DMPPointToPoint(int dof, int N, double alpha_x, double alpha_z, double beta_z);
    
    std::string getType() const {return "DMPPointToPoint";};
    DMPPointToPoint(const yarp::os::Property& properties);
    ~DMPPointToPoint();
    
    DMPPointToPoint(const DMPPointToPoint& original);
    virtual DMPPointToPoint* clone() const;
    
    void set_tau(double tau);
    
    virtual bool fromBottle(const yarp::os::Bottle& bot);
    virtual yarp::os::Bottle toBottle() const;
    
};

#endif
