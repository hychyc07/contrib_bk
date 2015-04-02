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

#include <cmath>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <gsl/gsl_fft_real.h>
#include <gsl/gsl_fft_halfcomplex.h>
#include <gsl/gsl_vector_double.h>
#include <gsl/gsl_statistics.h>

#include <iCub/DMP/DMPPeriodic.h>

using namespace yarp::os;
using namespace yarp::math;


double average(const yarp::sig::Vector& vector)
{
    double sum=0;
    for (size_t ct=0; ct<vector.size(); ++ct)
        sum +=vector[ct];
    return sum/vector.size();
};



/////////////

DMPCanonicalStatePhi::DMPCanonicalStatePhi()
{
    reset();
}

DMPCanonicalStatePhi::DMPCanonicalStatePhi(double phi)
{
    set_phi(phi);
    
}

DMPCanonicalStatePhi::DMPCanonicalStatePhi(const DMPCanonicalStatePhi& original):phi_(original.phi_)
{}

DMPCanonicalStatePhi* DMPCanonicalStatePhi::clone() const
{
    return new DMPCanonicalStatePhi(*this);
}


void DMPCanonicalStatePhi::reset()
{
    phi_=2*M_PI;  
}

double DMPCanonicalStatePhi::get_phi() const 
{
    return phi_;    
}

void DMPCanonicalStatePhi::set_phi(double phi)
{
    phi_=fmod(phi,2*M_PI);
} //need to consider also omega_?
    
/***************************/    
DMPCanonicalPeriodic::DMPCanonicalPeriodic()
{
    phase_.reset();
    state_=&phase_;
//   threshold=DMP_CANONICAL_THRESHOLD;
}

DMPCanonicalPeriodic::DMPCanonicalPeriodic(double omega)
{
    set_omega(omega);
    state_=&phase_;
    phase_.reset();
}

DMPCanonicalPeriodic::DMPCanonicalPeriodic(const DMPCanonicalPeriodic& original): omega_(original.omega_), phase_(original.phase_)
{
   state_=&phase_;
}
DMPCanonicalPeriodic* DMPCanonicalPeriodic::clone() const
{
    return new DMPCanonicalPeriodic(*this);
}

void DMPCanonicalPeriodic::set_omega(double omega)
{omega_=omega;}

double DMPCanonicalPeriodic::get_omega() const
{return omega_;}

double DMPCanonicalPeriodic::get_phase() const
{return phase_.get_phi();}

dmp::DMPCanonicalState* DMPCanonicalPeriodic::askStateAtTime(double time) const
{
    DMPCanonicalStatePhi* state=new DMPCanonicalStatePhi(time*omega_);
    return state;
}

bool DMPCanonicalPeriodic::estimate_time_constants(const dmp::Trajectory& trajectory)
{

    double origSize=trajectory.get_number_of_samples();
    double samplingTime=trajectory.get_time_vector()[1]-trajectory.get_time_vector()[0]; //TODO : use average of time diffs
//      double newSize=pow(2,(int(floor(log(origSize) / log(2)) + 1)));
//      zero padding
//      for (size_t ind=origSize; ind <newSize; ++ind)
//          x[ind]=0.0;
    double newSize=pow(double(2),(int(floor(log(origSize) / log(double(2))))));
    std::cout <<"Considering " << newSize << " instead of " << origSize << " samples for frequency estimation." << std::endl;
    // x.resize ((size_t) newSize);
    
    yarp::sig::Vector x((size_t) newSize);
    x.zero();
    for (size_t iDof=0; iDof<trajectory.get_number_of_dofs(); ++iDof)
    {
    yarp::sig::Vector tmp=trajectory.get_trajectory_for_dof(iDof);
//         double origSize=x.length();
//   //      double newSize=pow(2,(int(floor(log(origSize) / log(2)) + 1)));
//   //      zero padding
//   //      for (size_t ind=origSize; ind <newSize; ++ind)
//   //          x[ind]=0.0;
//         double newSize=pow(2,(int(floor(log(origSize) / log(2)))));
//         std::cout <<"Considering " << newSize << " instead of " << origSize << " samples for frequency estimation." << std::endl;
    tmp.resize ((size_t) newSize);
    
        
    int z = gsl_fft_real_radix2_transform(&(tmp.data()[0]), 1, tmp.size());

    x+=tmp;
    }
    size_t n=x.length();
    yarp::sig::Vector c(n); //power spectrum
    c[0] = x[0]*x[0];
    c[n/2] = x[n/2]*x[n/2];
        for (size_t i=1; i<n/2; ++i) 
        {
            c[i] = x[i]*x[i] + x[n-i]*x[n-i];
            c[n-i] = x[n-i]*x[i] - x[i]*x[n-i];
        }

    
    size_t ind=gsl_stats_max_index (&(c.data()[1]), 1, c.size()/2-1);
    ind++;
    std::cout <<"ind argmax: "<<ind<<" c(ind)" << c(ind) <<std::endl;
    
    
    double delta=-(x[ind+1]-x[ind-1])/(2*x[ind]-x[ind+1]-x[ind-1]);
    double newOmega=((double) (ind)+delta)/(n)/samplingTime*2*M_PI;

    std::cout <<"newOmega: "<<newOmega<<std::endl;
    set_omega(newOmega);
    return true;
}

bool DMPCanonicalPeriodic::integrate(double dt)
{        
    phase_.set_phi(phase_.get_phi() + omega_*dt);
    return true;        
}

void DMPCanonicalPeriodic::reset()
{
    phase_.reset();
    
}

size_t DMPCanonicalPeriodic::get_number_of_constants() const 
{return 0;}

yarp::sig::Vector DMPCanonicalPeriodic::get_constants() const
{
    return yarp::sig::Vector (0);
}

bool DMPCanonicalPeriodic::set_constants(const yarp::sig::Vector& newConsts)
{
    return true;
}

size_t DMPCanonicalPeriodic::get_number_of_variables() const
{return 1;}

yarp::sig::Vector DMPCanonicalPeriodic::get_variables() const
{
    return  yarp::sig::Vector(1, omega_);
}

bool DMPCanonicalPeriodic::set_variables(const yarp::sig::Vector& newVars)
{
    if(newVars.length()<get_number_of_variables())
        return false;
    omega_=newVars(0);
    return true;
}

void DMPCanonicalPeriodic::print(std::ostream& os) const
{
    os << "current phase phi = " << phase_.get_phi() <<  ", omega_=" << omega_ <<std::endl;
};
     
//////////////////
double PeriodicBasisFunction::calculate(const dmp::DMPCanonicalState* canonicalState)
{
    const DMPCanonicalStatePhi* myCanStatePhi= dynamic_cast<const DMPCanonicalStatePhi*>(canonicalState);
    if (!myCanStatePhi)
        return 0.0;
    return exp((cos(myCanStatePhi->get_phi() - c)-1) / sigma2);
}

 //////
void TransformationSystemPeriodic::define_basis_functions(double factor)
{  
    if (!basisFunctions.empty())
    {
        for (std::vector<dmp::BasisFunction*>::iterator bfIt=basisFunctions.begin(); bfIt!=basisFunctions.end(); ++bfIt)
            delete *bfIt;
        
        basisFunctions.clear();
    }
        
    basisFunctions.resize(N);
    
    for (size_t i = 0; i < N; i++)
    {
        
        basisFunctions.at(i)=new PeriodicBasisFunction(((double) i)*2*M_PI/N, pow((2*M_PI/N)*factor, 2) );
    }        
}

    
bool TransformationSystemPeriodic::estimate_attractor(const dmp::Trajectory& trajectory)
{
    yarp::sig::Vector positions=trajectory.get_trajectory_for_dof(0);
    attractor = average(positions);
    return true;
}

bool TransformationSystemPeriodic::normalize_fx(double& fx, const dmp::DMPCanonicalState* canonicalState)
{
    const DMPCanonicalStatePhi* myCanStatePhi= dynamic_cast<const DMPCanonicalStatePhi*>(canonicalState);
    if (!myCanStatePhi)
        return false;
    fx*=myCanStatePhi->get_phi();
    return true;        
}
   
bool TransformationSystemPeriodic::scale(double& fx)
{
    return true;
}

bool TransformationSystemPeriodic::integrate_output_system(double dt, double fx)
{
    double dz = alpha_z * (beta_z * ( attractor - y) -  z) + fx;

    //   std::cout << "z: " << z <<"\tdz: " << dz <<"\t dt: " << dt <<"\tomega "<<omega <<"\t";
    // Euler integration: new positions (y) and scaled velocities (z)
    y+=z*dt*omega;
    z+=dz*dt*omega;
    return true;
};
    
size_t TransformationSystemPeriodic::get_number_of_constants() const 
{return 2;}

    
yarp::sig::Vector TransformationSystemPeriodic::get_constants() const
{
    yarp::sig::Vector result(2);
    result(0)=alpha_z;
    result(1)=beta_z;
    return result;
}

bool TransformationSystemPeriodic::set_constants(const yarp::sig::Vector& newConsts)
{
    if(newConsts.length()<get_number_of_constants())
        return false;
    alpha_z=newConsts(0);
    beta_z=newConsts(1);
    return true;
}

size_t TransformationSystemPeriodic::get_number_of_variables() const
{
    return N+1;
}

yarp::sig::Vector TransformationSystemPeriodic::get_variables() const
{
    yarp::sig::Vector result=w;
    result.push_back(attractor);
    return result;
}

bool TransformationSystemPeriodic::set_variables(const yarp::sig::Vector& newVars)
{
    if(newVars.length()<get_number_of_variables())
        return false;
    w=newVars.subVector(0, newVars.length()-2);
    attractor=newVars(newVars.length()-1);
    return true;
}

double TransformationSystemPeriodic::calculate_output(double position, double velocity, double acceleration)
{
    return acceleration /(omega* omega)- alpha_z * (beta_z * (attractor - position) -
                              velocity/omega);
}

bool TransformationSystemPeriodic::set_weights(const yarp::sig::Vector& weights)
{
    if (weights.size()==N)
    {
        w=weights;
        return true;
    }
    else return false;
}

yarp::sig::Vector TransformationSystemPeriodic::get_weights() const 
{
    return w;
}

TransformationSystemPeriodic::TransformationSystemPeriodic( double alpha_z, double beta_z, int N):TransformationSystem()
{
    this->alpha_z=alpha_z;
    this->beta_z=beta_z;
    this->N=N;
    w.resize(N);
    w.zero();
    define_basis_functions(1);
    scaling=false;
    this->omega=0.0;
};

TransformationSystemPeriodic::TransformationSystemPeriodic(const TransformationSystemPeriodic& original): TransformationSystem(original)
{
    this->alpha_z=original.alpha_z;
    this->beta_z=original.beta_z;
    this->N=original.N;
    this->w=original.w;
    
    define_basis_functions(1);
    scaling=original.scaling;
    this->omega=original.omega;
}
TransformationSystemPeriodic* TransformationSystemPeriodic::clone() const
{
    return new TransformationSystemPeriodic(*this);
}

double TransformationSystemPeriodic::get_alphaz() const
{
    return alpha_z;
}

double TransformationSystemPeriodic::get_betaz() const
{
    return beta_z;
}

void TransformationSystemPeriodic::print(std::ostream& os) const
{
    os << "Weights = " << w.toString() << ", number BFs= " << N << ";" << std::endl;
    for (size_t i=0; i<N; ++i)
      os << "BF n." << i << ": " << *(basisFunctions.at(i)) << "\t"; 
    os<<std::endl;   
    os << "Beta_z= "<< beta_z << ", alpha_z=" << alpha_z <<", current position: " << y << ", current velocity: " << z << ", attractor " <<attractor << std::endl;
};
    
bool TransformationSystemPeriodic::set_time_constants(const dmp::DMPCanonical* canonicalFunction)
{
    const DMPCanonicalPeriodic* myCanPer= dynamic_cast<const DMPCanonicalPeriodic*>(canonicalFunction);
    if (!myCanPer)
        return false;
    omega=myCanPer->get_omega();
    return true; 
}

size_t TransformationSystemPeriodic::get_number_of_basis_functions() const
{
    return N;
}

//////

DMPPeriodic::DMPPeriodic(const yarp::os::Property& properties)
{
    canonicalSystem_=NULL;
    fromSearchable(&properties);    
}
 
DMPPeriodic::DMPPeriodic(int dof, int N, double alpha_z, double beta_z): DMP(dof_)
{
    for (size_t i=0; i<dof_; ++i)
        transformations_.push_back(new TransformationSystemPeriodic(alpha_z, beta_z, N));
    canonicalSystem_=new DMPCanonicalPeriodic();
    valid=true;  
}

DMPPeriodic::~DMPPeriodic()
{}

DMPPeriodic::DMPPeriodic(const DMPPeriodic& original):  DMP(original)
{}

DMPPeriodic* DMPPeriodic::clone() const
{
    return new DMPPeriodic(*this);
}

void DMPPeriodic::set_omega(double omega)
{
    dynamic_cast<DMPCanonicalPeriodic*>(canonicalSystem_)->set_omega(omega);
    for (std::vector<dmp::TransformationSystem*>::iterator transfIt=transformations_.begin(); transfIt!=transformations_.end(); ++transfIt)
    {
        (*transfIt)->set_time_constants(canonicalSystem_);
    }
};

bool DMPPeriodic::fromSearchable(const yarp::os::Searchable* property)
{
    Searchable* prop=const_cast<Searchable*>(property);
    if (canonicalSystem_)
        delete canonicalSystem_;
    for (std::vector<dmp::TransformationSystem*>::iterator transfIt=transformations_.begin(); transfIt!=transformations_.end(); ++transfIt)
       {
           if(*transfIt)
               delete *transfIt;
       }   
       transformations_.clear();
    
    if ( !prop->check("omega") || !prop->check("N")  || !prop->check("dof") || !prop->check("alphaz") || !prop->check("betaz"))
        valid=false;
    
    int N=prop->find("N").asInt();
    dof_=prop->find("dof").asInt();
   // double alpha_x=property.find("alphax").asDouble();
    double alpha_z=prop->find("alphaz").asDouble();
    double beta_z=prop->find("betaz").asDouble();
    
      for (size_t i=0; i<dof_; ++i)
          transformations_.push_back(new TransformationSystemPeriodic(alpha_z, beta_z, N));
    
    canonicalSystem_=new DMPCanonicalPeriodic();
    
    ///dimensions of the movement system.
   // std::vector<dmp::TransformationSystem*> transformations_;
//     integrationStep=0.001;
//     verbose=true;
    valid=true;
    
    set_omega(prop->find("omega").asDouble());
    Bottle* weights=prop->find("weights").asList(); //they are serialized row by row
    if (weights)
    {
        for (int i1=0; i1<weights->size(); ++i1)
        {
            Bottle* weightRow=weights->get(i1).asList();
            if (weightRow)
            {
                yarp::sig::Vector weightRowVector(weightRow->size());
                for (int i2=0; i2<weightRow->size(); ++i2)
                    weightRowVector(i2)=weightRow->get(i2).asDouble();
                set_weights(i1, weightRowVector);
            }
        }
    }

    Bottle* goal=prop->find("attractor").asList();
    if(goal)
    {
        yarp::sig::Vector goalVector(goal->size());
        for(int i1=0; i1<goal->size(); ++i1)
            goalVector(i1)=goal->get(i1).asDouble();
        set_attractor(goalVector);
    }
    
    return valid;
}

bool DMPPeriodic::fromBottle(const yarp::os::Bottle& bot)
{
    return fromSearchable(&bot);
      
}

yarp::os::Bottle DMPPeriodic::toBottle() const
{
    Bottle bDmp;
    TransformationSystemPeriodic* ts=dynamic_cast<TransformationSystemPeriodic*> (transformations_.at(0));
    DMPCanonicalPeriodic* myCanPer= dynamic_cast<DMPCanonicalPeriodic*>(canonicalSystem_);

    if (transformations_.size() <1 || ts == NULL || myCanPer == NULL)
        return bDmp;
    Bottle& N=bDmp.addList();
    N.addString("N");
    N.addInt(ts->get_number_of_basis_functions());
    Bottle& dof=bDmp.addList();
    dof.addString("dof");
    dof.addInt(dof_);
//     Bottle& alpha_x=bDmp.addList();
//     alpha_x.addString("alphax");
//     alpha_x.addInt(dmp.get_alphax());
    Bottle& alpha_z=bDmp.addList();
    alpha_z.addString("alphaz");
    alpha_z.addDouble(ts->get_alphaz());
    Bottle& beta_z=bDmp.addList();
    beta_z.addString("betaz");
    beta_z.addDouble(ts->get_betaz());
    Bottle& tau=bDmp.addList();
    tau.addString("omega");
    tau.addDouble(myCanPer->get_omega());
    Bottle& weights=bDmp.addList();
    weights.addString("weights");
    Bottle & weightRows=weights.addList();
    for (size_t i1=0; i1<dof_; ++i1)
    {
        yarp::sig::Vector weightsVector=get_weights(i1);
        Bottle& weightsColumn=weightRows.addList();
        for (size_t i2=0; i2<weightsVector.length(); ++i2)
        {
            weightsColumn.addDouble(weightsVector(i2));
        }
    }
    Bottle& goal=bDmp.addList();
    goal.addString("attractor");
    Bottle& goalData=goal.addList();
    yarp::sig::Vector goalVector=get_attractor();
    for(size_t i1=0; i1<goalVector.length(); ++i1)
        goalData.addDouble(goalVector(i1));
    return bDmp; 
}

void DMPPeriodic::print(std::ostream& os) const
  {
    os << "Canonical system: " << *canonicalSystem_;
    for (size_t ct=0; ct<dof_; ++ct)
        os <<"Transformation System n. " << ct << ": " << *(transformations_.at(ct));
}

