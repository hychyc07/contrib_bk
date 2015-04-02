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

#include <iCub/DMP/DMPPointToPoint.h>

using namespace yarp::os;
using namespace yarp::math;

#define DMP_CANONICAL_THRESHOLD 0.05

#ifdef _MSC_VER
  #define copysign _copysign
#endif


DMPCanonicalStateX::DMPCanonicalStateX()
{
    reset();
}

DMPCanonicalStateX::DMPCanonicalStateX(double x)
{
    set_x(x);
}

DMPCanonicalStateX::DMPCanonicalStateX(const DMPCanonicalStateX& original):x_(original.x_)
{}

DMPCanonicalStateX* DMPCanonicalStateX::clone() const
{
    return new DMPCanonicalStateX(*this);
}

void DMPCanonicalStateX::reset()
{
    x_=1.0;
}

double DMPCanonicalStateX::get_x() const 
{
    return x_;
}

void DMPCanonicalStateX::set_x(double x)
{
    x_= (x>0.0) ? x : 0.0;
    if (x_ >1.0)
        x_=1.0;
}

/******************************/ 
 
DMPCanonicalPointToPoint::DMPCanonicalPointToPoint()
{
    state_=&phase_;
    threshold=DMP_CANONICAL_THRESHOLD;
}
 
DMPCanonicalPointToPoint::DMPCanonicalPointToPoint(double alpha_x) 
{
    set_alphax(alpha_x);
    set_tau(alpha_x);
    state_=&phase_;
    threshold=DMP_CANONICAL_THRESHOLD; 
}

DMPCanonicalPointToPoint::DMPCanonicalPointToPoint(const DMPCanonicalPointToPoint& original): alpha_x_(original.alpha_x_), tau_(original.tau_), threshold(original.threshold), phase_(original.phase_) 
{
    state_=&phase_;
}

DMPCanonicalPointToPoint* DMPCanonicalPointToPoint::clone() const
{
    return new DMPCanonicalPointToPoint(*this);
}
 
void DMPCanonicalPointToPoint::set_alphax(double alpha_x)
{
    alpha_x_=alpha_x;
}
 
double DMPCanonicalPointToPoint::get_alphax() const 
{
    return alpha_x_;
}
 
void DMPCanonicalPointToPoint::set_tau(double tau)
{
    tau_=tau;
}

double DMPCanonicalPointToPoint::get_tau() const
{
    return tau_;
}

double const DMPCanonicalPointToPoint::get_phase()
{
    return phase_.get_x();   
}

bool DMPCanonicalPointToPoint::integrate(double dt)
{        
    //  std::cout << "integrating at phase " << phase_.get_x() <<std::endl;
    if (phase_.get_x()<threshold)
        return false;
    double dx = -alpha_x_ *phase_.get_x()/ tau_; 
    phase_.set_x(phase_.get_x() + dx*dt);
    return true;        
}

dmp::DMPCanonicalState* DMPCanonicalPointToPoint::askStateAtTime(double time) const
{
    DMPCanonicalStateX* state=new DMPCanonicalStateX(exp(-alpha_x_ / tau_ * time));
    return state;
}

bool DMPCanonicalPointToPoint::estimate_time_constants(const dmp::Trajectory& trajectory)
{
    yarp::sig::Vector times=trajectory.get_time_vector();
    set_tau(times[trajectory.get_number_of_samples()-1] - times[0]);
    return true;
}

void DMPCanonicalPointToPoint::reset()
{
    phase_.reset();
    
}

size_t DMPCanonicalPointToPoint::get_number_of_constants() const
{
    return 1;
}

yarp::sig::Vector DMPCanonicalPointToPoint::get_constants() const
{
    yarp::sig::Vector result(get_number_of_constants());
    result(0)=alpha_x_;
    return result;
}

bool DMPCanonicalPointToPoint::set_constants(const yarp::sig::Vector& newConsts)
{
    if(newConsts.length()<get_number_of_constants())
        return false;
    alpha_x_=newConsts(0);
    return true;
}

size_t DMPCanonicalPointToPoint::get_number_of_variables() const
{
    return 1;   
}
    
yarp::sig::Vector DMPCanonicalPointToPoint::get_variables() const
{
    return yarp::sig::Vector(1, tau_);
}

bool DMPCanonicalPointToPoint::set_variables(const yarp::sig::Vector& newVars)
{
    if(newVars.length()<get_number_of_variables())
        return false;
    tau_=newVars(0);
    return true;
} 

void DMPCanonicalPointToPoint::print(std::ostream& os) const
{
    os << "current phase x = " << phase_.get_x() << " alpha x= " << alpha_x_ << ", tau_=" << tau_ <<std::endl;
}
     
/******************************/ 
void TransformationSystemPointToPoint::define_basis_functions(double alpha_x, double factor)
{
    if (!basisFunctions.empty())
    {
        for (std::vector<dmp::BasisFunction*>::iterator bfIt=basisFunctions.begin(); bfIt!=basisFunctions.end(); ++bfIt)
            delete *bfIt;
        
        basisFunctions.clear();
    }
        
    basisFunctions.resize(N);   

    for (size_t i = 0; i < N-1; i++)
    {
        basisFunctions.at(i)=new PointToPointBasisFunction(exp(-alpha_x*((double) i)/(N-1.0)), pow((exp(-alpha_x*(i+1)/(N-1.0))-exp(-alpha_x*(i)/(N-1.0)))*factor, 2) );
    }
    basisFunctions.at(N-1)=new PointToPointBasisFunction(exp(-alpha_x*(N-1.0)/(N-1.0)), pow((exp(-alpha_x*(N-1)/(N-1.0))-exp(-alpha_x*(N-2)/(N-1.0)))*factor, 2));
    
    lastC=exp(-alpha_x*(N-1.0)/(N-1.0));
//         c(i)= exp(-alpha_x*i/(N-1.0));
//     for (size_t i = 0; i < N - 1; i++)
//         sigma2(i)=  pow((c(i+1)-c(i))*factor, 2);
//     sigma2 (N-1)= sigma2(N-2);

 
}
     
bool TransformationSystemPointToPoint::estimate_attractor(const dmp::Trajectory& trajectory)
{
    yarp::sig::Vector positions=trajectory.get_trajectory_for_dof(0);
    attractor = positions[positions.length()-1];
    return true;
}

bool TransformationSystemPointToPoint::normalize_fx(double& fx, const dmp::DMPCanonicalState* canonicalState)
{
    const DMPCanonicalStateX* myCanStateX= dynamic_cast<const DMPCanonicalStateX*>(canonicalState);
    if (!myCanStateX)
        return false;
    if (myCanStateX->get_x() < lastC)
        fx=0.0;
    else
        fx*=myCanStateX->get_x();
    return true;        
}

bool TransformationSystemPointToPoint::scale(double& fx)
{
    if (scaling)
    {
            
            double EPS=1.0e-4;
            double g=attractor;
            g=(abs(g)>EPS ? g : copysign(EPS,g));        
                fx *= g;
        
    }
    return true;
}

bool TransformationSystemPointToPoint::integrate_output_system(double dt, double fx)
{
    double dz = alpha_z * (beta_z * ( attractor - y) -  z) + fx; //acceleration
    // Euler integration: new positions (y) and scaled velocities (z)
    y+=z*dt/tau;
    z+=dz*dt/tau;
    return true;
}

size_t TransformationSystemPointToPoint::get_number_of_constants() const 
{
    return 2;
}

yarp::sig::Vector TransformationSystemPointToPoint::get_constants() const
{
    yarp::sig::Vector result(get_number_of_constants());
    result(0)=alpha_z;
    result(1)=beta_z;
    return result;
}

bool TransformationSystemPointToPoint::set_constants(const yarp::sig::Vector& newConsts)
{
    if(newConsts.length()<get_number_of_constants())
        return false;
    alpha_z=newConsts(0);
    beta_z=newConsts(1);
    return true;
}

size_t TransformationSystemPointToPoint::get_number_of_variables() const
{
    return N+1;
    
}
    
yarp::sig::Vector TransformationSystemPointToPoint::get_variables() const
{
    yarp::sig::Vector result=w;
    result.push_back(attractor);
    return result;
}

bool TransformationSystemPointToPoint::set_variables(const yarp::sig::Vector& newVars)
{
    if(newVars.length()<get_number_of_variables())
        return false;
    w=newVars.subVector(0, newVars.length()-2);
    attractor=newVars(newVars.length()-1);
    return true;
    
}
    
double TransformationSystemPointToPoint::calculate_output(double position, double velocity, double acceleration)
{
    return tau* tau * acceleration - alpha_z * (beta_z * (attractor - position) -
                             tau * velocity);
};
bool TransformationSystemPointToPoint::set_weights(const yarp::sig::Vector& weights)
{
    if (weights.size()==N)
    {
        w=weights;
        return true;
    }
    else return false;
};

yarp::sig::Vector TransformationSystemPointToPoint::get_weights() const
{
    return w;
};

TransformationSystemPointToPoint::TransformationSystemPointToPoint(double alpha_z, double beta_z, double alpha_x, int N):TransformationSystem()
{
    this->alpha_z=alpha_z;
    this->beta_z=beta_z;
    this->alpha_x=alpha_x;
    this->N=N;
    w.resize(N);
    w.zero();
    define_basis_functions(alpha_x, 1);
    scaling=false;
    tau=-1.0;
};

TransformationSystemPointToPoint::TransformationSystemPointToPoint(const TransformationSystemPointToPoint& original) : TransformationSystem(original)
{
    this->alpha_z=original.alpha_z;
    this->beta_z=original.beta_z;
    this->alpha_x=original.alpha_x;
    this->N=original.N;
    this->w=original.w;
    this->scaling=original.scaling;
    this->tau=original.tau;
    this->lastC=original.lastC;
}

TransformationSystemPointToPoint* TransformationSystemPointToPoint::clone() const
{
    return new TransformationSystemPointToPoint(*this);
} 
 
size_t TransformationSystemPointToPoint::get_number_of_basis_functions() const
{
    return N;
}

double TransformationSystemPointToPoint::get_alphaz() const 
{
    return alpha_z;
}

double TransformationSystemPointToPoint::get_betaz() const 
{
    return beta_z;
}

bool TransformationSystemPointToPoint::set_time_constants(const dmp::DMPCanonical* canonicalFunction)
{
    const DMPCanonicalPointToPoint* myCanP2p= dynamic_cast<const DMPCanonicalPointToPoint*>(canonicalFunction);
    if (!myCanP2p)
        return false;
    tau=myCanP2p->get_tau();
    return true;
}

void TransformationSystemPointToPoint::print(std::ostream& os) const
{
    os << "Weights = " << w.toString() << ", number BFs= " << N << ";" << std::endl;
    for (size_t i=0; i<N; ++i)
      os << "BF n." << i << ": " << *(basisFunctions.at(i)) << "\t"; 
    os<<std::endl;
    os << "Beta_z= "<< beta_z << ", alpha_z=" << alpha_z <<", current position: " << y << ", current velocity: " << z << ", attractor " <<attractor << std::endl;
};
 

/******************************/ 

DMPPointToPoint::DMPPointToPoint(int dof, int N, double alpha_x, double alpha_z, double beta_z): dmp::DMP(dof)
{
        for (int i=0; i<dof; ++i)
            transformations_.push_back(new TransformationSystemPointToPoint(alpha_z, beta_z, alpha_x, N));

    canonicalSystem_=new DMPCanonicalPointToPoint(alpha_x);
    valid=true;    
}
 
DMPPointToPoint::DMPPointToPoint(const yarp::os::Property& properties)
{
    canonicalSystem_=NULL;
    fromSearchable(&properties);   
}
DMPPointToPoint::~DMPPointToPoint()
{}

DMPPointToPoint::DMPPointToPoint(const DMPPointToPoint& original): DMP(original)
{}

DMPPointToPoint* DMPPointToPoint::clone() const
{
    return new DMPPointToPoint(*this);
}

void DMPPointToPoint::print(std::ostream& os) const
{
    os << "Canonical system: " << *canonicalSystem_;
    for (size_t ct=0; ct<get_number_of_dofs(); ++ct)
        os <<"Transformation System n. " << ct << ": " << *(transformations_.at(ct));
}

bool DMPPointToPoint::fromSearchable(const yarp::os::Searchable* property)
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
    
    if ( !prop->check("tau") || !prop->check("N")  || !prop->check("dof") || !prop->check("alphax") || !prop->check("alphaz") || !prop->check("betaz"))
        valid=false;
    
    int N=prop->find("N").asInt();
    dof_=prop->find("dof").asInt();
    double alpha_x=prop->find("alphax").asDouble();
    double alpha_z=prop->find("alphaz").asDouble();
    double beta_z=prop->find("betaz").asDouble();
    
      for (size_t i=0; i<dof_; ++i)
          transformations_.push_back(new TransformationSystemPointToPoint(alpha_z, beta_z, alpha_x, N));
    
    canonicalSystem_=new DMPCanonicalPointToPoint(alpha_x);
    
    ///dimensions of the movement system.
   // std::vector<dmp::TransformationSystem*> transformations_;
//     integrationStep=0.001;
//     verbose=true;
    valid=true;
    
    set_tau(prop->find("tau").asDouble());
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

    Bottle* goal=prop->find("goal").asList();
    if(goal)
    {
        yarp::sig::Vector goalVector(goal->size());
        for(int i1=0; i1<goal->size(); ++i1)
            goalVector(i1)=goal->get(i1).asDouble();
        set_attractor(goalVector);
    }
    
    return valid;
}


void DMPPointToPoint::set_tau(double tau)
{
    dynamic_cast<DMPCanonicalPointToPoint*>(canonicalSystem_)->set_tau(tau);
    for (std::vector<dmp::TransformationSystem*>::iterator transfIt=transformations_.begin(); transfIt!=transformations_.end(); ++transfIt)
    {
        (*transfIt)->set_time_constants(canonicalSystem_);
    }
};

bool DMPPointToPoint::fromBottle(const yarp::os::Bottle& bot)
{
    return fromSearchable(&bot);
      
}

yarp::os::Bottle DMPPointToPoint::toBottle() const
{
    std::cout << "dmpP2P to bottle" <<std::endl;
    Bottle bDmp;
    TransformationSystemPointToPoint* ts=dynamic_cast<TransformationSystemPointToPoint*> (transformations_.at(0));
    DMPCanonicalPointToPoint* myCanP2p= dynamic_cast<DMPCanonicalPointToPoint*>(canonicalSystem_);

    if (transformations_.size() <1 || ts == NULL || myCanP2p == NULL)
        return bDmp;
    Bottle& N=bDmp.addList();
    N.addString("N");
    N.addInt(ts->get_number_of_basis_functions());
    Bottle& dof=bDmp.addList();
    dof.addString("dof");
    dof.addInt(dof_);
    Bottle& alpha_x=bDmp.addList();
    alpha_x.addString("alphax");
    alpha_x.addDouble(myCanP2p->get_alphax());
    Bottle& alpha_z=bDmp.addList();
    alpha_z.addString("alphaz");
    alpha_z.addDouble(ts->get_alphaz());
    Bottle& beta_z=bDmp.addList();
    beta_z.addString("betaz");
    beta_z.addDouble(ts->get_betaz());
    Bottle& tau=bDmp.addList();
    tau.addString("tau");
    tau.addDouble(myCanP2p->get_tau());
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
    goal.addString("goal");
    Bottle& goalData=goal.addList();
    yarp::sig::Vector goalVector=get_attractor();
    std::cout << "attractor " << goalVector.toString(); fflush(stdout);
    for(size_t i1=0; i1<goalVector.length(); ++i1)
        goalData.addDouble(goalVector(i1));
	return bDmp;
}
