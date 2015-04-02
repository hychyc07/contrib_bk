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
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <yarp/math/SVD.h>

#include <iCub/DMP/DMP.h>

using namespace iCub::ctrl;
using namespace yarp::math;
using namespace dmp;

class LSestimator
{
    yarp::sig::Matrix A;
    yarp::sig::Vector f;
public:
    LSestimator(size_t nSamples, size_t nBasisFunctions): A(nSamples, nBasisFunctions), f(nSamples)
    {};
    ~LSestimator(){};
    bool set_target(size_t idSample, yarp::sig::Vector& inputVals, double fTarget)
    {
        if (idSample >= A.rows() || idSample >=f.length())
            return false;
        A.setRow(idSample, inputVals);
        f[idSample]=fTarget;
        return true;
    };
    yarp::sig::Vector compute_weights()
    {
        
        yarp::sig::Matrix Ainv=yarp::math::pinv(A);
         return Ainv*f;
        
    };
};

yarp::sig::Vector derive(const yarp::sig::Vector& vec, const yarp::sig::Vector& time)
 {
   yarp::sig::Vector result(vec.size());
   
//    size_t nSamples=time.length();
//     yarp::sig::Vector timeDiffs(nSamples);
//     timeDiffs(0)=0.0;
//     for (size_t i =1; i<nSamples; ++i)
//         timeDiffs(i)=time(i)-time(i-1);
// 
//     for (size_t i =1; i<nSamples; ++i)
//         result(i)=(vec(i)-vec(i-1))/timeDiffs(i);
//     result(0)=result(1);

   //TODO check that size is the same
   int NVel=3;
   double DVel=0.001;
    AWLinEstimator *linEst= new AWLinEstimator(NVel,DVel);
    for (size_t i=0; i<vec.size(); ++i) 
    {
      AWPolyElement el;
      yarp::sig::Vector sample(1);
      sample[0]=vec[i];
          el.data=sample;
          el.time=time[i];
          result[i]=linEst->estimate(el)[0];
    }
    for (size_t i=0; i<NVel; ++i) 
        result[i]=result[NVel];

   return result;
};


dmp::DMPCanonicalState::~DMPCanonicalState(){};
dmp::DMPCanonical::~DMPCanonical(){};

TransformationSystem::TransformationSystem(): z(0.0), y(0.0)
{}

dmp::TransformationSystem::TransformationSystem(const TransformationSystem& original): z(original.z), y(original.y), attractor(original.attractor)
{
    for (std::vector<dmp::BasisFunction*>::const_iterator bfIt=original.basisFunctions.begin(); bfIt!=original.basisFunctions.end(); ++bfIt)
         basisFunctions.push_back((*bfIt)->clone());
    
}
dmp::TransformationSystem::~TransformationSystem()
{
    if (!basisFunctions.empty())
    {
        for (std::vector<dmp::BasisFunction*>::iterator bfIt=basisFunctions.begin(); bfIt!=basisFunctions.end(); ++bfIt)
            delete *bfIt;
        
        basisFunctions.clear();
    }
};

double dmp::TransformationSystem::get_position() const
{
    return y;
}

void dmp::TransformationSystem::set_position(double newPos)
{
    y=newPos;
}

void dmp::TransformationSystem::set_attractor(double newAttr)
{
    attractor=newAttr;
}

double dmp::TransformationSystem::get_attractor() const
{
    return attractor;
}

double dmp::TransformationSystem::get_velocity() const
{
    return z;
}

void dmp::TransformationSystem::set_velocity(double newVel)
{
    z=newVel;
}


yarp::sig::Vector dmp::TransformationSystem::calculate_all_basis_functions_at_time(double time, const DMPCanonical* canonicalFunction)
{
    DMPCanonicalState* myState=canonicalFunction->askStateAtTime(time);
     
    if (!myState)
        return yarp::sig::Vector();
    double psi_sum = 0;
    yarp::sig::Vector result(get_number_of_basis_functions());
    for (size_t k = 0; k < get_number_of_basis_functions(); k++)
    {
        
        double psi=basisFunctions.at(k)->calculate(myState); 
      //std::cout << ", psi for bf " << k <<" :" <<psi;  
        psi_sum += psi;
        
        normalize_fx(psi, myState);
        
        result(k)= psi;
    }
    
    delete myState;
    if (psi_sum > DMP_EPSILON)
        return result/psi_sum;
    else
        return yarp::sig::Vector();
}

bool dmp::TransformationSystem::integrate(const DMPCanonicalState* canonicalState, double dt)
{
    double sum_psi=0.0;
    double fx=0;
    for (size_t i = 0; i <get_number_of_basis_functions(); i++)
    {
        //  double psi = exp( -0.5 * pow((myCanStateX->get_x()-c(i))/sigma2(i), 2));
        double psi=basisFunctions.at(i)->calculate(canonicalState);
        fx+= get_weight(i)*psi;
        sum_psi += psi;
    }
    
    if (sum_psi >DMP_EPSILON)
        fx/=sum_psi;   
    bool ok=normalize_fx(fx, canonicalState);
    ok= ok && scale(fx);
    ok=ok && integrate_output_system(dt, fx);
 //   return true;
    return ok;  //need to check when failing occurs....   
};


bool dmp::TransformationSystem::learn(const Trajectory& trajectory, const DMPCanonical* canonicalFunction)
{
    size_t nSamples=trajectory.get_number_of_samples();
    yarp::sig::Vector times=trajectory.get_time_vector()-trajectory.get_time_vector()[0];
    std::cout << "nSamples " <<nSamples <<std::endl;
    yarp::sig::Vector positions=trajectory.get_trajectory_for_dof(0);
    std::cout << "times size " << times.length() <<" positions size " << positions.length()<<std::endl;
    yarp::sig::Vector velocities=derive(positions, times);
    yarp::sig::Vector accelerations=derive(velocities, times);
    
//         std::cout << "times " << times.toString() <<std::endl;
//         std::cout << "positions " << positions.toString() <<std::endl;
//         std::cout << "velocities " << velocities.toString() <<std::endl;
//         std::cout << "accelerations " << accelerations.toString() <<std::endl;
    bool ok= estimate_attractor(trajectory);
        std::cout << "estimated attractor " << std::endl;
    LSestimator lsEst(nSamples, get_number_of_basis_functions());
    
    for (size_t j = 0; j < nSamples; ++j)
    {
        yarp::sig::Vector bfVals=calculate_all_basis_functions_at_time(times[j], canonicalFunction);
        if (bfVals.length()>0)
            ok= ok &&lsEst.set_target(j, bfVals, calculate_output(positions[j], velocities[j], accelerations[j]) );
    }
    return ok && set_weights(lsEst.compute_weights());
    
};

dmp::DMP::DMP(size_t dof) : integrationStep(0.001), verbose(false), dof_(dof), valid(false)
{}

dmp::DMP::DMP(const dmp::DMP& original): integrationStep(original.integrationStep), verbose(original.verbose), dof_(original.dof_), valid(original.valid)
{
    canonicalSystem_=original.canonicalSystem_->clone();
    for (size_t i=0; i<original.get_number_of_dofs(); ++i)
    {
        transformations_.push_back(original.transformations_.at(i)->clone());
    }
    
}

dmp::DMP::~DMP()
{
    if (canonicalSystem_)
         delete canonicalSystem_;
    for (std::vector<dmp::TransformationSystem*>::iterator transfIt=transformations_.begin(); transfIt!=transformations_.end(); ++transfIt)
        {
            if(*transfIt)
                delete *transfIt;
        }
};

bool dmp::DMP::integrate(double dt)
{
        
    size_t steps = (size_t) (dt /integrationStep + 0.5);

    if (verbose)
        std::cout << "Integration step: " <<integrationStep << ", n. steps: " << steps <<std::endl;
    if (steps <=0)
        return false;
    double stepdt = dt / steps;
    bool ok=true;
    for (size_t t=0; t<steps; ++t)
    {
        for (std::vector<dmp::TransformationSystem*>::iterator transfIt=transformations_.begin(); transfIt!=transformations_.end(); ++transfIt)
        {
            ok= (*transfIt)->integrate(canonicalSystem_->getCanonicalState(), stepdt) && ok;
        }
        ok=canonicalSystem_->integrate(stepdt) && ok;
        if (verbose)
            std::cout << "Step "<< t << ": positions " << get_positions().toString().c_str() << ", state: "<< *canonicalSystem_<< std::endl;
 
    }
        return ok;  
};


 bool dmp::DMP::estimate_from_trajectory(const dmp::Trajectory& trajectory)
{
    int nSamples=trajectory.get_number_of_samples();
    if (verbose)
        std::cout << "nSamples " <<nSamples <<std::endl;
    
    bool success = canonicalSystem_->estimate_time_constants(trajectory);
    
    
    size_t ct=0;
    yarp::sig::Vector time=trajectory.get_time_vector();
    for (std::vector<dmp::TransformationSystem*>::iterator transfIt=transformations_.begin(); transfIt!=transformations_.end(); ++transfIt, ++ct)
        {
            success = success && (*transfIt)->set_time_constants(canonicalSystem_);
            // std::cout <<"inside dmp estimate for dof " << ct <<std::endl;
            dmp::Trajectory tmpTraj(1, nSamples);
            yarp::sig::Vector values=trajectory.get_trajectory_for_dof(ct);
            for (size_t ctSamples=0; ctSamples < nSamples; ++ctSamples)
            {
                tmpTraj.add_trajectory_point(time[ctSamples], yarp::sig::Vector(1,values[ctSamples]));
            }
//             tmpTraj.set_time_vector(trajectory.get_time_vector());
//             std::cout << "set time vector ";
//             tmpTraj.set_trajectory_for_dof(0,trajectory.get_trajectory_for_dof(ct));
//             std::cout << "set  trajectory  vector ";
            success = (*transfIt)->learn(tmpTraj, canonicalSystem_) && success;
        }
    return success;
    
};

void dmp::DMP::resetState()
{
    canonicalSystem_->reset();
    
}
size_t dmp::DMP::get_number_of_dofs() const {return dof_;};

yarp::sig::Vector dmp::DMP::get_positions() const
{
    if (transformations_.size()<dof_)
        return yarp::sig::Vector();
    yarp::sig::Vector positions(dof_);
    for (size_t i=0; i<dof_; ++i)
    {
        positions(i)=transformations_.at(i)->get_position();
    }
    return positions;
};

yarp::sig::Vector dmp::DMP::get_velocities() const
{
    if (transformations_.size()<dof_)
        return yarp::sig::Vector();
    yarp::sig::Vector velocities(dof_);
    for (size_t i=0; i<dof_; ++i)
    {
        velocities(i)=transformations_.at(i)->get_velocity();
    }
    return velocities;
};

bool dmp::DMP::set_positions(const yarp::sig::Vector& newPos)
{
    if (transformations_.size()<dof_ || newPos.size()<dof_)
        return false;
    for (size_t i=0; i<dof_; ++i)
    {
        transformations_.at(i)->set_position(newPos[i]);
    }
    return true;
};

bool dmp::DMP::set_attractor(const yarp::sig::Vector& newAttractor)
{
    if (transformations_.size()<dof_ || newAttractor.size()<dof_)
        return false;
    for (size_t i=0; i<dof_; ++i)
    {
        transformations_.at(i)->set_attractor(newAttractor[i]);
    }
    return true;
};

yarp::sig::Vector dmp::DMP::get_attractor() const
{
    if (transformations_.size()<dof_)
        return yarp::sig::Vector();
    yarp::sig::Vector attractor(dof_);
    for (size_t i=0; i<dof_; ++i)
    {
        attractor(i)=transformations_.at(i)->get_attractor();
    }
    return attractor;
};

bool dmp::DMP::set_velocities(const yarp::sig::Vector& newVel)
{
    if (transformations_.size()<dof_ || newVel.size()<dof_)
        return false;
    for (size_t i=0; i<dof_; ++i)
    {
        transformations_.at(i)->set_velocity(newVel[i]);
    }
    return true;
};

bool dmp::DMP::set_weights(size_t indTS, const yarp::sig::Vector& newWeights)
{
    if (transformations_.size()<dof_ || indTS >= dof_)
        return false;

        return transformations_.at(indTS)->set_weights(newWeights);
};

yarp::sig::Vector dmp::DMP::get_weights(size_t indTS) const
{
    if (transformations_.size()<dof_ || indTS >= dof_)
        return yarp::sig::Vector();

    return transformations_.at(indTS)->get_weights();
};

yarp::sig::Vector dmp::DMP::get_DMP_constants() const
{
    yarp::sig::Vector result;
    for (std::vector<dmp::TransformationSystem*>::const_iterator transfIt=transformations_.begin(); transfIt!=transformations_.end(); ++transfIt)
    {
        result = yarp::math::cat(result, (*transfIt)->get_constants());
    }
    
        result = yarp::math::cat(result, canonicalSystem_->get_constants());
    return result;
};
bool dmp::DMP::set_DMP_constants(const yarp::sig::Vector & newConsts)
{
    bool ok=true;
    size_t first=0;
    size_t last=0;
    size_t argumentLength=newConsts.length();
    for (std::vector<dmp::TransformationSystem*>::iterator transfIt=transformations_.begin(); transfIt!=transformations_.end(); ++transfIt)
    {
        last=first+ (*transfIt)->get_number_of_constants()-1; 
        if (last>argumentLength-1)
        {
            ok=false;
            break;
        }
        yarp::sig::Vector tsConsts=newConsts.subVector(first, last);
        ok= ok && (*transfIt)->set_constants(tsConsts);
        first=last+1;
    }
    if (ok)
    { 
        yarp::sig::Vector csConsts=newConsts.subVector(first, argumentLength-1);
        ok=ok&& canonicalSystem_->set_constants(csConsts);
    }
    return ok;        
};

yarp::sig::Vector dmp::DMP::get_DMP_variables() const
{
    yarp::sig::Vector result(0);
    for (std::vector<dmp::TransformationSystem*>::const_iterator transfIt=transformations_.begin(); transfIt!=transformations_.end(); ++transfIt)
    {
        result = yarp::math::cat(result, (*transfIt)->get_variables());
    }
    
        result = yarp::math::cat(result, canonicalSystem_->get_variables());
    return result;
};

bool dmp::DMP::set_DMP_variables(const yarp::sig::Vector& newVars)
{
    bool ok=true;
    size_t first=0;
    size_t last=0;
    size_t argumentLength=newVars.length();
    for (std::vector<dmp::TransformationSystem*>::iterator transfIt=transformations_.begin(); transfIt!=transformations_.end(); ++transfIt)
    {
        last=first+ (*transfIt)->get_number_of_variables()-1; 
        if (last>argumentLength-1)
        {
            ok=false;
            break;
        }
        yarp::sig::Vector tsConsts=newVars.subVector(first, last);
        ok= ok && (*transfIt)->set_variables(tsConsts);
        first=last+1;
    }
    if (ok)
    { 
        yarp::sig::Vector csConsts=newVars.subVector(first, argumentLength-1);
        ok=ok&& canonicalSystem_->set_variables(csConsts);
    }
    return ok;        
};

