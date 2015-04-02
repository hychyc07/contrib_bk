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

#ifndef _DMP_TRAJECTORY_H_
#define _DMP_TRAJECTORY_H_

#include <yarp/math/Math.h>
#include "DMP.h"
namespace dmp{

class Trajectory{
protected:
    size_t nDofs_;
    size_t nSamples_;
    yarp::sig::Vector time;
    yarp::sig::Matrix dofs;
public:
    Trajectory(size_t nDofs, size_t nSamples)
    {

        nDofs_=nDofs;
        nSamples_=0;
        time.resize(nSamples);
        dofs.resize(nSamples, nDofs);
    };
    ~Trajectory()
    {    };
    bool get_time_vector(yarp::sig::Vector& time) const
    {
        time=this->time;
        return true;
    };
    yarp::sig::Vector get_time_vector() const {return time;};

    bool get_trajectory_for_dof(size_t i, yarp::sig::Vector& traj) const
    {
        if (i>=nDofs_)
            return true;
        traj=dofs.getCol(i);
        return true;
    };
    
    yarp::sig::Vector get_trajectory_for_dof(size_t i) const
    {
        if (i>=nDofs_)
            return yarp::sig::Vector(0);
        else return dofs.getCol(i);
    };
    
    bool add_trajectory_point(double time, const yarp::sig::Vector newPoint)
    {
        if (nSamples_ < this->time.length()) //memory allocated at consruction
        {
            this->time[nSamples_]=time;
            this->dofs.setRow(nSamples_, newPoint);
        }
        else
        {
            this->time.push_back(time);
            this->dofs= yarp::math::pile(dofs, newPoint);
        }
        nSamples_++;
        return true;
    };
    
    size_t get_number_of_samples() const 
    {
        return nSamples_;
    }
    
    size_t get_number_of_dofs() const {return nDofs_;}
    
    friend std::ostream& operator<<(std::ostream& os, const Trajectory& myTS)
    {
        //temp
        os << "Times: " << myTS.get_time_vector().toString() << std::endl;
        for (size_t ct=0; ct< myTS.get_number_of_dofs(); ++ct)
            os << "Values for dof " << ct << ": " << myTS.get_trajectory_for_dof(ct).toString() <<std::endl;
        return os;
    };
    
};}

#endif