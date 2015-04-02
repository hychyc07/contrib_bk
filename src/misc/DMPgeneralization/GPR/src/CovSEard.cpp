/* 
 * Copyright (C) 2012 Istituto Italiano di Tecnologia
 * Author: Elena Ceseracciu
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
#include <string>
#include <vector>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include "iCub/learningMachine/CovSEard.h"
using namespace yarp::sig;
using namespace yarp::math;

namespace iCub {
namespace learningmachine {

CovSEard::CovSEard(CovSEard& original):ell(original.ell), sigmaSignal(original.sigmaSignal), domain(original.domain)
{}
CovSEard& CovSEard::operator= (const CovSEard& original)
{
    if(this == &original) return *this; // handle self initialization

    this->ell=original.ell;
    this->sigmaSignal=original.sigmaSignal;
    this->domain=original.domain;
    return *this;
}
yarp::sig::Matrix CovSEard::calculateCovariance(const yarp::sig::Matrix &x)
{
    Matrix covariance(x.rows(), x.rows());
    for (size_t i=0; i<x.rows();++i)
        for (size_t j=0; j<=i; ++j) 
        {
            covariance(i, j)=sigmaSignal*exp(-0.5*(norm2((x.getRow(i)-x.getRow(j))/ell)));
          //  covariance(i, j)=exp(-0.5*sigmaSignal*(norm2((x.getRow(i)-x.getRow(j))/ell)));
            if (i!=j)
                covariance(j, i)=covariance(i, j);
        }
    return covariance;
}

yarp::sig::Matrix CovSEard::calculateCrossCovariance(const yarp::sig::Matrix &x, const yarp::sig::Matrix &xs)
{
    //TODO:assert that they have the same number of columns...
    Matrix covariance(xs.rows(), x.rows());
    for (size_t i=0; i<xs.rows();++i)
        for (size_t j=0; j<x.rows(); ++j) //calculate whole matrix, not just lower triangular part
        {
           // covariance(i, j)=exp(-0.5*sigmaSignal*(norm2((xs.getRow(i)-x.getRow(j))/ell)));
           covariance(i, j)=sigmaSignal*exp(-0.5*(norm2((xs.getRow(i)-x.getRow(j))/ell)));
        }
    return covariance;
    
}

yarp::sig::Matrix CovSEard::getDerivative(const yarp::sig::Matrix &x, size_t i)
{
    Matrix covariance=calculateCovariance(x);
    
    if (i>this->domain)
        return Matrix();
    if (i==this->domain)
        //return 2*covariance;
        return covariance; //I am using sigmasignal =sigma_f^2;
    if(i>=0)
    {
        Vector x_i=x.getCol(i);
        for (int i_row=0; i_row<covariance.rows(); ++i_row)
        {
            covariance(i_row, i_row) =0.0;
            for (int i_col=i_row+1; i_col<covariance.cols(); ++i_col)
            {
                covariance(i_col, i_row)=covariance(i_row, i_col) *= 2*(pow(x_i(i_row)-x_i(i_col), 2))/pow(ell(i), 3/2);
                
            }
            
        }
        
        
//         if dg
//       K = K*0;
//     else
//       if xeqz
//         K = K.*sq_dist(x(:,i)'/ell(i));
//       else
//         K = K.*sq_dist(x(:,i)'/ell(i),z(:,i)'/ell(i));
//       end
//     end
        
    }
    return covariance;
}

CovSEard* CovSEard::clone()
{
    return new CovSEard(*this);
}


void CovSEard::setLengthScale(yarp::sig::Vector newEll)
{
    this->ell=newEll;
}

yarp::sig::Vector CovSEard::getLengthScale()
{
    return this->ell;
}
    
void CovSEard::setSigmaSignal(double s)
{
    this->sigmaSignal=s;
}

double CovSEard::getSigmaSignal()
{
    return this->sigmaSignal;
}
    
bool CovSEard::setHyperparameters(const yarp::sig::Vector newHyperPar)
{
	if (newHyperPar.size()!=this->getNumberOfHyperparameters())
		return false;
    ell=newHyperPar.subVector(0, newHyperPar.size()-2);
    sigmaSignal=newHyperPar(newHyperPar.size()-1);
    return true;
}
yarp::sig::Vector CovSEard::getHyperparameters()
{
    return cat(ell, sigmaSignal);
}
   
int CovSEard::getNumberOfHyperparameters(){
    return domain+1;
}

} // learningmachine
} // iCub
