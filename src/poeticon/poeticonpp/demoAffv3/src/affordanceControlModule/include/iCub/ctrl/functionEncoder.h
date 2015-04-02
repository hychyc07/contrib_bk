/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
 * \defgroup functionEncoder functionEncoder
 *  
 * @ingroup ctrlLib
 *
 * Classes for encoding functions
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __FUNCTIONENCODER_H__
#define __FUNCTIONENCODER_H__

#include <yarp/sig/Vector.h>


namespace iCub
{

namespace ctrl
{

/**
* \ingroup functionEncoder
*
* Encodes any given function as a set of wavelet coefficients. 
* The father wavelet used here is the \b db4.
*/
class waveletEncoder
{
protected:
    void *w;
    void *F;
    yarp::sig::Vector *pVal;

    unsigned int iCoeff;
    double Resolution;

    double interpWavelet(const double x);
    double interpFunction(const yarp::sig::Vector &Val, const double x);

    friend double integrand(double x, void *params);

public:
    /**
    * Constructor. 
    */
    waveletEncoder();

    /**
    * Encodes the function.
    * @param Val is the vector containing the samples of function 
    *            \b f to be encoded. The \e x coordinates of the
    *            points are intended to be normalized in [0,1], so
    *            that f(0)=Val[0] and f(1)=Val[Val.length()-1].
    * @param R is the resolution to which the encoding is computed. 
    * @return the computed 1+N coefficients, with the first one 
    *         being f(0) and the following N are the actual wavelet
    *         expansion coefficients.
    * @note It holds that N=floor(R)+1, where N is the number of
    *       coefficients of the vector space. Recap that floor() is
    *       the round function towards minus infinity.
    */
    virtual yarp::sig::Vector encode(yarp::sig::Vector &Val, double R);

    /**
    * Computes the approximated value of function in x, given the 
    * input set of wavelet coefficients. 
    * @param Coeffs is the wavelet coefficients vector along with 
    *               the first initial value f(0).
    * @param R is the resolution to which the deconding is computed. 
    * @param x is the point at which the result is computed. It 
    *          shall be in [0,1].
    * @return the decoded function value. 
    * @note It shall hold that Coeffs.length()>=floor(R)+2. Recap 
    *       that floor() is the round function towards minus
    *       infinity.
    */
    virtual double decode(const yarp::sig::Vector &Coeffs, double R, const double x);

    /**
    * Destructor. 
    */
    ~waveletEncoder();
};

}

}

#endif



