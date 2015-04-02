/*
 * The GURLS Package in C++
 *
 * Copyright (C) 2011, IIT@MIT Lab
 * All rights reserved.
 *
 * authors:  M. Santoro
 * email:   msantoro@mit.edu
 * website: http://cbcl.mit.edu/IIT@MIT/IIT@MIT.html
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name(s) of the copyright holders nor the names
 *       of its contributors or of the Massacusetts Institute of
 *       Technology or of the Italian Institute of Technology may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef _GURLS_CONFGAP_H_
#define _GURLS_CONFGAP_H_


#include "confidence.h"
#include "gmath.h"

namespace gurls {

/**
 * \ingroup Confidence
 * \brief ConfGap is the sub-class of Confidence that computes a confidence estimation for the predicted class.
 */

template <typename T>
class ConfGap: public Confidence<T>
{
public:
    /**
     * Computes a confidence estimation for the predicted class (i.e. highest scoring class). The difference between the highest scoring class and the second highest scoring class is used as an estimate.
     * \param X not used
     * \param Y not used
     * \param opt options with the following:
     *  - pred (settable with the class Prediction and its subclasses)
     *
     * \return adds the following fields to opt:
     *  - confidence = array containing the confidence score for each row of the field pred of opt.
     */
    void execute(const gMat2D<T>& X, const gMat2D<T>& Y, GurlsOptionsList& opt)  throw(gException);
};

template<typename T>
void ConfGap<T>::execute(const gMat2D<T>& /*X*/, const gMat2D<T>& /*Y_OMR*/, GurlsOptionsList& opt) throw(gException)
{
//   out = struct;
//   [n,k] = size(opt.pred);
     GurlsOption *pred_opt = opt.getOpt("pred");
     gMat2D<T> *pred_mat = &(OptMatrix<gMat2D<T> >::dynacast(pred_opt))->getValue();
     int n = pred_mat->rows();
     int t = pred_mat->cols();
     gMat2D<T> y_pred(t, n);
     pred_mat->transpose(y_pred);
     T* expscoresTranspose = new T[n*t];
     copy< T >(expscoresTranspose,y_pred.getData(),n*t,1,1);

//      out.confidence = opt.pred;
//      out.confidence = sort(out.confidence,2,'descend');
//      out.confidence = out.confidence(:,1) - out.confidence(:,2);

     //TODO optmize search of two maxes
//      for(int i=0;i<n;i++)
//        std::sort(expscoresTranspose + t*i,expscoresTranspose + t*i +t);

     T* confidence = new T[n];
     T* rowT = new T[t];
     for(int i=0;i<n;i++)
     {
       getRow< T >(expscoresTranspose,n,t,i,rowT);
       std::sort(rowT,rowT+t);
       confidence[i] =  rowT[t-1]-rowT[t-2];
     }


     gVec< T > conf(confidence, n, false);


     opt.addOpt("confidence", new OptMatrix<gMat2D<T> >(conf.asMatrix()));
     delete [] expscoresTranspose;
     delete [] confidence;
     delete [] rowT;
}

}

#endif //_GURLS_CONFGAP_H_
