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


#ifndef _GURLS_MACROAVG_H_
#define _GURLS_MACROAVG_H_

#include "perf.h"

#include "utils.h"
#include "gvec.h"

#include "float.h"

namespace gurls {

/**
 * \ingroup Performance
 * \brief PerfMacroAvg is the sub-class of Performance that evaluates prediction accuracy
 */

template <typename T>
class PerfMacroAvg: public Performance<T>{

public:
    /**
     * Evaluates the average accuracy per class.
     *
     * \param X input data matrix
     * \param Y labels matrix
     * \param opt options with the following:
     *  - pred (settable with the class Prediction and its subclasses)
     *
     * \return adds the following fields to opt:
     *  - acc = array of prediction accuracy for each class
     *  - forho = acc
     *  - forplot = acc
     */
    void execute(const gMat2D<T>& X, const gMat2D<T>& Y, GurlsOptionsList& opt)  throw(gException);

protected:
    void macroavg(const unsigned long* trueY, const unsigned long* predY, const int length,int totClasses, T* &perClass, T &macroAverage, unsigned long &perClass_length);
};

template<typename T>
void PerfMacroAvg<T>::execute(const gMat2D<T>& /*X_OMR*/, const gMat2D<T>& Y_OMR, GurlsOptionsList& opt) throw(gException)
{
//    if isfield (opt,'perf')
//        p = opt.perf; % lets not overwrite existing performance measures.
//                  % unless they have the same name
//    end

    const unsigned long rows = Y_OMR.rows();
    const unsigned long cols = Y_OMR.cols();


    gMat2D<T> Y(cols, rows);
    Y_OMR.transpose(Y);


    if(!opt.hasOpt("perf"))
    {
        GurlsOptionsList* perf = new GurlsOptionsList("perf");
        opt.addOpt("perf", perf);
    }

    GurlsOptionsList* perf = GurlsOptionsList::dynacast(opt.getOpt("perf"));

    if(perf->hasOpt("acc"))
        perf->removeOpt("acc");

    if(perf->hasOpt("forho"))
        perf->removeOpt("forho");

    T* acc = new T[cols];
    T* forho = new T[cols];

//    T = size(y,2);

//    y_true = y;
    T* y_true = Y.getData();


//    y_pred = opt.pred;
    GurlsOption *pred_opt = opt.getOpt("pred");

//    if (pred_opt->getDataID() != typeid(T))
//            throw gException("Different types");

    gMat2D<T> *pred_mat = &(OptMatrix<gMat2D<T> >::dynacast(pred_opt))->getValue();
    gMat2D<T> y_pred(pred_mat->cols(), pred_mat->rows());
    pred_mat->transpose(y_pred);


    //TODO
//    for(int i=0;i<pred_mat->cols()*pred_mat->rows();i++)
//      if(abs(y_pred.getData()[i]) < opt.getOptAsNumber("smallnumber"))
//      y_pred.getData()[i]=0.0;


//    if size(y,2) == 1
    if(cols == 1)
    {
//        predlab = sign(y_pred);
        T* predLab = sign(y_pred.getData(), y_pred.getSize());

        T* tmp = compare<T>(predLab, Y.getData(), rows, &eq);

//        p.acc = mean(predlab == y);
        mean(tmp, acc, rows, 1, 1);

//        p.forho = mean(predlab == y);
        *forho = *acc;

//        p.forplot = mean(predlab == y);

        delete [] tmp;
        delete [] predLab;
    }
    else
    {
//        %% Assumes single label prediction.
//        [dummy, predlab] = max(y_pred,[],2);
//        unsigned long* predLab = indicesOfMax(y_pred.getData(), pred_mat->rows(), pred_mat->cols(), 2);
        T* work = new T[std::max(Y.getSize(), pred_mat->getSize() )];

        unsigned long* predLab = new unsigned long[rows];
        indicesOfMax(y_pred.getData(), rows, pred_mat->cols(), predLab, work, 2);

//        [dummy, truelab] = max(y_true,[],2);
//        unsigned long* trueLab = indicesOfMax(y_true, rows, cols, 2);
        unsigned long* trueLab = new unsigned long[rows];
        indicesOfMax(y_true, rows, cols, trueLab, work, 2);

        delete[] work;

//        [MacroAvg, PerClass] = macroavg(truelab, predlab);

        T macroAverage;
        T* perClass;
        unsigned long perClass_length;

        macroavg(trueLab, predLab, rows, cols, perClass, macroAverage, perClass_length);

        if(perClass_length > cols)
            throw gException(Exception_Inconsistent_Size);

        delete[] predLab;
        delete[] trueLab;


//        for t = 1:T,
//            p.acc(t) = PerClass(t);
//            p.forho(t) = p.acc(t);
//            p.forplot(t) = p.acc(t);
//        end

//        copy(acc, perClass, cols);
//        copy(forho, perClass, cols);



//        for t = 1:length(PerClass),
//            p.acc(t) = PerClass(t);
//            p.forho(t) = p.acc(t);
//            p.forplot(t) = p.acc(t);
//        end
//        for t = (length(PerClass)+1):T
//            p.acc(t) = 1;
//            p.forho(t) = 1;
//            p.forplot(t) = 1;
//        end

        copy(acc, perClass, perClass_length);
        copy(forho, perClass, perClass_length);

        if(perClass_length < cols)
        {
            set(acc+perClass_length, (T)1.0, cols-perClass_length);
            set(forho+perClass_length, (T)1.0, cols-perClass_length);
        }

    }

    gMat2D<T>* acc_mat = new gMat2D<T>(acc, 1, cols, true);
    OptMatrix<gMat2D<T> >* acc_opt = new OptMatrix<gMat2D<T> >(*acc_mat);
    perf->addOpt("acc", acc_opt);

    gMat2D<T>* forho_mat = new gMat2D<T>(forho, 1, cols, true);
    OptMatrix<gMat2D<T> >* forho_opt = new OptMatrix<gMat2D<T> >(*forho_mat);
    perf->addOpt("forho", forho_opt);

}

/**
 * Auxiliary function called by \ref execute method
 */
template<typename T>
void PerfMacroAvg<T>::macroavg(const unsigned long* trueY, const unsigned long* predY, const int length, int totClasses, T* &perClass, T &macroAverage, unsigned long &perClass_length)
{
//function [MacroAverage, PerClass] = macroavg(TrueY, PredY)
//% Computes average of performance for each class.

//% Macro
//nClasses = max(TrueY);
    int nClasses = *(std::max_element(trueY, trueY+length));

    if(nClasses < 0)
        throw gException(Exception_Inconsistent_Size);

    perClass_length = nClasses+1;
//     perClass = new T[perClass_length];
    perClass = new T[totClasses];

    unsigned long* ty_and_py = new unsigned long[length];
    unsigned long* num = new unsigned long[1];
    unsigned long* den = new unsigned long[1];

//    for i = 1:nClasses,
    for(unsigned long i=0; i<perClass_length; ++i)
    {
//    acc(i) = sum((TrueY == i) & (PredY == i))/(sum(TrueY == i) + eps);
        unsigned long* tyEqI = compare<unsigned long>(trueY, i, length, &eq);
        unsigned long* pyEqI = compare<unsigned long>(predY, i, length, &eq);

        mult(tyEqI, pyEqI, ty_and_py, length);

        sum(ty_and_py, num, length, 1, 1);
        sum(tyEqI, den, length, 1, 1);

        perClass[i] = ((T)(*num))/((*den) + std::numeric_limits<T>::epsilon());

        delete [] tyEqI;
        delete [] pyEqI;
    }

    delete [] ty_and_py;
    delete [] num;
    delete [] den;

    //set accuracy =1 on classes with no samples
    for(int i=perClass_length; i<totClasses; ++i)
      perClass[i] = 1;


//PerClass = acc;

//MacroAverage = mean(acc);
    T* meanValue = new T[1];
    mean(perClass, meanValue, nClasses+1, 1, 1);

    macroAverage = *meanValue;

    delete[] meanValue;
}

}

#endif //_GURLS_MACROAVG_H_
