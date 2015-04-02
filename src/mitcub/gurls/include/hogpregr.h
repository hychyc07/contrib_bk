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


#ifndef _GURLS_HOGPREGR_H_
#define _GURLS_HOGPREGR_H_

#include <cmath>

#include "options.h"
#include "optlist.h"
#include "gmat2d.h"
#include "gvec.h"
#include "gmath.h"

#include "paramsel.h"
#include "perf.h"

#include "rlsgp.h"
#include "predgp.h"

namespace gurls {

/**
 * \ingroup ParameterSelection
 * \brief ParamSelHoGPRegr is the sub-class of ParamSelection that implements
 */

template <typename T>
class ParamSelHoGPRegr: public ParamSelection<T>{

public:
    /**
     *
     */
   void execute(const gMat2D<T>& X, const gMat2D<T>& Y, GurlsOptionsList& opt);
};

template <typename T>
void ParamSelHoGPRegr<T>::execute(const gMat2D<T>& X_OMR, const gMat2D<T>& Y_OMR, GurlsOptionsList& opt)
{

    gMat2D<T> X(X_OMR.cols(), X_OMR.rows());
    X_OMR.transpose(X);

    gMat2D<T> Y(Y_OMR.cols(), Y_OMR.rows());
    Y_OMR.transpose(Y);

    //    [n,T]  = size(y);
    const unsigned long n = Y_OMR.rows();
    const unsigned long t = Y_OMR.cols();

    const unsigned long d = X_OMR.cols();

//    tot = opt.nlambda;
    int tot = static_cast<int>(opt.getOptAsNumber("nlambda"));


//    K = opt.kernel.K;
    GurlsOptionsList *kernel = GurlsOptionsList::dynacast(opt.getOpt("kernel"));
    gMat2D<T> &K_mat = OptMatrix<gMat2D<T> >::dynacast(kernel->getOpt("K"))->getValue();

    gMat2D<T> K(K_mat.cols(), K_mat.rows());
    K_mat.transpose(K);


    opt.removeOpt("kernel", false);
    GurlsOption* predKernel = NULL;
    if(opt.hasOpt("predkernel"))
    {
        predKernel = opt.getOpt("predkernel");
        opt.removeOpt("predkernel", false);
    }

    GurlsOption* optimizer = NULL;
    if(opt.hasOpt("optimizer"))
    {
        optimizer = opt.getOpt("optimizer");
        opt.removeOpt("optimizer", false);
    }


    GurlsOptionsList* split = GurlsOptionsList::dynacast(opt.getOpt("split"));

    gMat2D< unsigned long > &indices_mat = OptMatrix<gMat2D< unsigned long > >::dynacast(split->getOpt("indices"))->getValue();
    unsigned long* indices = new unsigned long[indices_mat.cols()*indices_mat.rows()];
    transpose(indices_mat.getData(), indices_mat.cols(), indices_mat.rows(), indices);

    gMat2D< unsigned long > &lasts_mat = OptMatrix<gMat2D< unsigned long > >::dynacast(split->getOpt("lasts"))->getValue();
    unsigned long *lasts = lasts_mat.getData();

    std::cout << indices_mat << std::endl;

    std::cout << lasts_mat << std::endl;
    unsigned long *tr = new unsigned long[indices_mat.cols()];
    unsigned long *va;


    T* work = new T[t+n];

//    lmax = mean(std(y));
    T* stdY = new T[t];

    stdDev(Y.getData(), n, t, stdY, work);

    const T lmax = sumv(stdY, t, work)/((T)t);

//    lmin = mean(std(y))/10^-5;
    const T lmin = lmax / (T)1.0e-5;

    delete[] stdY;
    delete[] work;

//    guesses = lmin.*(lmax/lmin).^linspace(0,1,tot);
    T* guesses = new T[tot];

    T* linspc = new T[tot];
    linspace((T)0.0, (T)1.0, tot, linspc);
    const T coeff = lmax/lmin;

    for(int i=0; i< tot; ++i)
        guesses[i] = lmin* std::pow(coeff, linspc[i]);

    delete[] linspc;

    GurlsOptionsList* tmpPredKernel = new GurlsOptionsList("predkernel");
    GurlsOptionsList* tmpKernel = new GurlsOptionsList("kernel");
    GurlsOptionsList* tmpParamSel = new GurlsOptionsList("paramsel");

    opt.addOpt("kernel", tmpKernel);
    opt.addOpt("predkernel", tmpPredKernel);
    opt.addOpt("paramsel", tmpParamSel);

    gMat2D<T> subXtr;
    gMat2D<T> subYtr;
    gMat2D<T> subXva;
    gMat2D<T> subYva;

    gMat2D<T>* subK = new gMat2D<T>();
    gMat2D<T>* subPredK = new gMat2D<T>();
    gMat2D<T>* subPredKTest = new gMat2D<T>();

    tmpKernel->addOpt("K", new OptMatrix<gMat2D<T> > (*subK));
    tmpPredKernel->addOpt("K", new OptMatrix<gMat2D<T> > (*subPredK));
    tmpPredKernel->addOpt("Ktest", new OptMatrix<gMat2D<T> > (*subPredKTest));


    RLSGPRegr<T> rlsgp;
    PredGPRegr<T> predgp;
    Performance<T>* perfClass = Performance<T>::factory(opt.getOptAsString("hoperf"));

    const int nholdouts = static_cast<int>(opt.getOptAsNumber("nholdouts"));
    const unsigned long indices_cols = indices_mat.cols();

    T *perf = new T[tot*t];
    T *lambdas_round = new T[nholdouts*t];
    gMat2D<T> *perf_mat = new gMat2D<T>(nholdouts, tot*t);
    T *ret_guesses = new T [nholdouts*tot];


//    for nh = 1:opt.nholdouts
    for(int nh = 0; nh < nholdouts; ++nh)
    {

//        if strcmp(class(opt.split),'cell')
//            tr = opt.split{nh}.tr;
//            va = opt.split{nh}.va;
//        else
//            tr = opt.split.tr;
//            va = opt.split.va;
//        end
        unsigned long last = lasts[nh];
        copy(tr, indices+nh, indices_cols, 1, indices_mat.rows());
        va = tr+last;
        const unsigned long va_size = indices_cols-last;

//        [n,T]  = size(y(tr,:));

        // here n is last

        T* tmpMat = new T[std::max(last, va_size) * std::max(va_size, std::max(t, d))];

//        opt.kernel.K = K(tr,tr);
        subK->resize(last, last);
        copy_submatrix(subK->getData(), K.getData(), K_mat.rows(), last, last, tr, tr);


//        opt.predkernel.K = K(va,tr);
        subPredK->resize(va_size, last);
        copy_submatrix(tmpMat, K.getData(), K_mat.rows(), va_size, last, va, tr);
        transpose(tmpMat, va_size, last, subPredK->getData());

//        opt.predkernel.Ktest = diag(K(va,va));
        subPredKTest->resize(va_size, 1);
        copy_submatrix(tmpMat, K.getData(), K_mat.rows(), va_size, va_size, va, va);
        copy(subPredKTest->getData(), tmpMat, va_size, 1, va_size+1);


        subXtr.resize(last, d);
        subMatrixFromRows(X.getData(), n, d, tr, last, tmpMat);
        transpose(tmpMat, last, d, subXtr.getData());

        subYtr.resize(last, t);
        subMatrixFromRows(Y.getData(), n, t, tr, last, tmpMat);
        transpose(tmpMat, last, t, subYtr.getData());

        subXva.resize(va_size, d);
        subMatrixFromRows(X.getData(), n, d, va, va_size, tmpMat);
        transpose(tmpMat, va_size, d, subXva.getData());

        subYva.resize(va_size, t);
        subMatrixFromRows(Y.getData(), n, t, va, va_size, tmpMat);
        transpose(tmpMat, va_size, t, subYva.getData());

        delete[] tmpMat;

//        for i = 1:tot
        for(int i=0; i< tot; ++i)
        {
//            opt.paramsel.noises = guesses(i);
            tmpParamSel->removeOpt("lambdas");
            tmpParamSel->addOpt("lambdas", new OptNumberList(guesses[i]));

//            opt.rls = rls_gpregr(X(tr,:),y(tr,:),opt);
            rlsgp.execute(subXtr, subYtr, opt);

//            tmp = pred_gpregr(X(va,:),y(va,:),opt);
            predgp.execute(subXva, subYva, opt);

//            opt.pred = tmp.means;
            opt.removeOpt("optimizer");
            GurlsOptionsList * pred_list = GurlsOptionsList::dynacast(opt.getOpt("pred"));
            opt.removeOpt("pred", false);

            opt.addOpt("pred", pred_list->getOpt("means"));
            pred_list->removeOpt("means", false);
            delete pred_list;


//            opt.perf = opt.hoperf([],y(va,:),opt);
            perfClass->execute(subXva, subYva, opt);

            opt.removeOpt("pred");

            GurlsOptionsList * perf_list = GurlsOptionsList::dynacast(opt.getOpt("perf"));
            gMat2D<T>& forho = OptMatrix<gMat2D<T> >::dynacast(perf_list->getOpt("forho"))->getValue();

//            for t = 1:T
//                perf(i,t) = opt.perf.forho(t);
            copy(perf+i, forho.getData(), t, tot, 1);


            opt.removeOpt("perf");
        }

//        [dummy,idx] = max(perf,[],1);
        work = NULL;
        unsigned long* idx = new unsigned long[t];
        indicesOfMax(perf, tot, t, idx, work, 1);

//        vout.lambdas_round{nh} = guesses(idx);
        T* lambdas_nh = copyLocations(idx, guesses, t, tot);
        copy(lambdas_round + nh, lambdas_nh, t, nholdouts, 1);
        delete[] lambdas_nh;

//        vout.perf{nh} = perf;
        transpose(perf, tot, t, perf_mat->getData()+(nh*tot*t));

//        vout.guesses{nh} = guesses;
        copy(ret_guesses+nh, guesses, tot, nholdouts, 1);

    }

    opt.removeOpt("kernel");
    opt.removeOpt("predkernel");
    opt.removeOpt("paramsel");

    delete[] guesses;
    delete perfClass;
    delete[] perf;


    opt.addOpt("kernel", kernel);
    if(predKernel != NULL)
        opt.addOpt("predkernel", predKernel);

    if(optimizer != NULL)
        opt.addOpt("optimizer", optimizer);


    GurlsOptionsList* paramsel = new GurlsOptionsList("paramsel");
    opt.addOpt("paramsel", paramsel);


    gMat2D<T>* lambdas_round_mat = new gMat2D<T>(nholdouts, t);
    transpose(lambdas_round, nholdouts, t, lambdas_round_mat->getData());
    paramsel->addOpt("lambdas_round", new OptMatrix<gMat2D<T> >(*lambdas_round_mat));

    paramsel->addOpt("perf", new OptMatrix<gMat2D<T> >(*perf_mat));


    gMat2D<T>* guesses_mat = new gMat2D<T>(nholdouts, tot);
    transpose(ret_guesses, nholdouts, tot, guesses_mat->getData());
    paramsel->addOpt("guesses", new OptMatrix<gMat2D<T> >(*guesses_mat));
    delete [] ret_guesses;


    T *lambdas = NULL;
    T *it, *end;

//    if numel(vout.lambdas_round) > 1
    if(nholdouts>1)
    {
        lambdas = new T[t];
//        lambdas = cell2mat(vout.lambdas_round');
//        vout.lambdas = mean(lambdas);
        mean(lambdas_round, lambdas, nholdouts, t, t);

        it = lambdas;
        end = lambdas+t;
    }
    else
    {
//        vout.lambdas = vout.lambdas_round{1};
        it = lambdas_round;
        end = lambdas_round+t;
    }

    OptNumberList* l = new OptNumberList();
    for(; it != end; ++it)
        l->add(static_cast<double>(*it));
    paramsel->addOpt("lambdas", l);

    if(lambdas != NULL)
        delete[] lambdas;

    delete[] lambdas_round;
}


}

#endif // _GURLS_HOGPREGR_H_
