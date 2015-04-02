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


#ifndef _GURLS_SIGLAMHOGPREGR_H_
#define _GURLS_SIGLAMHOGPREGR_H_

#include <cmath>


#include "options.h"
#include "optlist.h"
#include "gmat2d.h"
#include "gvec.h"
#include "gmath.h"

#include "paramsel.h"
#include "hogpregr.h"
#include "rbfkernel.h"

namespace gurls {

/**
 * \ingroup ParameterSelection
 * \brief ParamSelSiglamHoGPRegr is the sub-class of ParamSelection that implements
 */

template <typename T>
class ParamSelSiglamHoGPRegr: public ParamSelection<T>{

public:
    /**
     *
     */
   void execute(const gMat2D<T>& X, const gMat2D<T>& Y, GurlsOptionsList& opt);
};

template <typename T>
void ParamSelSiglamHoGPRegr<T>::execute(const gMat2D<T>& X_OMR, const gMat2D<T>& Y_OMR, GurlsOptionsList& opt)
{
    gMat2D<T> X(X_OMR.cols(), X_OMR.rows());
    X_OMR.transpose(X);

    gMat2D<T> Y(Y_OMR.cols(), Y_OMR.rows());
    Y_OMR.transpose(Y);

//    [n,T]  = size(y);
    const unsigned long n = Y_OMR.rows();
    const unsigned long t = Y_OMR.cols();

    const unsigned long d = X_OMR.cols();

//    if ~isfield(opt,'kernel')
    if(!opt.hasOpt("kernel"))
    {
//        opt.kernel.type = 'rbf';
        GurlsOptionsList* kernel = new GurlsOptionsList("kernel");
        kernel->addOpt("type", "rbf");

        opt.addOpt("kernel", kernel);
    }

    GurlsOptionsList* kernel = GurlsOptionsList::dynacast(opt.getOpt("kernel"));

//    if ~isfield(opt.kernel,'distance')
    if(!kernel->hasOpt("distance"))
    {
//        opt.kernel.distance = squareform(pdist(X)).^2;
        gMat2D<T> *dist = new gMat2D<T>(n, n);

        squareform(X.getData(), n, d, dist->getData(), n);

        mult(dist->getData(), dist->getData(), dist->getData(), n*n);

        kernel->addOpt("distance", new OptMatrix<gMat2D<T> >(*dist));
    }

    gMat2D<T> &distance = OptMatrix<gMat2D<T> >::dynacast(kernel->getOpt("distance"))->getValue();

//    if ~isfield(opt,'sigmamin')
    if(!opt.hasOpt("sigmamin"))
    {
//        D = sort(squareform(opt.kernel.distance.^(1/2)));

        int d_len = n*(n-1)/2;
        T* distLinearized = new T[d_len];

        const int size = distance.cols();
        T* it = distLinearized;
        T* d_it = distance.getData();

        for(int i=1; i< size; ++i)
        {
            gurls::copy(it , d_it+i, size - i);

            it += size - i;
            d_it += size;
        }

        std::sort(distLinearized, distLinearized + d_len);

        //        firstPercentile = round(0.01*numel(D)+0.5);
        int firstPercentile = gurls::round((T)0.01 * d_len + (T)0.5);

        // 	opt.sigmamin = D(firstPercentile);
        opt.addOpt("sigmamin", new OptNumber(sqrt( distLinearized[firstPercentile]) ));

        delete [] distLinearized;
    }

//    if ~isfield(opt,'sigmamax')
    if(!opt.hasOpt("sigmamax"))
    {
//        opt.sigmamax = max(max(opt.kernel.distance.^(1/2)));
        double sigmaMax = sqrt(*std::max_element(distance.getData(), distance.getData()+distance.getSize()));
        opt.addOpt("sigmamax", new OptNumber(sqrt(sigmaMax)));
    }

//    if opt.sigmamin <= 0
    if(opt.getOptAsNumber("sigmamin") <= 0.0)
    {
//        opt.sigmamin = eps;
        opt.removeOpt("sigmamin");
        opt.addOpt("sigmamin", new OptNumber(std::numeric_limits<T>::epsilon()));
    }
//    if opt.sigmamax <= 0
    if(opt.getOptAsNumber("sigmamax") <= 0.0)
    {
//        opt.sigmamax = eps;
        opt.removeOpt("sigmamax");
        opt.addOpt("sigmamax", new OptNumber(std::numeric_limits<T>::epsilon()));
    }

    const int nsigma = static_cast<int>(opt.getOptAsNumber("nsigma"));
    const int nlambda = static_cast<int>(opt.getOptAsNumber("nlambda"));
    const T sigmamin = static_cast<T>(opt.getOptAsNumber("sigmamin"));

//    q = (opt.sigmamax/opt.sigmamin)^(1/(opt.nsigma-1));
    T q = static_cast<T>(std::pow(opt.getOptAsNumber("sigmamax")/opt.getOptAsNumber("sigmamin"), 1.0/(nsigma-1.0)));

//    perf = zeros(opt.nsigma,opt.nlambda,T);
    T* perf = new T[nsigma*nlambda];
    set(perf, (T)0.0, nsigma*nlambda);

//    sigmas = zeros(1,opt.nsigma);
    T* sigmas = new T[nsigma];

    T* guesses = new T[nsigma*nlambda];

    KernelRBF<T> rbf;
    ParamSelHoGPRegr<T> hogp;


    GurlsOptionsList* paramsel_rbf = new GurlsOptionsList("paramsel");

//    for i = 1:opt.nsigma
    for(int i=0; i<nsigma; ++i)
    {
//        sigmas(i) = (opt.sigmamin*(q^(i-1)));
        const T sigma = sigmamin* std::pow(q, i);
        sigmas[i] = sigma;

//        opt.paramsel.sigma = sigmas(i);
        paramsel_rbf->removeOpt("sigma");
        paramsel_rbf->addOpt("sigma", new OptNumber(sigma));

        opt.addOpt("paramsel", paramsel_rbf);

//        opt.kernel = kernel_rbf(X,y,opt);
        rbf.execute(X_OMR, Y_OMR, opt);

        opt.removeOpt("paramsel", false);

//        paramsel = paramsel_hogpregr(X,y,opt);
        hogp.execute(X_OMR, Y_OMR, opt);

        GurlsOptionsList* paramsel_hogp = GurlsOptionsList::dynacast(opt.getOpt("paramsel"));

//        perf(i,:,:) = paramsel.perf{1};
        gMat2D<T> &perf_mat = OptMatrix<gMat2D<T> >::dynacast(paramsel_hogp->getOpt("perf"))->getValue();
        axpy(nsigma*nlambda, (T)1.0, perf_mat.getData(), 1, perf, 1);

//        guesses(i,:) = paramsel.guesses{1};
        gMat2D<T> &guesses_mat = OptMatrix<gMat2D<T> >::dynacast(paramsel_hogp->getOpt("guesses"))->getValue();
        copy(guesses+i, guesses_mat.getData(), nlambda, nsigma, 1);

        opt.removeOpt("paramsel");
    }

    delete paramsel_rbf;

//    M = sum(perf,3); % sum over classes

//    [dummy,i] = max(M(:));
    int i = std::max_element(perf, perf +(nsigma*nlambda)) - perf;

//    [m,n] = ind2sub(size(M),i);
    int im = i%nlambda;
    int in = static_cast<int>(i/nlambda);

    GurlsOptionsList* paramsel = new GurlsOptionsList("paramsel");

//    % opt sigma
//    vout.sigma = opt.sigmamin*(q^(m-1));
    paramsel->addOpt("sigma", new OptNumber( sigmamin*(std::pow(q, im))) );

//    % opt lambda
//    vout.noises = guesses(m,n)*ones(1,T);
    OptNumberList *lambdas = new OptNumberList();
    const double lambda = static_cast<double>(guesses[im + in*nsigma]);

    for(unsigned long j=0; j<t; ++j)
        lambdas->add(lambda);

    paramsel->addOpt("lambdas", lambdas);

    opt.addOpt("paramsel", paramsel);

}


}

#endif // _GURLS_SIGLAMHOGPREGR_H_
