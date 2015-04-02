/*
 * The GURLS Package in C++
 *
 * Copyright (C) 2011, IIT@MIT Lab
 * All rights reserved.
 *
 * author:  M. Santoro
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

/**
 * \ingroup Common
 * \file
 * \brief Utility functions
 */

#ifndef _GURLS_UTILS_H_
#define _GURLS_UTILS_H_

#include <map>
#include <algorithm>

#include "gmath.h"
#include "gmat2d.h"

#include "optlist.h"
#include "primal.h"
#include "macroavg.h"

/** "Equals" operator for float values*/
#define FLT_EQUALS(flt1, flt2) \
    ( flt1-flt2 >= -FLT_EPSILON && flt1-flt2 <= FLT_EPSILON )

/** "Greater than" operator for float values*/
#define FLT_GT(a, b)\
    ((a - b) > ( std::min(fabs(a), fabs(b))* FLT_EPSILON))

/** "Greater than" operator for float or double values*/
#define GT(a, b)\
    ((a - b) > ( std::min(fabs(a), fabs(b))* std::numeric_limits<T>::epsilon()))

namespace gurls {

/**
 * Utility function called by the class PrecisionRecall to evaluate the average precision through precision and recall.
 *
 * \param out vector of predicted labels
 * \param gt vector of true labels
 * \param N size of out and gt
 *
 * \return average precision
 */
template <typename T>
double precrec_driver(T* out, T* gt, unsigned long N)
{
    std::multimap<T, T> data;
    for (unsigned long i = 0; i < N; i++){
        data.insert(std::pair<T,T>(-out[i], gt[i]));
    }

    T* tp = new T[N];
    T* fp = new T[N];
    T* prec = new T[N];
    T* rec = new T[N];
    unsigned long tpcumsum = 0;
    unsigned long fpcumsum = 0;

    typename std::multimap<T, T>::iterator it = data.begin();
    typename std::multimap<T, T>::iterator end = data.end();


    unsigned long idx = 0;
    while (it != end) {
        tp[idx] = static_cast<T>(it->second > 0 ? tpcumsum+=1 : tpcumsum);
        fp[idx] = static_cast<T>(it->second < 0 ? fpcumsum+=1 : fpcumsum);
        it++;
        idx++;
    }

    for (idx = 0; idx < N; idx++){
        rec[idx] = tp[idx]/tpcumsum;
        prec[idx] = tp[idx]/(tp[idx]+fp[idx]);
    }

    // compute average precision
    T ap = 0;
    T p = 0.0;
    T t = 0.0;

    const int stepsNumber = 11;
    const T incr = static_cast<T>(0.1);

    for(int steps = 0; steps<stepsNumber; ++steps)
    {
        p = 0.0;
        for (idx = 0; idx < N; idx++){
            if (FLT_GT(rec[idx], t) || FLT_EQUALS(rec[idx], t)){
                p = std::max(p, prec[idx]);
            }
        }
        ap += p;
        t += incr;
    }
    delete [] tp;
    delete [] fp;
    delete [] prec;
    delete [] rec;
    return ap/=stepsNumber;
}

/**
 * Utility function called by the class RLSPegasos to implement a single pass for pegasos algorithm, performing the stochastic gradient descent over all training samples once
 *
 * \param X input data matrix
 * \param bY labels matrix
 * \param opt options
 * \param X_rows number of rows in X
 * \param X_cols number of columns in X     *
 * \param bY_rows number of rows in bY
 * \param bY_cols number of columns in bY
 *
 * \return updates the the field optimizer in opt by changing the following fields:
 *  - W = matrix of coefficient vectors of rls estimator for each class
 *  - W_sum = sum of the classifiers across iterations
 *  - count = number of iterations
 *  - acc_last = accuracy of the solution computed in the last iteration
 *  - acc_avg = average accuracy across iterations
 */
template <typename T>
void rls_pegasos_driver(const T* X, const T* bY, GurlsOptionsList& opt,
                        const int X_rows, const int X_cols,
                        const int bY_rows, const int bY_cols)
{

    //  lambda = opt.singlelambda(opt.paramsel.lambdas);
    //  std::vector<double> ll = OptNumberList::dynacast(opt.getOpt("lambdas"))->getValue();
    GurlsOptionsList* paramsel = GurlsOptionsList::dynacast(opt.getOpt("paramsel"));
    std::vector<double> ll = OptNumberList::dynacast(paramsel->getOpt("lambdas"))->getValue();
    T lambda = static_cast<T>((OptFunction::dynacast(opt.getOpt("singlelambda")))->getValue(&(*(ll.begin())), ll.size()));

    //            %% Inputs

    //            [n,d] = size(X);
    const int n = X_rows;
    const int d = X_cols;

    //            [T] = size(bY,2);
    const int t = bY_cols;

    //            %% Initialization
    //            cfr = opt.cfr;

    //            W = cfr.W; %dxT
    GurlsOptionsList* optimizer = GurlsOptionsList::dynacast(opt.getOpt("optimizer"));

    //     GurlsOption *W_opt = opt.getOpt("W");
    GurlsOption *W_opt = optimizer->getOpt("W");
    gMat2D<T> *W_mat = &(OptMatrix<gMat2D<T> >::dynacast(W_opt))->getValue();
    gMat2D<T> W(W_mat->cols(), W_mat->rows());
    W_mat->transpose(W);

    //    T* W = new T[d*t];

    //            W_sum = cfr.W_sum;
    //     GurlsOption *W_sum_opt = opt.getOpt("W_sum");
    GurlsOption *W_sum_opt = optimizer->getOpt("W_sum");
    gMat2D<T> *W_sum_mat = &(OptMatrix<gMat2D<T> >::dynacast(W_sum_opt))->getValue();
    gMat2D<T> W_sum(W_sum_mat->cols(), W_sum_mat->rows());
    W_sum_mat->transpose(W_sum);

    //            count = cfr.count; %contatore per numero iterazioni totali, prende quello in input e lo aggiorna
    //     int count = static_cast<int>(opt.getOptAsNumber("count"));
    int count = static_cast<int>(optimizer->getOptAsNumber("count"));

    //            t0 = cfr.t0;
    //     T t0 = static_cast<T>(opt.getOptAsNumber("t0"));
    T t0 = static_cast<T>(optimizer->getOptAsNumber("t0"));


    unsigned long * seq = new unsigned long[n];
    T* xt = new T[d];
    T* y_hat = new T[t];
    T* r = new T[t];
    const int W_size = d*t;
    T* xtr = new T[W_size];

    //            %% Initialization
    //            iter = 0;
    int iter;

    const T thr = sqrt(t/lambda);

    //            seq = randperm(n); %riordina 1:n in modo casuale
    randperm(n, seq);

    for(iter = 0; iter<n; ++iter)
    {
        //            while iter < n,
        //                iter = iter + 1;
        //                idx = seq(iter);
        const unsigned long idx = seq[iter];

        //                %% Stepsize

        //                %% Update Equations
        //                xt = X(idx,:); %1xd
        getRow(X, n, d, idx, xt);

        //                y_hat = (xt*W); %1xT
        dot(xt, W.getData(), y_hat, 1, d, d, t, 1, t, CblasNoTrans, CblasNoTrans, CblasColMajor);

        //                r = bY(idx,:) - y_hat; %1xT
        getRow(bY, bY_rows, t, idx, r);
        axpy(t, (T)-1.0, y_hat, 1, r, 1);


        //                eta = 1.0/(lambda*(count + t0));
        const T eta = ((T)1.0)/(lambda*(count + t0));

        //                W = (1 - lambda*eta)*W + eta*xt'*r; %dxT
        const T coeff = (T)1.0 - (lambda*eta);

        //        dot(xt, r, xtr, d, 1, 1, t, d, t, CblasNoTrans, CblasNoTrans, CblasColMajor);

        //        set(xtr, (T)0.0, W_size);
        //        axpy(W_size, coeff, W.getData(), 1, xtr, 1);
        //        copy(W.getData(), xtr, W_size);

        scal(W_size, coeff, W.getData(), 1);

        dot(xt, r, xtr, d, 1, 1, t, d, t, CblasNoTrans, CblasNoTrans, CblasColMajor);

        axpy(W_size, eta, xtr, 1, W.getData(), 1);

        //                %% Projection onto the ball with radius sqrt(T/lambda)
        //                nW = norm(W,'fro'); %Frobenius norm

        T tmp = dot(W_size, W.getData(), 1, W.getData(), 1);
        T  nW = sqrt(tmp);


        //                if nW > sqrt(T/lambda)
        if( GT(nW, thr) )
        {
            //                    W = (W/nW)*sqrt(T/lambda);
            set(xtr, (T)0.0, W_size);
            axpy(W_size, thr/nW, W.getData(), 1, xtr, 1);
            copy(W.getData(), xtr, W_size);
        }

        //                %% Averaging

        //                W_sum = W_sum + W;
        axpy(W_size, (T)1.0, W.getData(), 1, W_sum.getData(), 1);

        //                count = count + 1;
        ++count;

        //                %% Testing
        //                if(mod(count,n) == 1)
        //                    fprintf('\n\tObjective : %f',obj_primal(W, X, bY, lambda));
        //                    cfr.acc_last(end+1) = test_classifier (W,opt);
        //                    fprintf('\n\tLast Acc: %f', cfr.acc_last(end));
        //                    cfr.acc_avg(end+1) = test_classifier (W_sum/count,opt);
        //                    fprintf('\n\tAvg Acc: %f\n', cfr.acc_avg(end));
        //                end

    }

    delete[] seq;
    delete[] xt;
    delete[] y_hat;
    delete[] r;
    delete[] xtr;

    //            cfr.W = W;
    //     W.transpose(*W_mat);

    //            cfr.W_last = W;
    //     if(opt.hasOpt("W_last"))
    //     {
    //         GurlsOption *W_last_opt = opt.getOpt("W_last");
    //         gMat2D<T> *W_last = &(OptMatrix<gMat2D<T> >::dynacast(W_last_opt))->getValue();
    //         copy(W_last->getData(), W_mat->getData(), W_size);
    //     }
    //     else
    //     {
    //         gMat2D<T> *W_last = new gMat2D<T>(*W_mat);
    //         opt.addOpt("W_last", new OptMatrix<gMat2D<T> >(*W_last));
    //     }

    W.transpose(*W_mat);
    //            cfr.W_sum = W_sum;
    W_sum.transpose(*W_sum_mat);

    //            cfr.count = count;
    //     opt.removeOpt("count");
    //     opt.addOpt("count", new OptNumber(count));
    optimizer->removeOpt("count");
    optimizer->addOpt("count", new OptNumber(count));

    //            cfr.iter = iter;
    //     opt.removeOpt("iter");
    //     opt.addOpt("iter", new OptNumber(iter));
    optimizer->removeOpt("iter");
    optimizer->addOpt("iter", new OptNumber(iter));

    //	cfr.C = [];
    //     opt.removeOpt("C");
    optimizer->removeOpt("C");
    gMat2D<T>* emptyC = new gMat2D<T>();
    //     opt.addOpt("C", new OptMatrix<gMat2D<T> >(*emptyC));
    optimizer->addOpt("C", new OptMatrix<gMat2D<T> >(*emptyC));

    //	cfr.X = [];
    //     opt.removeOpt("X");
    optimizer->removeOpt("X");
    gMat2D<T>* emptyX = new gMat2D<T>();
    //     opt.addOpt("X", new OptMatrix<gMat2D<T> >(*emptyX));
    optimizer->addOpt("X", new OptMatrix<gMat2D<T> >(*emptyX));

    opt.removeOpt("optimizer", false);
    opt.addOpt("optimizer",optimizer);

}


/**
 * Utility function called by the rls_pegasos_driver; it evaluate classification accuracy on the test set given in fields Xte and yte of opt
 *
 * \param W coefficient vector for a linear classifier
 * \param opt options with the followng fields:
 *  - Xte
 *  - yte
 * \param rows number of rows in W
 * \param cols number of columns in W
 *
 * \return res classification accuracy
 */
// function [acc] = test_classifier (W, opt)
template <typename T>
T test_classifier(T* W, GurlsOptionsList& opt, const int rows, const int cols)
{
    //opt.rls.W = W;

    GurlsOptionsList* optimizer = GurlsOptionsList::dynacast(opt.getOpt("optimizer"));

    gMat2D<T> *W_mat = NULL;
    if(!optimizer->hasOpt("W"))
    {
        W_mat = new gMat2D<T>(rows,cols);
        optimizer->addOpt("W", new OptMatrix<gMat2D<T> >(*W_mat));
    }
    else
    {
        GurlsOption *W_opt = optimizer->getOpt("W");
        W_mat = &(OptMatrix<gMat2D<T> >::dynacast(W_opt))->getValue();
    }

    gMat2D<T> W_t(W, W_mat->cols(), W_mat->rows(), false);
    W_t.transpose(*W_mat);

    GurlsOption *x = opt.getOpt("Xte");
    gMat2D<T>* X = &(OptMatrix< gMat2D<T> >::dynacast(x))->getValue();
    GurlsOption *y = opt.getOpt("yte");
    gMat2D<T>* Y = &(OptMatrix< gMat2D<T> >::dynacast(y))->getValue();

    //opt.pred = pred_primal(opt.Xte, opt.yte, opt);
    PredPrimal<T> pp;
    pp.execute(*X, *Y, opt);

    //opt.perf   = perf_macroavg(opt.Xte, opt.yte, opt);
    PerfMacroAvg<T> ma;
    ma.execute(*X, *Y, opt);

    //acc = mean([opt.perf.acc]);
    GurlsOptionsList* perf = GurlsOptionsList::dynacast(opt.getOpt("perf"));
    GurlsOption *perf_opt = perf->getOpt("acc");
    gMat2D<T> *acc_mat = &(OptMatrix<gMat2D<T> >::dynacast(perf_opt))->getValue();

    T res;
    mean<T>(acc_mat->getData(), &res, 1, acc_mat->getSize(), 1);

    return res;
}

/**
 * Utility function used to build the kernel matrix; it computes the matrix of the squared euclidean distance between each column of A and each colum of B
 *
 * \param A matrix
 * \param B matrix
 * \param rows number of rows of both A and B
 * \param A_cols number of columns of A
 * \param B_cols number of columns of B
 * \param D output A_colsxB_cols kernel matrix
 */

template <typename T>
void distance(const T* A, const T* B, const int rows, const int A_cols, const int B_cols, T* D)
{
    set(D, (T)0.0, A_cols*B_cols);

    //for i = 1:na
    for(int i=0; i< A_cols; ++i)
        //    for j = 1:nb
        for(int j=0; j< B_cols; ++j)
            //      for k = 1:dim_a
            for(int k=0; k<rows; ++k)
                //          d(i,j) = d(i,j) + (a(k,i) - b(k,j))^2;
                D[i+A_cols*j] += pow(A[k+rows*i]-B[k+rows*j], (T)2.0);

}

/**
 * Constructs a nearly optimal rank-\a k approximation USV' to \a A, using \a its full iterations of a block Lanczos method
 * of block size \a l, started with an n x \a l random matrix, when \a A is m x n;
 *
 * \param A Matrix
 * \param A_rows Number of rows of A
 * \param A_cols Number of columns of A
 * \param U Matrix
 * \param S Matrix
 * \param V Matrix
 * \param k Rank, must be a positive integer <= the smallest dimension of A.
 * \param its Iterations of Lanczos method
 * \param l Block size
 */
template <typename T>
void random_svd(const T* A, const unsigned long A_rows, const unsigned long A_cols,
                T* U, T* S, T* V,
                unsigned long k = 6, unsigned long its = 2, unsigned long l = 0)
{
    // U: (A_rows,k)
    // S: (k)
    // V: (A_cols, k)

    if(l < k)
        l = k+2;

    if((k > A_rows) || (k > A_cols))
        throw gException("k must be <= the smallest dimension of A");

//    %
//    % SVD A directly if (its+1)*l >= m/1.25 or (its+1)*l >= n/1.25.
//    %

//    if(((its+1)*l >= m/1.25) || ((its+1)*l >= n/1.25))

    const T thr1 = static_cast<T>(A_rows/1.25);
    const T thr2 = static_cast<T>(A_cols/1.25);
    const T block_dim = static_cast<T>((its+1)*l);

    if(gt(block_dim, thr1) || eq(block_dim, thr1) || gt(block_dim, thr2) || eq(block_dim, thr2))
    {
//        [U,S,V] = svd(A,'econ');
        T *Q, *L, *Vt;
        int Q_rows, Q_cols;
        int L_len;
        int Vt_rows, Vt_cols;

        svd(A, Q, L, Vt, A_rows, A_cols, Q_rows, Q_cols, L_len, Vt_rows, Vt_cols, true);

//    %
//    % Retain only the leftmost k columns of U, the leftmost k columns of V,
//    % and the uppermost leftmost k x k block of S.
//    %

//      U = U(:,1:k);
        copy(U, Q, k*Q_rows);
        delete[] Q;

//      S = S(1:k,1:k);
        copy(S, L, k);
        delete[] L;

//      V = V(:,1:k);
        if(V != NULL)
        {
            T* tr = new T[Vt_cols*Vt_rows];

            transpose(Vt, Vt_rows, Vt_cols, tr);

            copy(V, tr, k*Vt_cols);
            delete[] tr;
        }
        delete[] Vt;

        return;
    }

    const T randMax = static_cast<T>(RAND_MAX);
    const T one = static_cast<T>(1.0);
    const T two = static_cast<T>(2.0);

//    if(m >= n)
    if(A_rows >= A_cols)
    {
        const int H_len = A_rows*l;

        T* H = new T[H_len];
        T* tmp = new T[A_cols*l];

//    %
//    % Apply A to a random matrix, obtaining H.
//    %
//        H = A*(2*rand(n,l)-ones(n,l));
        for(T *it =tmp, *end = tmp +(A_cols*l); it != end; ++it)
            *it = (two* rand()/randMax) - one;

        dot(A, tmp, H, A_rows, A_cols, A_cols, l, A_rows, l, CblasNoTrans, CblasNoTrans, CblasColMajor);

//    %
//    % Initialize F to its final size and fill its leftmost block with H.
//    %
//      F(1:m, 1:l) = H;
        const unsigned long F_cols = static_cast<unsigned long>(block_dim);
        T* F = new T[A_rows*F_cols];
        copy(F, H, H_len);

//    %
//    % Apply A*A' to H a total of its times,
//    % augmenting F with the new H each time.
//    %

//      for it = 1:its
        for(unsigned long iter = 1; iter<=its; ++iter)
        {
//        H = (H'*A)';
//        H = A*H;
            dot(H, A, tmp, A_rows, l, A_rows, A_cols, l, A_cols, CblasTrans, CblasNoTrans, CblasColMajor);
            dot(A, tmp, H, A_rows, A_cols, l, A_cols, A_rows, l, CblasNoTrans, CblasTrans, CblasColMajor);

//        F(1:m, (1+it*l):((it+1)*l)) = H;
            copy(F + (H_len*iter), H, H_len);
        }

        delete[] H;

//    %
//    % Form a matrix Q whose columns constitute an orthonormal basis
//    % for the columns of F.
//    %
//      [Q,R,E] = qr(F,0);
        const unsigned long Q_cols = std::min(A_rows, F_cols);
        T* Q = new T[A_rows*Q_cols];
        T* R = NULL;
        int* E = new int [F_cols];

        qr_econ(F, A_rows, F_cols, Q, R, E);

        delete[] E;
        delete[] F;

//    %
//    % SVD Q'*A to obtain approximations to the singular values
//    % and right singular vectors of A; adjust the left singular vectors
//    % of Q'*A to approximate the left singular vectors of A.
//    %
//      [U2,S,V] = svd(Q'*A,'econ');
        T *U2, *L, *Vt;
        int U2_rows, U2_cols;
        int L_len;
        int Vt_rows, Vt_cols;

        delete[] tmp;
        tmp = new T[Q_cols*A_cols];

        dot(Q, A, tmp, A_rows, Q_cols, A_rows, A_cols, Q_cols, A_cols, CblasTrans, CblasNoTrans, CblasColMajor);
        svd(tmp , U2, L, Vt, Q_cols, A_cols, U2_rows, U2_cols, L_len, Vt_rows, Vt_cols, true);

//      U = Q*U2;
        delete[] tmp;
        tmp = new T[A_rows*U2_cols];
        dot(Q, U2, tmp, A_rows, Q_cols, U2_rows, U2_cols, A_rows, U2_cols, CblasNoTrans, CblasNoTrans, CblasColMajor);

//      clear Q U2;
        delete[] Q;
        delete[] U2;

//    %
//    % Retain only the leftmost k columns of U, the leftmost k columns of V,
//    % and the uppermost leftmost k x k block of S.
//    %
//      U = U(:,1:k);
        copy(U, tmp, k*A_rows);
        delete[] tmp;

//      V = V(:,1:k);
        if(V != NULL)
        {
            T* tr = new T[Vt_cols*Vt_rows];

            transpose(Vt, Vt_rows, Vt_cols, tr);

            copy(V, tr, k*Vt_cols);
            delete[] tr;
        }
        delete[] Vt;


//      S = S(1:k,1:k);
        copy(S, L, k);
        delete[] L;

    }
    else
    {
//        %
//        % Apply A' to a random matrix, obtaining H.
//        %

        const int H_len = A_cols*l;

//            H = ((2*rand(l,m)-ones(l,m))*A)';
        T* H = new T[H_len];
        T* tmp = new T[l*A_rows];

        for(T *it =tmp, *end = tmp +(l*A_rows); it != end; ++it)
            *it = (two* rand()/randMax) - one;

        dot(A, tmp, H, A_rows, A_cols, l, A_rows, A_cols, l, CblasTrans, CblasTrans, CblasColMajor);


//        %
//        % Initialize F to its final size and fill its leftmost block with H.
//        %
//          F = zeros(n,(its+1)*l);
//          F(1:n, 1:l) = H;

        const unsigned long F_cols = static_cast<unsigned long>(block_dim);
        T* F = new T[A_cols*F_cols];
        copy(F, H, H_len);

//        %
//        % Apply A'*A to H a total of its times,
//        % augmenting F with the new H each time.
//        %
//          for it = 1:its
        for(unsigned long iter = 1; iter<=its; ++iter)
        {
//            H = A*H;
            dot(A, H, tmp, A_rows, A_cols, A_cols, l, A_rows, l, CblasNoTrans, CblasNoTrans, CblasColMajor);

//            H = (H'*A)';
            dot(A, tmp, H, A_rows, A_cols, A_rows, l, A_cols, l, CblasTrans, CblasNoTrans, CblasColMajor);

//            F(1:n, (1+it*l):((it+1)*l)) = H;
            copy(F + (H_len*iter), H, H_len);
        }

        delete[] H;

//        %
//        % Form a matrix Q whose columns constitute an orthonormal basis
//        % for the columns of F.
//        %

//          [Q,R,E] = qr(F,0);
        const unsigned long Q_cols = std::min(A_cols, F_cols);
        T* Q = new T[A_cols*Q_cols];
        T* R = NULL;
        int* E = new int [F_cols];

        qr_econ(F, A_cols, F_cols, Q, R, E);

        delete[] E;
        delete[] F;

//        %
//        % SVD A*Q to obtain approximations to the singular values
//        % and left singular vectors of A; adjust the right singular vectors
//        % of A*Q to approximate the right singular vectors of A.
//        %

//          [U,S,V2] = svd(A*Q,'econ');
        T *U2, *L, *Vt;
        int U2_rows, U2_cols;
        int L_len;
        int Vt_rows, Vt_cols;

        delete[] tmp;
        tmp = new T[A_rows*Q_cols];

        dot(A, Q, tmp, A_rows, A_cols, A_cols, Q_cols, A_rows, Q_cols, CblasNoTrans, CblasNoTrans, CblasColMajor);
        svd(tmp, U2, L, Vt, A_rows, Q_cols, U2_rows, U2_cols, L_len, Vt_rows, Vt_cols, true);

//          V = Q*V2;
        delete[] tmp;
        if(V != NULL)
        {
            tmp = new T[A_cols*Vt_rows];
            dot(Q, Vt, tmp, A_cols, Q_cols, Vt_rows, Vt_cols, A_cols, Vt_rows, CblasNoTrans, CblasTrans, CblasColMajor);
        }

        delete[] Vt;
        delete[] Q;

//        %
//        % Retain only the leftmost k columns of U, the leftmost k columns of V,
//        % and the uppermost leftmost k x k block of S.
//        %

//          U = U(:,1:k);
        copy(U, U2, k*U2_rows);
        delete[] U2;

//          V = V(:,1:k);
        if(V != NULL)
            copy(V, tmp, k*A_cols);

        delete[] tmp;

//          S = S(1:k,1:k);
        copy(S, L, k);
        delete[] L;

    }
}

/**
  *
  */
template<typename T>
void GInverseDiagonal(const T* Q, const T* L, const T* lambda, T* Z,
                    const int Q_rows, const int Q_cols,
                    const int L_length, const int lambda_length)
{
    //function Z = GInverseDiagonal( Q, L, lambda )

    //n = size(Q, 1); -> Q_rows
    //t = size(lambda, 2); -> lambda_length

    //Z = zeros(n, t);

    //D = Q.^(2);
    const int Q_size = Q_rows*Q_cols;
    T* D = new T[Q_size];
    //copy(D, Q, Q_size, 1, 1);
    mult(Q, Q, D, Q_size);

    T* d = new T[L_length];
//    T* Dd = new T[Q_rows /* *1 */];

    //for i = 1 : t
    for(int i=0; i<lambda_length; ++i)
    {
//    d = L + (n*lambda(i));
        set(d, Q_rows*lambda[i] , L_length);
        axpy(L_length, (T)1.0, L, 1, d, 1);

//    d  = d.^(-1);
        setReciprocal(d, L_length);

//    Z(:,i) = D*d;
        gemv(CblasNoTrans, Q_rows, Q_cols, (T)1.0, D, Q_cols, d, 1, (T)0.0, Z+(i*lambda_length), 1);
    }

    delete[] d;
    delete[] D;
}

}

#endif // _GURLS_UTILS_H_
