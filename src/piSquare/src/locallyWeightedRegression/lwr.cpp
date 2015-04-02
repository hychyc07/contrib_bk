/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************/

#include <iostream>
#include <fstream>
#include <assert.h>

#include <boost/foreach.hpp>

#include <iCub/piSquare/locallyWeightedRegression/lwr.h>

using namespace Eigen;

namespace lwr
{

LocallyWeightedRegression::LocallyWeightedRegression() :
    initialized_(false),
	numRfs_(0)
{
}

LocallyWeightedRegression::~LocallyWeightedRegression()
{
}

bool LocallyWeightedRegression::initialize(const int numRfs, const double activation, const bool exponentiallySpaced, const double canSysCutoff)
{
	if(initialized_) printf("LWR model already initialized. Re-initializing with new parameters.\n");

    if (numRfs <= 0)
    {
        printf("ERROR: Number of receptive fields (%i) is invalid.\n", numRfs);
        initialized_ = false;
        return initialized_;
    }

    numRfs_ = numRfs;
    centers_ = VectorXd::Zero(numRfs_);
    thetas_ = VectorXd::Zero(numRfs_);
    widths_ = VectorXd::Zero(numRfs_);

    if(exponentiallySpaced)
    {
        double lastInputX = 1.0;
        double alphaX = -log(canSysCutoff);
        for (int i = 0; i < numRfs_; ++i)
        {
			// 1.0 is the default duration
            double t = (i+1) * (1. / static_cast<double> (numRfs_ - 1)) * 1.0;
            double inputX = exp(-alphaX * t);

            widths_(i) = pow(inputX - lastInputX, 2) / -log(activation);

            centers_(i) = lastInputX;
            lastInputX = inputX;
        }
    }
    else
    {
        double diff;
        if (numRfs_ == 1)
        {
            centers_(0) = 0.5;
            diff = 0.5;
        }
        else
        {
            for (int i = 0; i < numRfs_; i++)
            {
                centers_(i) = static_cast<double> (i) / static_cast<double> (numRfs - 1);
            }
            diff = static_cast<double> (1.0) / static_cast<double> (numRfs - 1);
        }
        double width = -pow(diff / static_cast<double> (2.0), 2) / log(activation);
        for (int i = 0; i < numRfs_; i++)
        {
            widths_(i) = width;
        }
    }


    initialized_ = true;

    return initialized_;
}

bool LocallyWeightedRegression::predict(const double xQuery, double &yPrediction)
{
    if (!initialized_)
    {
        printf("ERROR: LWR model not initialized.\n");
        return initialized_;
    }

    double sx = 0;
    double sxtd = 0;
    for (int i = 0; i < numRfs_; i++)
    {
        double psi = getKernel(xQuery, i);
        sxtd += psi * thetas_(i) * xQuery;
        sx += psi;
    }

    yPrediction = sxtd / sx;

    return true;
}

bool LocallyWeightedRegression::generateBasisFunctionMatrix(const VectorXd &xInputVector, MatrixXd &basisFunctionMatrix)
{
    if (xInputVector.size() == 0)
    {
        printf("ERROR: Cannot compute psi for an empty vector.\n");
        return false;
    }
    assert(basisFunctionMatrix.rows() == xInputVector.size());
    assert(basisFunctionMatrix.cols() == centers_.size());
    for (int i = 0; i < xInputVector.size(); i++)
    {
        for (int j = 0; j < centers_.size(); j++)
        {
            basisFunctionMatrix(i, j) = getKernel(xInputVector[i], j);
        }
    }

    return true;
}

bool LocallyWeightedRegression::getThetas(VectorXd &thetas)
{
    if (!initialized_)
    {
        printf("ERROR: LWR model not initialized.\n");
        return initialized_;
    }

    assert(thetas.cols() == thetas_.cols());
    assert(thetas.rows() == thetas_.rows());

    thetas = thetas_;

    return true;
}

bool LocallyWeightedRegression::setThetas(const VectorXd &thetas)
{
    if (!initialized_)
    {
        printf("ERROR: LWR model not initialized.\n");
        return initialized_;
    }
    assert(thetas.cols() == thetas_.cols());
    assert(thetas.rows() == thetas_.rows());
    thetas_ = thetas;

    return true;
}

bool LocallyWeightedRegression::updateThetas(const VectorXd &deltaThetas)
{
    if (!initialized_)
    {
        printf("ERROR: LWR model not initialized.\n");
        return initialized_;
    }
    assert(deltaThetas.cols() == thetas_.cols());
    assert(deltaThetas.rows() == thetas_.rows());
    thetas_ += deltaThetas;

    return true;
}

bool LocallyWeightedRegression::getWidthsAndCenters(VectorXd &widths, VectorXd &centers)
{
    if (!initialized_)
    {
        printf("ERROR: LWR model not initialized.\n");
        return initialized_;
    }
    widths = widths_;
    centers = centers_;

    return true;
}

double LocallyWeightedRegression::getKernel(const double xInput, const int centerIndex)
{
    if (!initialized_)
    {
        printf("ERROR: LWR model not initialized.\n");
        return initialized_;
    }

    return exp(-(static_cast<double> (1.0) / widths_(centerIndex)) * pow(xInput - centers_(centerIndex), 2));
}

bool LocallyWeightedRegression::learnWeights(const VectorXd &xInputVector, const VectorXd &yTargetVector)
{
    if (!initialized_)
    {
        printf("ERROR: LWR model is not initialized.\n");
        return initialized_;
    }

    if(xInputVector.size() != yTargetVector.size())
    {
        printf("ERROR: Input (%i) and target (%i) vector have different sizes.\n", xInputVector.size(), yTargetVector.size());
        return false;
    }

    MatrixXd basisFunctionMatrix = MatrixXd::Zero(xInputVector.size(), centers_.size());
    if (!generateBasisFunctionMatrix(xInputVector, basisFunctionMatrix))
    {
        printf("ERROR: Could not generate basis function matrix.\n");
        return false;
    }

    MatrixXd tmpMatrixA = MatrixXd::Zero(xInputVector.size(), numRfs_);
    tmpMatrixA = xInputVector.cwise().square() * MatrixXd::Ones(1, numRfs_);
    tmpMatrixA = tmpMatrixA.cwise() * basisFunctionMatrix;

    VectorXd tmpMatrixSx = VectorXd::Zero(numRfs_, 1);
    tmpMatrixSx = tmpMatrixA.colwise().sum();

    MatrixXd tmpMatrixB = MatrixXd::Zero(xInputVector.size(), numRfs_);
    tmpMatrixB = xInputVector.cwise() * yTargetVector * MatrixXd::Ones(1, numRfs_);
    tmpMatrixB = tmpMatrixB.cwise() * basisFunctionMatrix;

    VectorXd tmpMatrixSxtd = VectorXd::Zero(numRfs_, 1);
    tmpMatrixSxtd = tmpMatrixB.colwise().sum();

    double ridgeRegression = 0.0000000001;
    thetas_ = tmpMatrixSxtd.cwise() / (tmpMatrixSx.cwise() + ridgeRegression);

    return true;
}

bool LocallyWeightedRegression::getNumRFS(int &numRfs)
{
    if (!initialized_)
    {
        printf("ERROR: LWR is not initialized, not returning number of receptive fields.\n");
        return false;
    }
    numRfs = numRfs_;
    return true;
}

}
