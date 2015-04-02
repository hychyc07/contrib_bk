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

#include <assert.h>

#define EIGEN2_SUPPORT
#include <Eigen/Array>

#include <iCub/piSquare/policyLibrary/dmpPolicy.h>

using namespace Eigen;

namespace library 
{

DMPPolicy::DMPPolicy() :
    initialized_(false)
{
}

DMPPolicy::~DMPPolicy()
{
}

bool DMPPolicy::initialize(boost::shared_ptr<dmp::DynamicMovementPrimitive> dmp)
{
    if (!dmp->isInitialized())
    {
        printf("ERROR: DMP is not initialized\n.");
        return false;
    }
    dmp_ = dmp;

    return (initialized_ = true);
}

bool DMPPolicy::setNumTimeSteps(const int numTimeSteps)
{
    if (!initialized_)
    {
        return false;
    }
    numTimeSteps_ = numTimeSteps;

    return true;
}

bool DMPPolicy::getNumTimeSteps(int& numTimeSteps)
{
    numTimeSteps = numTimeSteps_;

    return true;
}


bool DMPPolicy::getNumDimensions(int& numDimensions)
{
    if (!initialized_)
    {
        return false;
    }
    numDimensions = dmp_->getNumTransformationSystems();

    return true;
}

bool DMPPolicy::getNumParameters(std::vector<int>& numParams)
{
    if (!initialized_)
    {
        return false;
    }
    numParams.clear();

    return dmp_->getNumRFS(numParams);
}

bool DMPPolicy::getBasisFunctions(std::vector<MatrixXd>& basisFunctions)
{
    if (!initialized_)
    {
        return false;
    }

    std::vector<MatrixXd> pureBasisFunctions;
    if(!dmp_->getBasisFunctions(numTimeSteps_, pureBasisFunctions))
    {
        printf("ERROR: Could not get basis function matrix.\n");
        return false;
    }

    VectorXd canSystemVector = VectorXd::Zero(numTimeSteps_);
    if(!dmp_->getCanonicalSystem(numTimeSteps_, canSystemVector))
    {
       printf("ERROR: Could not evaluate the canonical system.\n");
	   return false;
    }

    std::vector<int> numThetas;
    if (!getNumParameters(numThetas))
    {
        printf("ERROR: Could not get number of parameters.\n");
        return false;
    }

    basisFunctions.clear();
    for (int d=0; d<dmp_->getNumTransformationSystems(); d++)
    {
        MatrixXd basisFunctionMatrix = MatrixXd::Zero(numTimeSteps_, numThetas[d]);
        for (int j=0; j<numThetas[d]; j++)
        {
            basisFunctionMatrix.col(j) = pureBasisFunctions[d].col(j).cwise() * canSystemVector;
        }
        basisFunctions.push_back(basisFunctionMatrix);
    }

    return true;
}

bool DMPPolicy::getControlCosts(std::vector<MatrixXd>& controlCosts)
{
    if (!initialized_)
    {
        return false;
    }
    controlCosts.clear();
    std::vector<int> numThetas;
    if (!getNumParameters(numThetas))
    {
        printf("ERROR: Could not get number of parameters.\n");
        return false;
    }
    for (std::vector<int>::iterator it = numThetas.begin(); it != numThetas.end(); it++)
    {
        MatrixXd idendityControlCostMatrix = MatrixXd::Identity(*it, *it);
        controlCosts.push_back(idendityControlCostMatrix);
    }

    return true;
}

bool DMPPolicy::updateParameters(const std::vector<MatrixXd>& updates)
{
    if (!initialized_)
    {
        return false;
    }

    std::vector<VectorXd> thetaVectors;
    if (!dmp_->getThetas(thetaVectors))
    {
        printf("ERROR: Could not get parameter vector.\n");
        return false;
    }
    assert(thetaVectors.size() == updates.size());

    std::vector<MatrixXd> basisFunctions;
    if (!dmp_->getBasisFunctions(numTimeSteps_, basisFunctions))
    {
        printf("ERROR: Could not get basis function matrix.\n");
        return false;
    }
    assert(basisFunctions.size() == updates.size());

    for (int d = 0; d < static_cast<int> (updates.size()); d++)
    {
        int numRfs = basisFunctions[d].cols();

        assert(updates[d].rows() == static_cast<int>(numTimeSteps_));
        assert(updates[d].cols() == static_cast<int>(thetaVectors[d].size()));
        assert(updates[d].rows() == basisFunctions[d].rows());
        assert(updates[d].cols() == basisFunctions[d].cols());

        for (int j = 0; j < numRfs; j++)
        {
            double sumTimeWeightTimesBasisFunctionWeight = 0;
            double sumTimeWeightTimesBasisFunctionWeightTimesUpdate = 0;
            for (int i = 0; i < numTimeSteps_; i++)
            {
                double timeWeight = numTimeSteps_ - i;
                sumTimeWeightTimesBasisFunctionWeight += (timeWeight * basisFunctions[d](i,j));
                sumTimeWeightTimesBasisFunctionWeightTimesUpdate += ((timeWeight * basisFunctions[d](i,j)) * updates[d](i,j));
            }

            //update the theta vector
            thetaVectors[d](j) += sumTimeWeightTimesBasisFunctionWeightTimesUpdate / sumTimeWeightTimesBasisFunctionWeight;
        }
    }

    if (!dmp_->setThetas(thetaVectors))
    {
       printf("ERROR: Could not set parameter vector.\n");
        return false;
    }

    return true;
}

bool DMPPolicy::getParameters(std::vector<VectorXd>& parameters)
{
    if (!initialized_)
    {
        return false;
    }

    parameters.clear();

    return dmp_->getThetas(parameters);
}

bool DMPPolicy::setParameters(const std::vector<VectorXd>& parameters)
{
    if (!initialized_)
    {
        return false;
    }

    return dmp_->setThetas(parameters);
}

bool DMPPolicy::getDMP(boost::shared_ptr<dmp::DynamicMovementPrimitive>& dmp)
{
    if (!initialized_)
        return false;
    dmp = dmp_;

    return true;
}

}
