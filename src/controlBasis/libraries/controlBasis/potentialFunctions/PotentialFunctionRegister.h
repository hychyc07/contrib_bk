// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-  
/**
 * \defgroup icub_controlbasis_libraries Control Basis Libraries
 * @ingroup icub_controlbasis_libraries
 *
 */
#ifndef _POTENTIAL_FUNCTION_REGISTER__H_
#define _POTENTIAL_FUNCTION_REGISTER__H_

#include "PotentialFunctionFactory.h"

// potential functions
#include "CosineField.h"
#include "ManipulabilityField.h"
#include "ConfigurationSquaredError.h"
#include "HeadingSquaredError.h"
#include "LogPolarHeadingSquaredError.h"
#include "StereoHeadingSquaredError.h"
#include "CartesianPositionSquaredError.h"
#include "CartesianOrientationSquaredError.h"
#include "CartesianPositionHarmonicFunction.h"
#include "ContactSetSquaredError.h"

namespace CB {

    void registerPotentialFunctions() {
        PotentialFunctionFactory::instance().registerClass<ConfigurationSquaredError>(new ConfigurationSquaredError());
        PotentialFunctionFactory::instance().registerClass<HeadingSquaredError>(new HeadingSquaredError());
        PotentialFunctionFactory::instance().registerClass<LogPolarHeadingSquaredError>(new LogPolarHeadingSquaredError());
        PotentialFunctionFactory::instance().registerClass<StereoHeadingSquaredError>(new StereoHeadingSquaredError());
        PotentialFunctionFactory::instance().registerClass<CartesianPositionSquaredError>(new CartesianPositionSquaredError());
        PotentialFunctionFactory::instance().registerClass<CartesianOrientationSquaredError>(new CartesianOrientationSquaredError());
        PotentialFunctionFactory::instance().registerClass<ContactSetSquaredError>(new ContactSetSquaredError());
        PotentialFunctionFactory::instance().registerClass<CosineField>(new CosineField());
        PotentialFunctionFactory::instance().registerClass<ManipulabilityField>(new ManipulabilityField());
        //        PotentialFunctionFactory::instance().registerClass<CartesianPositionHarmonicFunction>(new CartesianPositionHarmonicFunction());
    }

}

#endif
