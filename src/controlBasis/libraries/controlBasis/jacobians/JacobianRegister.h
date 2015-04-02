// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-  
/**
 * \defgroup icub_controlbasis_libraries Control Basis Libraries
 * @ingroup icub_controlbasis_libraries
 *
 */
#ifndef _JACOBIAN_REGISTER__H_
#define _JACOBIAN_REGISTER__H_

#include "JacobianFactory.h"

// jacobians
#include "ManipulatorPositionJacobian.h"
#include "ManipulatorOrientationJacobian.h"
#include "HeadingJacobian.h"
#include "LogPolarHeadingJacobian.h"

namespace CB {

    void registerJacobians() {
        JacobianFactory::instance().registerClass<ManipulatorPositionJacobian>(new ManipulatorPositionJacobian());
        JacobianFactory::instance().registerClass<ManipulatorOrientationJacobian>(new ManipulatorOrientationJacobian());
        JacobianFactory::instance().registerClass<HeadingJacobian>(new HeadingJacobian());
        JacobianFactory::instance().registerClass<LogPolarHeadingJacobian>(new LogPolarHeadingJacobian());
    }

}

#endif
