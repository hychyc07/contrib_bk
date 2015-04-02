#include "gmat2d.h"
#include "options.h"
#include "optlist.h"

#include "rlsprimalr.h"
#include "rlsdualr.h"
#include "hodual.h"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE yeast

#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "test.h"

using namespace gurls;

typedef double T;

//BOOST_AUTO_TEST_SUITE(yeast)

BOOST_AUTO_TEST_CASE(Rlsprimalr)
{
    OptimizerData<T, RLSPrimalr<T> > fixture(path(std::string(GURLS_DATA_DIR))/path("yeast"), "rlsprimalr");

    fixture.runTask();

    fixture.checkResults();

}

BOOST_AUTO_TEST_CASE(RlsdualrLinearKernel)
{
    OptimizerData<T, RLSDualr<T> > fixture(path(std::string(GURLS_DATA_DIR))/path("yeast"), "rlsdualr_linearkernel");

    GurlsOptionsList* kernel = GurlsOptionsList::dynacast(fixture.opt->getOpt("kernel"));
    kernel->addOpt("type", "linear");

    fixture.runTask();

    fixture.checkResults();

}

BOOST_AUTO_TEST_CASE(RlsdualrGaussKernel)
{
    OptimizerData<T, RLSDualr<T> > fixture(path(std::string(GURLS_DATA_DIR))/path("yeast"), "rlsdualr_gausskernel");

    GurlsOptionsList* kernel = GurlsOptionsList::dynacast(fixture.opt->getOpt("kernel"));
    kernel->addOpt("type", "rbf");

    fixture.runTask();

    fixture.checkResults();

}

BOOST_AUTO_TEST_CASE(HodualrGaussKernel)
{
    ParamselData<T, ParamSelHoDualr<T> > fixture(path(std::string(GURLS_DATA_DIR))/path("yeast"), "rlsdualr_gausskernel");

    GurlsOptionsList* kernel = GurlsOptionsList::dynacast(fixture.opt->getOpt("kernel"));
    kernel->addOpt("type", "rbf");

    fixture.runTask();

    fixture.checkResults();

}

BOOST_AUTO_TEST_CASE(HodualrlinearKernel)
{
    ParamselData<T, ParamSelHoDualr<T> > fixture(path(std::string(GURLS_DATA_DIR))/path("yeast"), "rlsdualr_linearkernel");

    GurlsOptionsList* kernel = GurlsOptionsList::dynacast(fixture.opt->getOpt("kernel"));
    kernel->addOpt("type", "linear");

    fixture.checkResults();

}

//BOOST_AUTO_TEST_SUITE_END()
