#ifndef TEST_H
#define TEST_H

#include <exception>
#include <string>

#include "gmat2d.h"
#include "optlist.h"
#include "exceptions.h"
#include "gmath.h"

#include "../misc/test_utils.h"

#include "rlsprimalr.h"
#include "rlsdualr.h"

#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

using namespace boost::filesystem3;

namespace gurls
{

template<typename T, class TaskType>
class Data
{
public:
    gurls::gMat2D<T>* X;
    gurls::gMat2D<T>* Y;
    gurls::GurlsOptionsList* opt;

    Data(path dataDirectory, path taskName) : dataDir(dataDirectory), task(taskName), extension(".txt")
    {
        X = readFile<T>(path(dataDir / path("Xtr" + extension)).native());
        Y = readFile<T>(path(dataDir / path("ytr" + extension)).native());

        opt = new gurls::GurlsOptionsList("Testdata");
    }

    virtual void checkResults() = 0;

    void runTask()
    {
        TaskType taskProcess;
        taskProcess.execute(*X, *Y, *opt);
    }

    ~Data()
    {
        delete X;
        delete Y;
        delete opt;
    }

protected:

    path dataDir;

    std::string extension;

    path task;
};

template<typename T, class TaskType>
class OptimizerData: public Data<T, TaskType>
{
public:
    OptimizerData(path dataDirectory, path taskName) :Data<T, TaskType>(dataDirectory, taskName)
    {
        this->opt->addOpt("singlelambda", new gurls::OptFunction("median"));

        gurls::GurlsOptionsList* paramsel = new gurls::GurlsOptionsList("paramsel");
        this->opt->addOpt("paramsel", paramsel);

        path dataPath = this->dataDir / this->task;

        groundTruth.X = readFile<T>(path(dataPath / path("rls_X" + this->extension)).native());
        groundTruth.C = readFile<T>(path(dataPath / path("rls_C" + this->extension)).native());
        groundTruth.W = readFile<T>(path(dataPath / path("rls_W" + this->extension)).native());

        gurls::OptNumberList *lambdas_opt = new gurls::OptNumberList();

        paramsel->addOpt("lambdas", lambdas_opt);

        gMat2D<T>* lambdas = readFile<T>(path(dataPath / path("paramsel_lambdas" + this->extension)).native());

        for(T *it = lambdas->getData(), *end = it+lambdas->getSize(); it != end; ++it)
            lambdas_opt->add(static_cast<double>(*it));

        delete lambdas;

        path kernelPath = path(dataPath / path("kernel_K" + this->extension));
        if(exists(kernelPath) )
        {
            gMat2D<T>* K = readFile<T>(kernelPath.native());

            GurlsOptionsList* kernel = new GurlsOptionsList("kernel");
            kernel->addOpt("K", new OptMatrix<gMat2D<T> > (*K));
            this->opt->addOpt("kernel", kernel);
        }
    }

    void checkResults()
    {
        GurlsOptionsList* optimizer = GurlsOptionsList::dynacast(this->opt->getOpt("optimizer"));

        gMat2D<T>& res_X = OptMatrix<gMat2D<T> >::dynacast(optimizer->getOpt("X"))->getValue();
        gMat2D<T>& res_C = OptMatrix<gMat2D<T> >::dynacast(optimizer->getOpt("C"))->getValue();
        gMat2D<T>& res_W = OptMatrix<gMat2D<T> >::dynacast(optimizer->getOpt("W"))->getValue();

        BOOST_TEST_MESSAGE( "Checking X:" );
        check_matrix(res_X, *(groundTruth.X));

        BOOST_TEST_MESSAGE( "Checking C:" );
        check_matrix(res_C, *(groundTruth.C));

        BOOST_TEST_MESSAGE( "Checking W:" );
        check_matrix(res_W, *(groundTruth.W));
    }

    struct
    {
        gurls::gMat2D<T>* X;
        gurls::gMat2D<T>* C;
        gurls::gMat2D<T>* W;

    } groundTruth;
};

template<typename T, class TaskType>
class ParamselData: public Data<T, TaskType>
{
public:
    ParamselData(path dataDirectory, path taskName) :Data<T, TaskType>(dataDirectory, taskName)
    {
        this->opt->addOpt("nlambda",new OptNumber(20));
        this->opt->addOpt("nholdouts", new OptNumber(1));
        this->opt->addOpt("smallnumber", new OptNumber(1e-8));
        this->opt->addOpt("hoperf", new OptString("macroavg"));


        path dataPath = this->dataDir / this->task;

        gMat2D<T>* indices = readFile<T>(path(dataPath / path("split_indices" + this->extension)).native());
        gMat2D<T>* lasts = readFile<T>(path(dataPath / path("split_lasts" + this->extension)).native());


        GurlsOptionsList* split = new GurlsOptionsList("split");
        split->addOpt("indices", new OptMatrix<gMat2D<T> > (*indices));
        split->addOpt("lasts", new OptMatrix<gMat2D<T> > (*lasts));
        this->opt->addOpt("split", split);


        path kernelPath = path(dataPath / path("kernel_K" + this->extension));
        if(exists(kernelPath))
        {
            gMat2D<T>* K = readFile<T>(kernelPath.native());

            GurlsOptionsList* kernel = new GurlsOptionsList("kernel");
            kernel->addOpt("K", new OptMatrix<gMat2D<T> > (*K));
            this->opt->addOpt("kernel", kernel);
        }

    }


    void checkResults()
    {
        GurlsOptionsList* paramsel = GurlsOptionsList::dynacast(this->opt->getOpt("paramsel"));

        gMat2D<T>& res_guesses = OptMatrix<gMat2D<T> >::dynacast(paramsel->getOpt("guesses"))->getValue();
        gMat2D<T>& res_forho = OptMatrix<gMat2D<T> >::dynacast(paramsel->getOpt("forho"))->getValue();
        std::vector<double>& res_lambdas = OptNumberList::dynacast(paramsel->getOpt("lambdas"))->getValue();

        BOOST_TEST_MESSAGE( "Checking guesses:" );
        check_matrix(res_guesses, *(groundTruth.guesses));

        BOOST_TEST_MESSAGE( "Checking forho:" );
        check_matrix(res_forho, *(groundTruth.forho));


        BOOST_TEST_MESSAGE( "Checking lambdas:" );
        BOOST_REQUIRE_EQUAL(res_lambdas.size(), groundTruth.lambdas->getValue().size());
        check_vectors( &(*(res_lambdas.begin())), &(*(groundTruth.lambdas->getValue().begin())), res_lambdas.size());
    }

    struct
    {
        gurls::gMat2D<T>* guesses;
        gurls::gMat2D<T>* forho;
        gurls::OptNumberList* lambdas;

    } groundTruth;

};

template<typename T>
void check_vectors(const T* result, const T* groundTruth, const unsigned long size)
{
//    const T warnCoeff = 1.0e-6;
    const T errCoeff = 1.0e-4;
    for(const T *res_it = result, *res_end = res_it+size, *ref_it = groundTruth; res_it != res_end; ++res_it, ++ref_it)
    {
//        BOOST_WARN_LE(std::abs(*res_it - *ref_it), warnCoeff*std::abs(*ref_it));
        BOOST_REQUIRE_LE(std::abs(*res_it - *ref_it), errCoeff*std::abs(*ref_it));
    }
}

template<typename T>
void check_matrix(gurls::gMat2D<T>& result, gurls::gMat2D<T>& groundTruth)
{
    BOOST_REQUIRE_EQUAL(result.rows(), groundTruth.rows());
    BOOST_REQUIRE_EQUAL(result.cols(), groundTruth.cols());

    check_vectors(result.getData(), groundTruth.getData(), result.getSize());
}

}

#endif //TEST_H
