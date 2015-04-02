/*
 * Copyright (C) 2013 Elena Ceseracciu, iCub Facility - Istituto Italiano di Tecnologia
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 and later.
 *
 */

#include "GurlsClassificationBatchModule.h"
#include "gurls++/KernelRLSwrapper.h"
#include <string>
#include <iostream>

using namespace gurls;
using namespace yarp::os;


GurlsClassificationBatchModule::GurlsClassificationBatchModule(): gurlsClassifier(NULL)
{
    currentState=STATE_DONOTHING;

};

GurlsClassificationBatchModule::~GurlsClassificationBatchModule()
{
    //deallocate classifier
    if (gurlsClassifier)
        delete gurlsClassifier;
};

bool GurlsClassificationBatchModule::add_sample(const std::string& className, const yarp::sig::Vector& sample)
{
    return dataset.addDataForClass(className, sample);
};

bool GurlsClassificationBatchModule::save(const std::string& className)
{
    currentClass=className;
    currentState=STATE_SAVING;
    return true;
};

bool GurlsClassificationBatchModule::train()
{
    gMat2D<double> X, y;
    dataset.getDataForAllClasses(X, y);
    if (gurlsClassifier)
    {
        gurlsClassifier->train(X, y);
        return true;
    }
    else
        return false;
};

bool GurlsClassificationBatchModule::forget(const std::string& className)
{
    return dataset.forget(className); 
};

std::vector<std::string>  GurlsClassificationBatchModule::classList()
{
    return dataset.classList();
};

bool GurlsClassificationBatchModule::stop()
{
    currentState=STATE_DONOTHING;
    return true;
};
bool GurlsClassificationBatchModule::recognize()
{
    currentState=STATE_RECOGNIZING;
    return true;
};

std::string GurlsClassificationBatchModule::classify_sample(const yarp::sig::Vector& sample)
{
    yarp::sig::Vector sampleCopy=sample;
    gVec< double> X(sampleCopy.data(), sampleCopy.length());
    unsigned long index;
    double result=gurlsClassifier->eval (X,&index);
    return dataset.getNameForId(index);
};

std::vector<ClassScore>  GurlsClassificationBatchModule::get_scores_for_sample(const yarp::sig::Vector& sample)
{
    yarp::sig::Vector sampleCopy=sample;
    gMat2D<double> X(sampleCopy.data(), 1, sampleCopy.length(), false);
    std::cout << "getting scores for sample " <<X.what() <<std::endl;
    gMat2D<double> *scores=gurlsClassifier->eval (X);

    std::vector<ClassScore> result;
    for(size_t ct=0; ct<scores->rows(); ++ct)
    {
        result.push_back(ClassScore(dataset.getNameForId(ct), (*scores)(0, ct)));
    }
    
    return result;
};

bool GurlsClassificationBatchModule::configure (yarp::os::ResourceFinder &rf)
{
    std::string moduleName = rf.check("name", Value("gurlsClassifier"), "module name (string)").asString().c_str();
    setName(moduleName.c_str());

    //Open ports
    std::string slash="/";
    std::string configPortName = slash + getName().c_str() + slash + std::string("rpc");
    std::string featuresPortName = slash + getName().c_str() + slash + std::string("features:i");
    std::string outputScoresPortName = slash + getName().c_str() + slash + std::string("scores:o");
    std::string outputClassPortName = slash + getName().c_str() + slash + std::string("classification:o");

    if (!configPort.open(configPortName.c_str())) {
        std::cout  << ": unable to open port " << configPortName << std::endl;
        return false; 
    }
    attach(configPort);

    if (!featuresPort.open(featuresPortName.c_str())) {
        std::cout  << ": unable to open port " << featuresPortName << std::endl;
        return false; 
    }

    if (!outputClassPort.open(outputClassPortName.c_str())) {
        std::cout  << ": unable to open port " << outputClassPortName << std::endl;
        return false; 
    }

    if (!outputScoresPort.open(outputScoresPortName.c_str())) {
        std::cout  << ": unable to open port " << outputScoresPortName << std::endl;
        return false; 
    }

    //Instantiate classifier  -- need to check if the name should also contain a path...
    std::string experimentName= rf.check("experiment", Value("defaultExperiment")).asString().c_str();
    gurlsClassifier = new KernelRLSWrapper<double>(experimentName.c_str());

    //Read parameters
    //yarp::os::Bottle &parameters =rf.findGroup("PARAMETERS");
    //for (int i=0; i<parameters.size(); ++i)
    //{
    //    gurlsClassifier->setParam(parameters.get(i).asString().c_str(), parameters.get(i).asString().c_str());
    //}
    
    return true;
}

bool GurlsClassificationBatchModule::close()
{
    //close ports
    outputScoresPort.close();
    outputClassPort.close();
    featuresPort.close();
    configPort.close(); 
    return true;
}

bool GurlsClassificationBatchModule::interruptModule()
{
    //interrupt ports
    outputScoresPort.interrupt();
    outputClassPort.interrupt();
    featuresPort.interrupt();
    configPort.interrupt(); 
    return true;
}

bool GurlsClassificationBatchModule::updateModule()
{
    switch (currentState)
    {
         yarp::sig::Vector *sample;
        case STATE_SAVING:
            sample=featuresPort.read(false);

            if(sample==NULL)
                return true;

            if (!dataset.addDataForClass(currentClass, *sample))
                std::cerr << "WARNING! Could not add last sample to database" << std::endl;

            return true;
       
        case STATE_RECOGNIZING:
            sample=featuresPort.read(false);

            if(sample==NULL)
                return true;


            ClassScoreList &onPort=outputScoresPort.prepare();
            onPort.elements= get_scores_for_sample(*sample);

            outputScoresPort.write();

            //find maximum score as winner...
            size_t winner=0;
            double maxScore=-1;
            for (size_t ind=0; ind<onPort.elements.size(); ++ind)
                if (onPort.elements.at(ind).score >maxScore)
                    winner=ind;
            
            yarp::os::Value &classOnPort=outputClassPort.prepare();
            classOnPort.asString()= onPort.elements.at(winner).className.c_str();
            outputClassPort.write();
            

            return true;
    }
    //save data to file every now and then
    return true;
}
