/*
 * Copyright (C) 2013 Elena Ceseracciu, iCub Facility - Istituto Italiano di Tecnologia
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 and later.
 *
 */

#ifndef _GURLSCLASSIFICATIONBATCHMODULE__
#define _GURLSCLASSIFICATIONBATCHMODULE__


// yarp
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConstString.h>

//stl
#include <string>
#include <vector>
//#include <set>

// thrift-generated interface
#include <ClassScoreList.h>
#include <GurlsClassificationInterface.h>
//gurls
#include "gurls++/wrapper.h"
#include "gurls++/gmath.h"

#include "TrainingDatabase.h"

enum ModuleState {STATE_DONOTHING = 0, STATE_SAVING=1, STATE_RECOGNIZING =2};




class GurlsClassificationBatchModule : public yarp::os::RFModule, public GurlsClassificationInterface {


private:
    //ports
    yarp::os::BufferedPort<ClassScoreList> outputScoresPort;
    yarp::os::BufferedPort<yarp::os::Value> outputClassPort;
    yarp::os::BufferedPort<yarp::sig::Vector> featuresPort;
    yarp::os::Port  configPort;   //to configure the module and change its state

    gurls::GurlsWrapper<double> *gurlsClassifier;
    TrainingDatabase dataset;
    //std::map <std::string, std::vector<yarp::sig::Vector> > trainingSamples;
    ModuleState currentState;
    std::string currentClass;

protected:
        //inherited from GurlsClassificationInterface
    virtual bool add_sample(const std::string& className, const yarp::sig::Vector& sample);
    virtual bool save(const std::string& className);
    virtual bool train();
    virtual bool forget(const std::string& className);
    virtual std::vector<std::string>  classList();
    virtual bool stop();
    virtual bool recognize();
    virtual std::string classify_sample(const yarp::sig::Vector& sample);
    virtual std::vector<ClassScore>  get_scores_for_sample(const yarp::sig::Vector& sample);
    virtual std::string get_parameter(const std::string& parameterName){return "nyi";};
    virtual bool set_parameter(const std::string& parameterName, const std::string& parameterValue){return false;};

public:
    GurlsClassificationBatchModule();
    ~GurlsClassificationBatchModule();

    //inherited from RFModule
    virtual bool configure (yarp::os::ResourceFinder &rf);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    bool attach(yarp::os::Port &source)
    {
        return this->yarp().attachAsServer(source);
    }

};


#endif

