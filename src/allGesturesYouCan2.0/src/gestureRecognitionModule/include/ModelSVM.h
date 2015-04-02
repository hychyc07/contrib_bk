#include <deque>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <iostream>
#include <fstream>
#include <string>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/math/Math.h>
#include <yarp/os/ResourceFinder.h>
#include "Model.h"

class ModelSVM: public Model
{
    std::string actionsFilePath;
    std::deque<yarp::sig::Vector> models;
    int numActions;
    int dictionarySize;
    int buffer;
    int nFeatures;
    int frameFeatureSize;

public:

    ModelSVM(std::string actionsFilePath);
    void computeOutput(std::deque<yarp::sig::Vector> &features, yarp::sig::Vector &output);
    void concatenate(std::deque<yarp::sig::Vector> &features, yarp::sig::Vector &descriptor);
    void trainModel();
    int getNumActions();
    bool read();
    void write();
    void printModel();
    ~ModelSVM();

};