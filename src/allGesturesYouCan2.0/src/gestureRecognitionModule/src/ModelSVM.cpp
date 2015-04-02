#include "ModelSVM.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;

ModelSVM::ModelSVM(string actionsFilePath)
{
    this->actionsFilePath=actionsFilePath+"/SVMModels.ini";
    numActions=0;
    read();
}

void ModelSVM::computeOutput(std::deque<yarp::sig::Vector> &features, yarp::sig::Vector &output)
{
    Vector descriptor;
    concatenate(features,descriptor);
    for (int i=0; i<numActions; i++)
    {
        double score=dot(models.at(i),descriptor);
        output.push_back(score);
    }
}

void ModelSVM::concatenate(std::deque<yarp::sig::Vector> &features, yarp::sig::Vector &descriptor)
{
    for (unsigned int i=0; i<features.size(); i++)
        for (size_t j=0; j<features.at(i).size(); j++)
            descriptor.push_back(features.at(i)[j]);
}

void ModelSVM::trainModel()
{

}

int ModelSVM::getNumActions()
{
    return 0;
}

bool ModelSVM::read()
{
    yarp::os::ResourceFinder config;
    config.setDefaultContext("demoGestureRecognition/conf");
    config.setDefaultConfigFile("SVMModels.ini");
    int argc=0; char *argv[1];
    config.configure("ICUB_ROOT",argc,argv);

    Bottle& bgeneral=config.findGroup("GENERAL");
    numActions=bgeneral.find("numActions").asInt();
    dictionarySize=bgeneral.find("dictionarySize").asInt();
    buffer=bgeneral.find("bufferSize").asInt();
    nFeatures=bgeneral.find("nFeatures").asInt();
    frameFeatureSize=bgeneral.find("frameFeatureSize").asInt();

    for (int i=0; i<numActions; i++)
    {
        ostringstream oss;
        oss << (i+1);
        string num = oss.str();
        string actionName="ACTION"+num;

        Bottle& baction=config.findGroup(actionName.c_str());
        Bottle* w=baction.find("w").asList();
        Vector tmp;
        tmp.resize(frameFeatureSize*buffer);
        for (int i=0; i<(frameFeatureSize*buffer); i++)
            tmp[i]=w->get(i).asDouble();
        models.push_back(tmp);
    }

    return true;
}

void ModelSVM::write()
{
    ofstream outFile(actionsFilePath.c_str(), ios_base::trunc);
    outFile << "[GENERAL]\n";
    outFile << "numActions " << numActions << "\n";
    outFile << "dictionarySize " << dictionarySize << "\n";
    outFile << "bufferSize " << buffer << "\n";
    outFile << "nFeatures " << nFeatures << "\n";
    outFile << "frameFeatureSize " << frameFeatureSize << "\n";

    for (int i=0; i<numActions; i++)
    {
        ostringstream oss;
        oss << (i+1);
        string num = oss.str();
        string actionName="ACTION"+num;

        outFile << "[" << actionName << "]\n";
        outFile << "w (" << models.at(i).toString() << ")\n";
    }
    outFile.close();
}

void ModelSVM::printModel()
{

}

ModelSVM::~ModelSVM()
{
    //write();
    models.clear();
}
