
#ifndef __EXTRACTOR_MODULE__
#define __EXTRACTOR_MODULE__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>


#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>

#include <vector>
#include <deque>

#include <string>
#include <fstream>

#include "Extractor.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;



class ExtractorModule: public RFModule
{
private:
    int                         state;
    int                         mode;
    int                         max_samples;

    vector<Extractor*>          extractors;
    vector<int>                 extractors_sample_count;
    vector<Extractor*>          dictionarizers;

    int                         subsample;
    int                         subsample_itr;

    BufferedPort<Image>         port_img;
    BufferedPort<Bottle>        port_labels;

    RpcClient                   port_dataset_player;

    RpcServer                   rpcPort;

    bool                        extracting;

    bool registerExtractors(ResourceFinder &rf);
    bool startExperiment(string extractor_name="");
    bool initExtractors(const string &extractor_name);

public:
    ExtractorModule()
    {}

    virtual bool configure(ResourceFinder &rf);

    virtual bool interruptModule();

    virtual bool close();


    virtual double getPeriod()
    {
        return 0.1;
    }

    virtual bool updateModule();

    virtual bool respond(const Bottle &command, Bottle &reply);


};


#endif
