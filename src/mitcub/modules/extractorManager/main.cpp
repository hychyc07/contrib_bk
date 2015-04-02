

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>


#include <string>
#include <deque>

using namespace std;
using namespace yarp;
using namespace yarp::os;



struct Dataset
{
    string      name;
    string      path;
};



class ManagerModule: public RFModule
{
private:
    RpcClient                   port_rpc_dataSetPlayer;
    RpcClient                   port_rpc_featureExtractor;
    Port                        port_rpc;

    deque<Dataset>             datasets;
    unsigned int                datasets_curr_idx;

public:
    ManagerModule(){}

    virtual bool configure(ResourceFinder &rf)
    {
        if(!rf.check("datasets") || rf.find("datasets").asList()->size()==0)
        {
            fprintf(stdout,"No datasets found. Closing.\n");
            return false;
        }

        Bottle &bDatasets=rf.findGroup("datasets");
        datasets.resize(bDatasets.size()-1);

        for(int i=1; i<bDatasets.size(); i++)
        {
            datasets[i-1].name=bDatasets.get(i).asList()->get(0).asString().c_str();
            datasets[i-1].path=bDatasets.get(i).asList()->get(1).asString().c_str();
        }

        datasets_curr_idx=0;


        //open the ports
        string name=rf.find("name").asString().c_str();

        port_rpc_dataSetPlayer.open(("/"+name+"/dataSetPlayer:io").c_str());
        port_rpc_featureExtractor.open(("/"+name+"/featureExtractor:io").c_str());
        port_rpc.open(("/"+name+"/rpc").c_str());
        attach(port_rpc);


        return true;
    }

    virtual bool updateModule()
    {
        if(port_rpc_dataSetPlayer.getOutputCount()==0 || port_rpc_featureExtractor.getOutputCount()==0)
            return true;

        Bottle bReply;

        //check if the featureExtractor is busy
        Bottle bQuery; bQuery.clear();
        bQuery.addString("status");

        bReply.clear();
        port_rpc_featureExtractor.write(bQuery,bReply);

        if(bReply.isNull() || bReply.size()==0 || bReply.get(0).asString()!="idle")
            return true;

        //if all the datasets have been evaluated interrupt the module
        if(datasets.size()<=datasets_curr_idx)
        {
            Bottle bQuit;
            bQuit.addString("quit");
            port_rpc_featureExtractor.write(bQuit,bReply);
            port_rpc_dataSetPlayer.write(bQuit,bReply);

            stopModule();
            return true;
        }

        //load the new dataset
        Bottle bLoad; bLoad.clear();
        bLoad.addString("load");
        bLoad.addString(datasets[datasets_curr_idx].path.c_str());


        bReply.clear();
        port_rpc_dataSetPlayer.write(bLoad,bReply);

        //if the dataset could not be loaded, go on to the next one
        if(bReply.isNull() || bReply.size()==0 || bReply.get(0).asString()!="ok")
        {
            datasets_curr_idx++;
            return true;
        }

        //if everything has been successfully loaded start the new extraction
        Bottle bExtract; bExtract.clear();
        bExtract.addString("start");
        bExtract.addString(datasets[datasets_curr_idx].name.c_str());

        datasets_curr_idx++;


        bReply.clear();
        port_rpc_featureExtractor.write(bExtract,bReply);
        if(bReply.isNull() || bReply.size()==0 || bReply.get(0).asString()!="started")
            return true;

        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        return RFModule::respond(command,reply);
    }


    virtual bool close()
    {
        port_rpc_dataSetPlayer.close();
        port_rpc_featureExtractor.close();
        port_rpc.close();

        return true;
    }

    virtual double getPeriod()
    {
        return 0.1;
    }
};

















int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("extractorManager/conf");
    rf.setDefaultConfigFile("config.ini");
    rf.setDefault("name","extractorManager");
    rf.configure("ICUB_ROOT",argc,argv);

    Network yarp;

    bool started_network=false;
    if (!yarp.checkNetwork())
    {
        //make a new server start
        int g=0;
    }

    ManagerModule mod;

    return mod.runModule(rf);
}

