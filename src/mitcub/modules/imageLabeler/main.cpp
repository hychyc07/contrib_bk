


#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PreciselyTimed.h>



#include <cv.h>
#include <highgui.h>

#include <string>
#include <vector>
#include <list>

#include <iostream>
#include <fstream>


using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;



class LabelerThread: public RateThread
{
private:
    ResourceFinder                  &rf;

    Port                            port_label;
    BufferedPort<Image>             port_time_stamp;

    Bottle                          labels;

    Semaphore                       mutex;

public:
    LabelerThread(ResourceFinder &_rf)
        :RateThread(20),rf(_rf)
    {
    }

    virtual bool threadInit()
    {
        this->setRate(rf.check("rate",Value(20)).asInt());

        string name=rf.check("name",Value("labeler")).asString().c_str();

        port_label.open(("/"+name+"/label:o").c_str());
        port_time_stamp.open(("/"+name+"/time_stamp:i").c_str());

        labels.clear();
        labels.addString("none");

        return true;
    }


    virtual void run()
    {
        mutex.wait();
        if(port_time_stamp.getInputCount()>0)
        {
            port_time_stamp.read(true);

            Stamp time_stamp;
            port_time_stamp.getEnvelope(time_stamp);

            port_label.setEnvelope(time_stamp);

            port_label.write(labels);
        }
        mutex.post();
    }

    virtual void threadRelease()
    {
        port_time_stamp.interrupt();
        port_time_stamp.close();

        port_label.interrupt();
        port_label.close();
    }

    bool execReq(const Bottle &command, Bottle &reply)
    {
        mutex.wait();

        labels.clear();
        if(command.size()==0 || command.get(0).asString()=="" || command.get(0).asString()=="none")
            labels.addString("none");
        else
            labels=command;

        mutex.post();

        reply.addString("new label: ");
        reply.addString(labels.toString());

        return true;
    }
};







class LabelerModule: public RFModule
{
private:
    LabelerThread                  *thr;

    Port                        rpcPort;


public:
    LabelerModule()
    {}

    virtual bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("image_labeler")).asString().c_str();

        thr=new LabelerThread(rf);
        thr->start();

        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);

        return true;
    }

    virtual bool interruptModule()
    {
        rpcPort.interrupt();

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;

        rpcPort.close();

        return true;
    }


    virtual double getPeriod()
    {
        return 0.1;
    }

    virtual bool updateModule()
    {
        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        if(thr->execReq(command,reply))
            return true;
        else
            return RFModule::respond(command,reply);
    }
};






int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;


    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("image_labeler/conf");
    rf.setDefaultConfigFile("config.ini");
    rf.setDefault("name","image_labeler");
    rf.configure("ICUB_ROOT",argc,argv);

    LabelerModule mod;

    return mod.runModule(rf);
}




