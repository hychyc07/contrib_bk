#include "linearClassifierModule.h"
#include <yarp/os/Stamp.h>

bool linearClassifierModule::configure(yarp::os::ResourceFinder &rf)
{    


    string moduleName = rf.check("name", Value("linearClassifier"), "module name (string)").asString().c_str();
    setName(moduleName.c_str());


    handlerPortName = "/";
    handlerPortName += getName(rf.check("CommandPort",Value("/rpc"),"Output image port (string)").asString().c_str());



    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << ": unable to open port " << handlerPortName << endl;
        return false;
    }

    attach(handlerPort);


    lCThread = new linearClassifierThread(rf,&handlerPort);

    lCThread->start(); 

    return true ;

}


bool linearClassifierModule::interruptModule()
{
   lCThread->stop();


    return true;
}


bool linearClassifierModule::close()
{

    lCThread->stop();
    delete lCThread;

    return true;
}


bool linearClassifierModule::respond(const Bottle& command, Bottle& reply) 
{
    if(command.size()==0)
    {
        reply.addString("nack");
        return true;
    }

    if(command.get(0).asString()=="save" && command.size()==2)
    {
        string obj=command.get(1).asString().c_str();
        this->lCThread->prepareObjPath(obj);
        reply.addString("ack");
        return true;

    }

    if(command.get(0).asString()=="stop")
    {
    
        this->lCThread->stopAll();
        reply.addString("ack");
        return true;

    } 

    if(command.get(0).asString()=="objList")
    {
    
        this->lCThread->getClassList(reply);
        reply.addString("ack");
        return true;

    } 

    if(command.get(0).asString()=="train")
    {
    
        this->lCThread->trainClassifiers();
        reply.addString("ack");
        return true;
    }

    if(command.get(0).asString()=="recognize")
    {
    
        this->lCThread->startRecognition();
        reply.addString("ack");
        return true;
    }

    if(command.get(0).asString()=="forget" && command.size()>1)
    {
        string className=command.get(1).asString().c_str();
        if(className=="all")
            this->lCThread->forgetAll();
        else
            this->lCThread->forgetClass(className,true);
        reply.addString("ack");
        return true;
    }


    reply.addString("nack");
    return true;
}


bool linearClassifierModule::updateModule()
{
    return true;
}



double linearClassifierModule::getPeriod()
{
    return 0.1;
}
