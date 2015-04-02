#include <iostream>
#include <vector>
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace std;
using namespace yarp::os;

class ObjRecControlModule:public RFModule
{
    Port handlerPort; //a port to handle messages
    Port outPort;
    Port inPort;
    std::vector<Bottle> script;
    int pc;
    int timeStart;
    Bottle *objectNames;

public:

    double getPeriod()
    {
        return 0.5; //module periodicity (seconds)
    }

    /*
    * This is our main function. Will be called periodically every getPeriod() seconds.
    */
    bool updateModule()
    {

        if (pc >= script.size())
            return true;

        int timeSec = Time::now() - timeStart;
        std::cout << pc+1 << "/" << script.size() << " (" << timeSec/60 << "m)" << ": " << script[pc].toString() << std::endl;

        Bottle rpcRet;

        outPort.write(script[pc], rpcRet);
        //if (rpcRet.size() == 0)
        //{
            //// timeout: message got lost -> send last command again
            //std::cout << "INFO: timeout for last command, resending message '" << script[pc].toString() << std::endl;;
            //return true;
        //}
        //if (pc > 0)
        //{
            if ((script[pc].get(0).asString() != "set")
            && (script[pc].get(0).asString() != "look")
            && (script[pc].get(0).asString() != "snapshot"))
            {
                Bottle ret;
                inPort.read(ret,false); // wait till operation completed
                if (ret.size() == 0)
                    return true;
                std::cout << "Received: " << ret.toString() << std::endl;
            }
        //}

        pc++; 

        return true;
    }

    /*
    * Message handler. Just echo all received messages.
    */
    bool respond(const Bottle& command, Bottle& reply) 
    {
        cout<<"Got something, echo is on"<<endl;
        if (command.get(0).asString()=="quit")
            return false;     
        else
            reply=command;
        return true;
    }

    /* 
    * Configure function. Receive a previously initialized
    * resource finder object. Use it to configure your module.
    * Open port and attach it to message handler.
    */
    bool configure(yarp::os::ResourceFinder &rf)
    {
        std::string moduleName = rf.check("name", Value("activeObjRecControl")).asString().c_str();
        std::string objRecName = rf.check("objRecName", Value("activeObjRec")).asString().c_str();
        //int rotation = rf.check("rotation", Value(0)).asInt();
        //std::cout << "rotation: " << rotation << std::endl;

        std::string outPortName = "/" + moduleName + ":o";
        std::string inPortName = "/" + moduleName + ":i";

        std::string remoteInPortName = "/" + objRecName + "/rpc:i";
        std::string remoteOutPortName = "/" + objRecName + ":o";

        objectNames = rf.find("objects").asList();
        if (objectNames)
        {
            for (int i=0; i<objectNames->size(); i++)
                std::cout << objectNames->get(i).asString() << std::endl;
        }

        //outPort.setTimeout(10);
        std::string handlerPortName = "/" + moduleName + "/rpc";
        handlerPort.open(handlerPortName.c_str());
        outPort.open(outPortName.c_str());
        inPort.open(inPortName.c_str());
        attach(handlerPort);

        Network::connect(outPortName.c_str(), remoteInPortName.c_str(), "tcp"); 
        Network::connect(remoteOutPortName.c_str(), inPortName.c_str(), "tcp"); 


        configureProgram();

        pc = 0;

        timeStart = Time::now();

        return true;
    }

    /*
    * Interrupt function.
    */
    bool interruptModule()
    {
        cout<<"Interrupting your module, for port cleanup"<<endl;
        handlerPort.interrupt();
        std::cout << "done" << std::endl;
        return true;
    }

    /*
    * Close function, to perform cleanup.
    */
    bool close()
    {
        cout<<"Calling close function\n";
        handlerPort.close();
        outPort.close();
        inPort.close();
        return true;
    }

    void configureProgram()
    {
        script.clear();

        //runVtmEvaluation(-30, "./data_sim/vtm_results/rotation_30neg");
        runVtmEvaluation(-15, "./data_sim/vtm_results/rotation_15neg");
        runVtmEvaluation(0, "./data_sim/vtm_results/rotation_0");
        runVtmEvaluation(15, "./data_sim/vtm_results/rotation_15");
        runVtmEvaluation(30, "./data_sim/vtm_results/rotation_30");
        runVtmEvaluation(45, "./data_sim/vtm_results/rotation_45");
    }

        //Bottle b;
        //for (int i = 0; i < objectNames->size(); i++)
        //{
            //std::string objectname = objectNames->get(i).asString().c_str();

            ////b.clear(); b.addString("set"); b.addString("boosting"); b.addInt(1);    script.push_back(b);
            ////b.clear(); b.addString("set"); b.addString("planning"); b.addInt(1);    script.push_back(b);
            ////b.clear(); b.addString("recog"); b.addString(objectname.c_str());   script.push_back(b);
            ////b.clear(); b.addString("set"); b.addString("planning"); b.addInt(0);    script.push_back(b);
            ////b.clear(); b.addString("recog"); b.addString(objectname.c_str());   script.push_back(b);
            ////b.clear(); b.addString("set"); b.addString("boosting"); b.addInt(0);    script.push_back(b);
            ////b.clear(); b.addString("set"); b.addString("planning"); b.addInt(1);    script.push_back(b);
            ////b.clear(); b.addString("recog"); b.addString(objectname.c_str());   script.push_back(b);
            ////b.clear(); b.addString("set"); b.addString("planning"); b.addInt(0);    script.push_back(b);
            ////b.clear(); b.addString("recog"); b.addString(objectname.c_str());   script.push_back(b);

            /////////////////
            //// Snapshot
            /////////////////
            ////std::string imgname = objectname+".png";
            ////b.clear(); b.addString("look"); b.addInt(65); b.addInt(200); script.push_back(b);
            ////b.clear(); b.addString("snapshot"); b.addString(imgname.c_str()); script.push_back(b);
            /////////////////



            /////////////////
            //// VTM
            /////////////////
            ////b.clear(); b.addString("buildvtm"); b.addString(objectname.c_str()); script.push_back(b);
            /////////////////


        //}
        

    void runRecogEvaluation(int rotation , const std::string &resultDir)
    {
        setRecogResultDir(resultDir);
        for (int i = 0; i < objectNames->size(); i++)
        {
            std::string objectname = objectNames->get(i).asString().c_str(); 
            openHand();
            graspBox(objectname, rotation);

            setBoosting(true);
            setPlanning(true);
            recogVtm(objectname);
            
            setPlanning(false);
            recogVtm(objectname);

            setBoosting(false);
            setPlanning(true);
            recogVtm(objectname);

            setPlanning(false);
            recogVtm(objectname);
        }
    }

    void runVtmEvaluation(int rotation, const std::string &resultDir)
    {
        setVtmResultDir(resultDir);

        for (int i = 0; i < objectNames->size(); i++)
        {
            std::string objectname = objectNames->get(i).asString().c_str(); 
            openHand();
            graspBox(objectname, rotation);
            recogVtm(objectname);
        }
    }

    void setRecogResultDir(const std::string &dir)
    {
        Bottle b; b.addString("set"); b.addString("recogresultdir"); b.addString(dir.c_str()); script.push_back(b);
    }

    void setVtmResultDir(const std::string &dir)
    {
        Bottle b; 
        ConstString s = dir.c_str();
        b.addString("set"); b.addString("vtmresultdir"); b.addString(s); script.push_back(b);
    }

    void openHand()
    {
        Bottle b; b.addString("open");  b.addString("hand"); script.push_back(b);
    }

    void graspBox(const std::string &name, int rotation)
    {
        Bottle b; b.addString("grasp"); b.addString("box.x"); b.addString(name.c_str()); b.addInt(rotation);     script.push_back(b);
    }

    void setBoosting(bool state)
    {
        Bottle b; b.addString("set"); b.addString("boosting"); b.addInt(1);    script.push_back(b);
    }

    void setPlanning(bool state)
    {
        Bottle b; b.addString("set"); b.addString("planning"); b.addInt(1);    script.push_back(b);
    }

    void recog(const std::string &truelabel)
    {
        Bottle b; b.addString("recog"); b.addString(truelabel.c_str());   script.push_back(b);
    }

    void recogVtm(const std::string &truelabel)
    {
        Bottle b; b.addString("vtm"); b.addString(truelabel.c_str()); script.push_back(b);
    }
    
};

int main(int argc, char * argv[])
{
    Network yarp;

    ObjRecControlModule module;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("config.ini");      //overridden by --from parameter
    rf.setDefaultContext("./conf");             //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

    cout<<"Configure module..."<<endl;
    module.configure(rf);
    cout<<"Start module..."<<endl;
    module.runModule();

    cout<<"Main returning..."<<endl;
    return 0;
}



