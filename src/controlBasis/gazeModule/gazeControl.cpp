// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include <iostream>
#include <string>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/Random.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>

#include <controlBasis/RunnableControlLaw.h>
#include <controlBasis/PotentialFunctionRegister.h>
#include <controlBasis/JacobianRegister.h>

#include <controlBasis/iCubHeadConfigurationVariables.h>
#include <controlBasis/iCubEyeConfigurationVariables.h>
#include <controlBasis/YARPBlobTrackerHeading.h>
#include <controlBasis/HeadingFovea.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace std;
using namespace CB;

#define COMMAND_VOCAB_HELP VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_SET VOCAB3('s','e','t')
#define COMMAND_VOCAB_RUN VOCAB3('r','u','n')
#define COMMAND_VOCAB_STOP VOCAB4('s','t','o','p')
#define COMMAND_VOCAB_CLEAR VOCAB4('c','l','e','a')
#define COMMAND_VOCAB_GAIN VOCAB4('g','a','i','n')
#define COMMAND_VOCAB_FAILED VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_OK VOCAB2('o','k')
#define COMMAND_VOCAB_QUIT VOCAB4('q','u','i','t')

class gazeControlCoordinator : public RFModule {

protected:

    Port handlerPort; // to handle messagees
    Semaphore mutex;

    enum BlobSensors {
        AVERAGE_BLOB=0,
        LEFT_BLOB,
        RIGHT_BLOB,
        NUM_SENSORS
    };
    
    enum Effectors {
        HEAD=0,
        EYES,
        NUM_EFFECTORS
    };
    
    iCubHeadConfigurationVariables *iCubHead;
    iCubEyeConfigurationVariables  *iCubEyes;

    YARPBlobTrackerHeading *leftBlob;
    YARPBlobTrackerHeading *rightBlob;
    YARPBlobTrackerHeading *avgBlob;
    Heading *fovea;

    Controller * smooth_pursuit[NUM_SENSORS][NUM_EFFECTORS];    
    RunnableControlLaw controlLaw;

    string robotPrefix;

    bool controlLawRunning;
    bool resourcesStarted;
    bool useVergence;

    bool runningSaccade;
    BlobSensors saccadeSensor;
    Effectors saccadeEffector;

public:

    gazeControlCoordinator() { 
        controlLawRunning = false;
        resourcesStarted = false;
        useVergence = false;
        runningSaccade = false;
    }

    ~gazeControlCoordinator() { 
        
        stopResources();

        for(int i=0; i<NUM_SENSORS; i++) {
            for(int j=0; j<NUM_EFFECTORS; j++) {
                delete smooth_pursuit[i][j];
            }
        }

    }

protected:

    bool startResources() {

        if(resourcesStarted) {
            return true;
        }

        bool simulationMode = true;
        if(robotPrefix=="/icub") simulationMode = false;

        string iCubDir(getenv("ICUB_ROOT"));
        string configFilePath = iCubDir+"/app/controlBasis/conf/";
        string headConfigFile = configFilePath+"head-pt.dh";        
        string eyePTConfigFile = configFilePath+"eyes-pt.dh";        
        string eyePTVConfigFile = configFilePath+"eyes-ptv.dh";        
        string headVelPort = robotPrefix + "/vc/head";
        string leftBlobName = "/blobTracker" + robotPrefix + "/left/sat";
        string rightBlobName = "/blobTracker" + robotPrefix + "/right/sat";
        
        iCubHead = new iCubHeadConfigurationVariables(simulationMode);
        iCubHead->loadConfig(headConfigFile);
        iCubHead->startResource();        
        iCubHead->setVelocityControlMode(true, headVelPort);

        iCubEyes = new iCubEyeConfigurationVariables(simulationMode, useVergence);
        if(useVergence) {
            iCubEyes->loadConfig(eyePTVConfigFile);
        } else {
            iCubEyes->loadConfig(eyePTConfigFile);
        }
        iCubEyes->startResource();        
        iCubEyes->setVelocityControlMode(true, headVelPort);

        leftBlob = new YARPBlobTrackerHeading(leftBlobName);
        leftBlob->startResource();
        rightBlob = new YARPBlobTrackerHeading(rightBlobName);
        rightBlob->startResource();
        avgBlob = new YARPBlobTrackerHeading(leftBlobName,rightBlobName);
        avgBlob->startResource();

        fovea = new HeadingFovea("/icub");       
        fovea->setUpdateDelay(0.02);
        fovea->startResource();

        resourcesStarted = true;
        return true;
    }

    bool stopResources() {

        resourcesStarted = false;        

        iCubHead->stopResource();        
        iCubEyes->stopResource();

        leftBlob->stopResource();
        rightBlob->stopResource();
        avgBlob->stopResource();

        fovea->stopResource();

    }

    void configureControllers() {
        
        string avgSensor, leftSensor, rightSensor;
        string headEffector, eyesEffector;
        string pf, ref;

        pf = "/cb/heading/squared_error_pf";

        ref = "/cb/heading/icub/fovea";

        avgSensor    = "/cb/heading/blobTracker/avg";
        leftSensor  = "/cb/heading/blobTracker" + robotPrefix + "/left/sat";
        rightSensor = "/cb/heading/blobTracker" + robotPrefix + "/right/sat";

        headEffector = "/cb/configuration" + robotPrefix + "/head";
        eyesEffector = "/cb/configuration" + robotPrefix + "/eyes-pt";
        if(useVergence) eyesEffector += "v";

        smooth_pursuit[AVERAGE_BLOB][HEAD] = new Controller(avgSensor,   ref,  pf,  headEffector);
        smooth_pursuit[LEFT_BLOB][HEAD]    = new Controller(leftSensor,  ref,  pf,  headEffector);
        smooth_pursuit[RIGHT_BLOB][HEAD]   = new Controller(rightSensor, ref,  pf,  headEffector);
        smooth_pursuit[AVERAGE_BLOB][EYES] = new Controller(avgSensor,   ref,  pf,  eyesEffector);
        smooth_pursuit[LEFT_BLOB][EYES]    = new Controller(leftSensor,  ref,  pf,  eyesEffector);
        smooth_pursuit[RIGHT_BLOB][EYES]   = new Controller(rightSensor, ref,  pf,  eyesEffector);

        //    saccade_head.addController(sensor,reference,pf,effector,gain);
        
    }

    bool setSaccadeParams(string s, string e) {

        bool b_sen = true;
        bool b_eff = true;
        bool b = false;
 
        if(s=="AVG") {
            saccadeSensor = AVERAGE_BLOB;
        } else if(s=="LEFT") {
            saccadeSensor = LEFT_BLOB;
        } else if(s=="RIGHT") {
            saccadeSensor = RIGHT_BLOB;
        } else {
            b_sen = false;
        }

        if(e=="HEAD") {
            saccadeEffector = HEAD;
        } else if(e=="EYES") {
            saccadeEffector = EYES;
        } else {
            b_eff = false;
        }

        b = b_sen && b_eff;

        if(!b) {
            cout << "incorrect Saccade paramns("<<s<<","<<e<<")"<<endl;
        }

        return b;
    }

    bool runSaccade() {

        if(!runningSaccade) {
            cout << "Saccade Params not set!!" << endl;
            return false;
        }

        Vector blobData, configData;
        if(saccadeSensor==AVERAGE_BLOB) {
            cout << "getting blob data from Average Blob Sensor" << endl;
            blobData = avgBlob->getResourceData();
        } else if(saccadeSensor==LEFT_BLOB) {
            cout << "getting blob data from Left Blob Sensor" << endl;
            blobData = leftBlob->getResourceData();
        } else if(saccadeSensor==RIGHT_BLOB) {
            cout << "getting blob data from Right Blob Sensor" << endl;
            blobData = rightBlob->getResourceData();
        }
        if(blobData.size()!=2) {
            cout << "Saccade data wrong size -- " << blobData.size() << endl;
            return false;
        }
        Vector v;
        double scaleFactor = 1;

        if(saccadeEffector==HEAD) {
            configData = iCubHead->getResourceData();
            v.resize(2);
            v[0] = (configData[0] - blobData[1])*scaleFactor;
            v[1] = (configData[1] - blobData[0])*scaleFactor;
            iCubHead->setJointPositionDirect(0,v[0]);
            iCubHead->setJointPositionDirect(2,v[1]);
        } else if(saccadeEffector==EYES) {
            configData = iCubEyes->getResourceData();
            if(useVergence) {
                v.resize(3);               
                v[0] = configData[1] - blobData[1];
                v[1] = configData[0] + blobData[0]/2.0;
                v[2] = configData[2] - blobData[0]/2.0;
                iCubEyes->setJointPositionDirect(3,v[0]);
                iCubEyes->setJointPositionDirect(4,v[1]);
                iCubEyes->setJointPositionDirect(5,v[2]);
            } else {
                v.resize(2);               
                v[0] = configData[1] - blobData[1];
                v[1] = configData[0] + blobData[0];
                iCubEyes->setJointPositionDirect(3,v[0]);
                iCubEyes->setJointPositionDirect(4,v[1]);
            }
        }

        for(int i=0; i< configData.size(); i++) {
            cout << "configData["<<i<<"]: " << configData[i] << endl;
        }
        for(int i=0; i< blobData.size(); i++) {
            cout << "blobData["<<i<<"]: " << blobData[i] << endl;
        }
        for(int i=0; i<v.size(); i++) {
            cout << "v["<<i<<"]: " << v[i] << endl;
        }
        cout << endl;

        return true;
        
    }

    bool configure(ResourceFinder &rf)  {
        
        string iCubDir(getenv("ICUB_ROOT"));
        
        if(rf.check("robot")) {
            robotPrefix=rf.find("robot").asString().c_str();
        } 
        if(rf.check("verge")) {
            useVergence=(bool)(rf.find("verge").asInt());
        } 
        
        handlerPort.open("/gazeControlCoordinator/rpc:i");
        attach(handlerPort);
        attachTerminal();
        
        startResources();
        
        configureControllers();
        return true;
    }
    
    bool respond(const Bottle &command, Bottle &reply)  {
        
        bool ok = true;
        bool rec = false; // is the command recognized?

        string sen, eff, cont;
        mutex.wait();
        
        switch (command.get(0).asVocab()) {
        case COMMAND_VOCAB_HELP:
            rec = true;
            {
                reply.addString("help");
                // print help
                ok = true;
            }        
            break;
        case COMMAND_VOCAB_SET:
            controlLaw.resetControlLaw();
            rec = true;
            cout << "Controller [ SmoothPursuit | Saccade ]: ";
            cin >> cont;
            cout << "Sensor [ AVG | LEFT | RIGHT ]: ";
            cin >> sen;
            cout << "Effector [ HEAD | EYES ]: ";
            cin >> eff;

            if(cont=="SmoothPursuit") {
                if((sen == "AVG") && (eff == "HEAD")) {
                    controlLaw.addController(smooth_pursuit[AVERAGE_BLOB][HEAD]);
                } else if((sen == "AVG") && (eff == "EYES")) {
                    controlLaw.addController(smooth_pursuit[AVERAGE_BLOB][EYES]);
                } else if((sen == "LEFT") && (eff == "HEAD")) {
                    controlLaw.addController(smooth_pursuit[LEFT_BLOB][HEAD]);
                } else if((sen == "LEFT") && (eff == "EYES")) {
                    controlLaw.addController(smooth_pursuit[LEFT_BLOB][EYES]);
                } else if((sen == "RIGHT") && (eff == "HEAD")) {
                    controlLaw.addController(smooth_pursuit[RIGHT_BLOB][HEAD]);
                } else if((sen == "RIGHT") && (eff == "EYES")) {
                    controlLaw.addController(smooth_pursuit[RIGHT_BLOB][EYES]);
                }
                runningSaccade = false;
            } else if(cont=="Saccade"){
                setSaccadeParams(sen,eff);
                runningSaccade = true;
            } else {
                cout << "unknown controller ("<<cont<<","<<sen<<","<<eff<<")"<<endl;
            }
            ok = true;
            break;
        case COMMAND_VOCAB_RUN:
            rec = true;
            {
                if(!runningSaccade) {
                    if (!controlLawRunning) {
                        controlLaw.startAction();
                    }
                    controlLawRunning = true;
                } else {
                    runSaccade();
                }
            }
            ok = true; 
            break;
        case COMMAND_VOCAB_STOP:
            rec = true;
            {
                if (controlLawRunning) {
                    controlLaw.stopAction();
                }
                controlLawRunning = false;
            }
            ok = true;
            break;                
        case COMMAND_VOCAB_CLEAR:
            rec = true;
            {
                if (controlLawRunning) {
                    controlLaw.stopAction();
                }
                controlLawRunning = false;
                controlLaw.resetControlLaw();
            }
            ok = true;
            break;
        }
        mutex.post();
        
        if (!rec)
            ok = RFModule::respond(command,reply);
        
        if (!ok) {
            reply.clear();
            reply.addVocab(COMMAND_VOCAB_FAILED);
        }
        else
            reply.addVocab(COMMAND_VOCAB_OK);
                
        return ok;
    }
    
    bool close() {
        handlerPort.close();
        return true;
    }
    
    double getPeriod() {
        return 1;
    }
    
    bool updateModule() {
        Time::delay(1);
        return true;
    }
    
};
    
int main(int argc, char *argv[]) {
    
    Network yarp;
    string robotPrefix = "/icubSim";
    
    gazeControlCoordinator mod;
    
    registerPotentialFunctions();
    registerJacobians();
    
    ResourceFinder rf;
    rf.setDefault("robot",robotPrefix.c_str());
    
    rf.configure("ICUB_ROOT", argc, argv);
    rf.setVerbose(true);
    
    cout<<"Starting gazeControlCoordinator Module..."<<endl;
    return mod.runModule(rf);

}


