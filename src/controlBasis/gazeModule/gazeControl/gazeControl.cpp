// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include <iostream>
#include <string>
#include <string.h>

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

#include <controlBasis/HeadingFovea.h>

#include "AdmissibilityPredictor.h"
#include "GazeControlPolicyLearner.h"

#include "Saccade.h"
#include "SmoothPursuit.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace std;
using namespace CB;
using namespace gc;

#define COMMAND_VOCAB_HELP VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_SET VOCAB3('s','e','t')
#define COMMAND_VOCAB_ADD VOCAB3('a','d','d')
#define COMMAND_VOCAB_RUN VOCAB3('r','u','n')
#define COMMAND_VOCAB_STOP VOCAB4('s','t','o','p')
#define COMMAND_VOCAB_OBSERVE VOCAB4('o','b','s','e')
#define COMMAND_VOCAB_LEARN VOCAB4('l','e','a','r')
#define COMMAND_VOCAB_HEAD VOCAB4('h','e','a','d')

#define COMMAND_VOCAB_CLEAR VOCAB4('c','l','e','a')
#define COMMAND_VOCAB_GAIN VOCAB4('g','a','i','n')

#define COMMAND_VOCAB_FAILED VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_OK VOCAB2('o','k')
#define COMMAND_VOCAB_QUIT VOCAB4('q','u','i','t')

class BlobInfo {

public:
    double Gamma_u;
    double Gamma_v;
    double Gamma_u_dot;
    double Gamma_v_dot;
};

class gazeControlCoordinator : public RFModule {

public:
        
    enum ControlMode {
        SACCADE=0,
        SMOOTH_PURSUIT,
        NUM_CONTROL_MODES
    };
    
    enum Sensor {
        BLOB_AVG=0,
        BLOB_LEFT,
        BLOB_RIGHT,
        NUM_SENSORS
    };
    
    enum Effector {
        HEAD=0,
        EYES,
        NUM_EFFECTORS
    };

protected:

    Port handlerPort; // to handle messagees
    BufferedPort<Bottle> headPort;

    Semaphore mutex;

    iCubHeadConfigurationVariables *iCubHead;
    iCubEyeConfigurationVariables  *iCubEyes;

    YARPBlobTrackerHeading *blob_l;
    YARPBlobTrackerHeading *blob_r;
    YARPBlobTrackerHeading *blob_a;
    Heading *fovea;

    map<int, GazeControlAction *> actions;
    vector< pair<GazeControlAction *, GazeControlAction *> > actionPairs;

    string robotPrefix;
    bool learnPolicyMode;

    bool actionRunning;
    bool policyLearnerRunning;

    bool resourcesStarted;
    bool useVergence;
    
    ControlMode controlMode;
    Sensor sensor[2];
    Effector effector[2];
    bool compositeMode;

    bool signalPresent[NUM_SENSORS];
    bool inFovea[NUM_SENSORS];

    GazeControlPolicyLearner * policyLearner;
    AdmissibilityPredictor * predictor[NUM_CONTROL_MODES][NUM_SENSORS][NUM_EFFECTORS];

    Vector observedData;
    
    vector<BlobInfo *> blobInfo[3];

    int foveaSize;
    double foveaHeadingWindow;
    double focal_length;
    double meters_per_pixel;

public:

    gazeControlCoordinator() { 
     
        actionRunning = false;
        policyLearnerRunning = false;
        resourcesStarted = false;
        useVergence = false;
        compositeMode = false;

        clearControlLaw();

        focal_length = 0.0225;
        meters_per_pixel = 0.0001;

        int numActions = NUM_CONTROL_MODES*NUM_EFFECTORS*NUM_SENSORS;

    }

    ~gazeControlCoordinator() { 
        
        stopResources();

        int idx = 0;
        for(int k=0; k<NUM_CONTROL_MODES; k++) {
            for(int i=0; i<NUM_SENSORS; i++) {
                for(int j=0; j<NUM_EFFECTORS; j++) {
                    delete actions[idx++];
                }
            }
        }
        actions.clear();

        for(int k=0; k<3; k++) {
            if(blobInfo[k].size() != 0) {
                for(int i=0; i<blobInfo[k].size(); i++) {
                    delete blobInfo[k][i];
                }
            }
            blobInfo[k].clear();
        }

        delete predictor;
        delete policyLearner;

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
        string blob_lName = "/blobTracker" + robotPrefix + "/left";
        string blob_rName = "/blobTracker" + robotPrefix + "/right";
        
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

        blob_l = new YARPBlobTrackerHeading(blob_lName);
        blob_l->startResource();
        blob_r = new YARPBlobTrackerHeading(blob_rName);
        blob_r->startResource();
        blob_a = new YARPBlobTrackerHeading(blob_lName,blob_rName);
        blob_a->startResource();

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

        blob_l->stopResource();
        blob_r->stopResource();
        blob_a->stopResource();

        fovea->stopResource();

    }
    
    void createActions() {
        
        actions[NUM_SENSORS*NUM_EFFECTORS*SACCADE + NUM_EFFECTORS*BLOB_AVG    + HEAD] = new Saccade(blob_a,iCubHead); // 0
        actions[NUM_SENSORS*NUM_EFFECTORS*SACCADE + NUM_EFFECTORS*BLOB_AVG    + EYES] = new Saccade(blob_a,iCubEyes); // 1
        actions[NUM_SENSORS*NUM_EFFECTORS*SACCADE + NUM_EFFECTORS*BLOB_LEFT   + HEAD] = new Saccade(blob_l,iCubHead); // 2
        actions[NUM_SENSORS*NUM_EFFECTORS*SACCADE + NUM_EFFECTORS*BLOB_LEFT   + EYES] = new Saccade(blob_l,iCubEyes); // 3
        actions[NUM_SENSORS*NUM_EFFECTORS*SACCADE + NUM_EFFECTORS*BLOB_RIGHT  + HEAD] = new Saccade(blob_r,iCubHead); // 4
        actions[NUM_SENSORS*NUM_EFFECTORS*SACCADE + NUM_EFFECTORS*BLOB_RIGHT  + EYES] = new Saccade(blob_r,iCubEyes); // 5

        actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_AVG    + HEAD] = new SmoothPursuit(blob_a,iCubHead); // 6
        actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_AVG    + EYES] = new SmoothPursuit(blob_a,iCubEyes); // 7
        actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_LEFT   + HEAD] = new SmoothPursuit(blob_l,iCubHead); // 8
        actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_LEFT   + EYES] = new SmoothPursuit(blob_l,iCubEyes); // 9
        actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_RIGHT  + HEAD] = new SmoothPursuit(blob_r,iCubHead); // 10
        actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_RIGHT  + EYES] = new SmoothPursuit(blob_r,iCubEyes); // 11
        
        for(int i=0; i<actions.size(); i++) {
            actions[i]->setID(i);
        }

        actionPairs.clear();

        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_AVG + HEAD],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_AVG + EYES]) );
        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_AVG + HEAD],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_LEFT + EYES]) );
        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_AVG + HEAD],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_RIGHT + EYES]) );

        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_AVG   + EYES],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_AVG   + HEAD]) );
        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_AVG   + EYES],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_LEFT  + HEAD]) );
        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_AVG   + EYES],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_RIGHT + HEAD]) );

        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_LEFT  + HEAD],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_AVG   + EYES]) );
        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_LEFT  + HEAD],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_LEFT  + EYES]) );
        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_LEFT  + HEAD],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_RIGHT + EYES]) );

        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_LEFT  + EYES],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_AVG   + HEAD]) );
        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_LEFT  + EYES],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_LEFT  + HEAD]) );
        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_LEFT  + EYES],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_RIGHT + HEAD]) );

        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_RIGHT + HEAD],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_AVG   + EYES]) );
        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_RIGHT + HEAD],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_LEFT  + EYES]) );
        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_RIGHT + HEAD],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_RIGHT + EYES]) );

        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_RIGHT + EYES],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_AVG   + HEAD]) );
        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_RIGHT + EYES],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_LEFT  + HEAD]) );
        actionPairs.push_back( make_pair(actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_RIGHT + EYES],
                                         actions[NUM_SENSORS*NUM_EFFECTORS*SMOOTH_PURSUIT + NUM_EFFECTORS*BLOB_RIGHT + HEAD]) );


        cout << "Number of action pairs: " << actionPairs.size() << endl;

    }

    void clearControlLaw() {
        compositeMode = false;
        controlMode = NUM_CONTROL_MODES;
        for(int i=0; i<2; i++) {
            sensor[i] = NUM_SENSORS;
            effector[i] = NUM_EFFECTORS;
        }
    }


    void observeData() {

        /*

        for(int k=0; k<3; k++) {
            if(blobInfo[k].size() != 0) {
                for(int i=0; i<blobInfo[k].size(); i++) {
                    delete blobInfo[k][i];
                }
            }
            blobInfo[k].clear();
        }

        Vector heading, headingVel;
        BlobInfo *b;

        Matrix avgVals(3,4);
        Vector headData;

        avgVals.zero();
        
        int samples = 10;
        int numSamples[] = {0, 0, 0};

        cout << "left            right             avg" <<endl;
        cout << "====================================="<<endl;
        for(int i=0; i<samples; i++) {

            Time::delay(0.05);
       
            if(blob_a->isValid()) {

                heading = blob_a->getResourceData();
                headingVel = blob_a->getHeadingVelocity();
                    
                b = new BlobInfo();
                b->Gamma_u = heading[0];
                b->Gamma_v = heading[1];
                b->Gamma_u_dot = headingVel[0];
                b->Gamma_v_dot = headingVel[1];
                blobInfo[BLOB_AVG].push_back(b);

                avgVals[BLOB_AVG][0] += heading[0];
                avgVals[BLOB_AVG][1] += heading[1];
                avgVals[BLOB_AVG][2] += headingVel[0];
                avgVals[BLOB_AVG][3] += headingVel[1];
                
                cout << "(" << heading[0] << "," << heading[1] << ") ";
                cout << "(" << headingVel[0] << "," << headingVel[1] << ")";
                cout << "\t";
                
                numSamples[BLOB_AVG]++;

            }

            if(blob_l->isValid()) {

                heading = blob_l->getResourceData();
                headingVel = blob_l->getHeadingVelocity();
            
                b = new BlobInfo();
                b->Gamma_u = heading[0];
                b->Gamma_v = heading[1];
                b->Gamma_u_dot = headingVel[0];
                b->Gamma_v_dot = headingVel[1];
                blobInfo[BLOB_LEFT].push_back(b);
                
                avgVals[BLOB_LEFT][0] += heading[0];
                avgVals[BLOB_LEFT][1] += heading[1];
                avgVals[BLOB_LEFT][2] += headingVel[0];
                avgVals[BLOB_LEFT][3] += headingVel[1];
                
                cout << "(" << heading[0] << "," << heading[1] << ") ";
                cout << "(" << headingVel[0] << "," << headingVel[1] << ")";
                cout << "\t";
                
                numSamples[BLOB_LEFT]++;
            }

            if(blob_r->isValid()) {

                heading = blob_r->getResourceData();
                headingVel = blob_r->getHeadingVelocity();
            
                b = new BlobInfo();
                b->Gamma_u = heading[0];
                b->Gamma_v = heading[1];
                b->Gamma_u_dot = headingVel[0];
                b->Gamma_v_dot = headingVel[1];
                blobInfo[BLOB_RIGHT].push_back(b);
                
                avgVals[BLOB_RIGHT][0] += heading[0];
                avgVals[BLOB_RIGHT][1] += heading[1];
                avgVals[BLOB_RIGHT][2] += headingVel[0];
                avgVals[BLOB_RIGHT][3] += headingVel[1];
                
                cout << "(" << heading[0] << "," << heading[1] << ") ";
                cout << "(" << headingVel[0] << "," << headingVel[1] << ")";
                cout << "\t";
                
                numSamples[BLOB_RIGHT]++;

                cout <<endl;
            }
        }

        cout << "=====================================" << endl;
        for(int i=0; i<3; i++) {
            avgVals[i][0] /= numSamples[i];
            avgVals[i][1] /= numSamples[i];
            avgVals[i][2] /= numSamples[i];
            avgVals[i][3] /= numSamples[i];
        }

        cout << "(" << avgVals[0][0] << "," << avgVals[0][1] << ") ";
        cout << "(" << avgVals[0][2] << "," << avgVals[0][3] << ")";
        cout << "\t";

        cout << "(" << avgVals[1][0] << "," << avgVals[1][1] << ") ";
        cout << "(" << avgVals[1][2] << "," << avgVals[1][3] << ")";
        cout << "\t";

        cout << "(" << avgVals[2][0] << "," << avgVals[2][1] << ") ";
        cout << "(" << avgVals[2][2] << "," << avgVals[2][3] << ")";
        cout << "\t";

        cout << endl;

        // get the head data
        headData = getHeadData();

        // concatenate results
        observedData.resize(10);        
        observedData.zero();       
        for(int i=0; i<6; i++) {
            observedData[i] = headData[i];
        }        
        for(int i=0; i<4; i++) {
            observedData[6+i] = avgVals[sensor][i];
        }

        // calculate simple stats on data
        for(int i=0; i<3; i++) {
            inFovea[i] = false;
            signalPresent[i] = (numSamples[i] > 0);
            if(!signalPresent[i]) break;
            inFovea[i] = ( (fabs(avgVals[i][0]) <= foveaHeadingWindow) && (fabs(avgVals[i][1]) <= foveaHeadingWindow) && (signalPresent[i]));
            cout << "testing["<<i<<"]: (" <<avgVals[i][0]<<","<<avgVals[i][1]<<") in: " << foveaHeadingWindow << endl;
        }           

        */

        string ss = "";
        for(int i=0; i<actions.size(); i++) {
            actions[i]->evaluateState();
            ss += actions[i]->getStateString();
            //            cout << "action[" << i << "] state: " << actions[i]->getStateString().c_str() << endl;
        }
        cout << "state: [" << ss.c_str() << "]" << endl;
        
    }

    Vector getHeadData() {
        Vector d(6);
        Bottle *b = NULL;

        // try for 25 attempts at reading head data....
        for(int i=0; i<25; i++) {
            b = headPort.read(false);
            if(b!=NULL) {
                for(int i=0; i<6; i++) {
                    d[i] = b->get(i).asDouble();
                    cout << "head["<<i<<"]: " << d[i] << endl;
                }
                break;
            }            
        }
        if(b==NULL) {
            cout << "couldn't read head data..." << endl;
        }
        return d;
    }
    
    void learnAdmissibility(int n) {
    }

    bool runAction() {       
        int idx[2];

        idx[0]=NUM_SENSORS*NUM_EFFECTORS*controlMode+NUM_EFFECTORS*sensor[0]+effector[0]; 

        if(compositeMode) {
            idx[1]=NUM_SENSORS*NUM_EFFECTORS*controlMode+NUM_EFFECTORS*sensor[1]+effector[1];
        } 

        actions[idx[0]]->setVelocities();        
        if(compositeMode) {
            actions[idx[1]]->setVelocities();        
        }

        actions[idx[0]]->start();       
        if(compositeMode) {
            actions[idx[1]]->start();       
        }

        actionRunning = true;
        return true;
    }
    
    bool checkAction() {       
        int idx = NUM_SENSORS*NUM_EFFECTORS*controlMode+NUM_EFFECTORS*sensor[0]+effector[0]; 
        return actions[idx]->isRunning();       
    }
        
    bool stopAction() {       

        int idx[2];

        idx[0]=NUM_SENSORS*NUM_EFFECTORS*controlMode+NUM_EFFECTORS*sensor[0]+effector[0]; 
        idx[1]=NUM_SENSORS*NUM_EFFECTORS*controlMode+NUM_EFFECTORS*sensor[1]+effector[1]; 
        
        actions[idx[0]]->getCost();
        actions[idx[0]]->stopAction();        
        if(compositeMode) {
            actions[idx[1]]->getCost();
            actions[idx[1]]->stopAction();        
        }
        
        actions[idx[0]]->setVelocities();        
        if(compositeMode) {
            actions[idx[1]]->setVelocities();        
        }
        
        
        actionRunning = false;
        return true;
    }

    bool isAdmissible(Sensor s, Effector e) {
        // for now... want to do a lookup based on observed statistics 
        // or learned model.
        if(!signalPresent[s]) {
            return false;
        }
        return true;
    }

    bool getHeadPositionFromUser() {
    
        float pos[5];
        string inStr;
        int lim;
        if(useVergence) {
            cout << "head position [head-pan head-tilt eye-pan eye-tilt eye-verge] (deg): ";
            lim = 5;
        } else {
            cout << "head position [head-pan head-tilt eye-pan eye-tilt] (rad): ";
            lim = 4;
        }
        
        for(int i=0; i<lim; i++) {
            cin >> pos[i];
        }

        for(int i=0; i<lim; i++) {
            cout << "got pos[" << i << "]: " <<  pos[i] << endl;
        }

        iCubHead->setJointReferenceSpeed(0,10);
        iCubHead->setJointReferenceSpeed(2,10);
        iCubEyes->setJointReferenceSpeed(3,10);
        iCubEyes->setJointReferenceSpeed(4,10);
        iCubEyes->setJointReferenceSpeed(5,10);

        iCubHead->setJointPositionDirect(2,pos[0]);
        iCubHead->setJointPositionDirect(0,pos[1]);

        iCubEyes->setJointPositionDirect(3,pos[3]);
        iCubEyes->setJointPositionDirect(4,pos[2]);
        if(useVergence) {
            iCubEyes->setJointPositionDirect(5,pos[4]);
        }
            
    }

    bool getActionParamsFromUser(int id) {
    
        bool ok = true;
        string sen, eff, cont;

        compositeMode = false;

        if(id == 0 ) {
            cout << "set higher priority controller..." << endl;
        } else if(id == 1 ) {
            cout << "set lower priority controller..." << endl;
        } else {
            cout << "invalid priority controller..." << endl;
            return false;
        }

        // get controller
        cout << "Controller [ TRACK | SACCADE ]: ";
        cin >> cont;

        if(cont=="TRACK") {
            controlMode=SMOOTH_PURSUIT;
        } else if(cont=="SACCADE") {
            controlMode=SACCADE;
        } else {
            cout << "unknown control mode: " << cont << endl;
            ok = false;
            return ok;
        }

        if(id == 1) {
            if(controlMode==SACCADE) {
                cout << "cannot set SACCADE controller when setting multi-objective control law..." << endl;
                return false;
            }
        }

        // get sensor
        cout << "Sensor [ AVG | LEFT | RIGHT ]: ";
        cin >> sen;
        
        if(sen == "AVG") {
            sensor[id] = BLOB_AVG;
        } else if(sen == "LEFT") {
            sensor[id] = BLOB_LEFT;
        } else if(sen == "RIGHT") {
            sensor[id] = BLOB_RIGHT;
        } else {
            cout << "unknown sensor: " << sen << endl;
            ok = false;
            return ok;
        }

        // get effector
        cout << "Effector [ HEAD | EYES ]: ";
        cin >> eff;
                
        if(eff == "HEAD") {                
            effector[id] = HEAD;
        } else if(eff == "EYES") {
            effector[id] = EYES;
        } else {
            cout << "unknown effector: " << eff << endl;
            ok = false;
            return ok;
        }

        if(id == 1) {
            if( (sensor[0]==sensor[1]) && (effector[0]==effector[1]) ) {
                cout << "all controllers are the same, no need to set multi-objective control law..." << endl;
                return true;
            } else {
                compositeMode = true;
            }
        }
        return ok;
       
    }

    void doUI() {
        cout << endl;
        cout << "GAZE CONTROL COORDINATOR" << endl;
        cout << "========================" << endl;
        cout << "set      -- sets the current controller parameters" << endl;
        cout << "run      -- runs the current controller" << endl;
        cout << "clear    -- clears the current control law" << endl;
        cout << "stop     -- stops the currenct controller (if running)" << endl;
        cout << "observe  -- observes the current blob and configuration data" << endl;
        cout << "learn    -- performs learning experiments to learn a gaze control policy" << endl;
        cout << "head     -- sets the head joint angles" << endl;
        cout << "help" << endl;
        cout << "quit" << endl << endl;
        //        cout << "> "; 
    }

    bool configure(ResourceFinder &rf)  {
        
        string iCubDir(getenv("ICUB_ROOT"));
        
        if(rf.check("robot")) {
            robotPrefix=rf.find("robot").asString().c_str();
        } 
        if(rf.check("verge")) {
            useVergence=(bool)(rf.find("verge").asInt());
        }
        foveaSize=rf.check("fovea",Value(10)).asInt();       
        learnPolicyMode=rf.check("policy");       
        foveaHeadingWindow=fabs(atan2((double)foveaSize*meters_per_pixel,focal_length));
        cout << "got fovea size: " << foveaSize << ", window: " << foveaHeadingWindow << endl;
                        
        handlerPort.open("/gazeControlCoordinator/rpc:i");
        attach(handlerPort);
        attachTerminal();

        string headOut = robotPrefix + "/head/state:o";
        string headIn = "/gazeControlCoordinator/head:i";
        headPort.open(headIn.c_str());
        Network::connect(headOut.c_str(),headIn.c_str());
        startResources();

        // set up the actions
        createActions();

        if(learnPolicyMode) {
            policyLearner = new GazeControlPolicyLearner(actions, actionPairs, 10);
            //policyLearner = new GazeControlPolicyLearner(actions, 10);
        }

        cout << "Learn Policy Mode: " << (int)learnPolicyMode << endl;
        Time::delay(1);
        doUI();

        return true;
    }
    
    bool respond(const Bottle &command, Bottle &reply)  {
        
        bool ok = true;
        bool rec = false; // is the command recognized?

        int numEpisodes;
        int idx;

        mutex.wait();

        switch (command.get(0).asVocab()) {
        case COMMAND_VOCAB_HELP:
            rec = true;
            doUI();
            break;
        case COMMAND_VOCAB_HEAD:
            rec = true;
            getHeadPositionFromUser();
            break;
        case COMMAND_VOCAB_SET:
            rec = true;
            if(!compositeMode) {
                if(controlMode == SMOOTH_PURSUIT) {
                    idx = 1; 
                } else if(controlMode == SACCADE) {
                    cout << "can't set an additional control objective with a SACCADE" << endl;
                    break;
                } else {
                    idx = 0;
                }
            } else {
                cout << "must clear control law before setting a new one...." << endl;
                break;
            }
            while(!getActionParamsFromUser(idx)) { }
            break;
        case COMMAND_VOCAB_RUN:
            rec = true;
            actionRunning = checkAction();
            if(actionRunning) {
                cout << "action already running..." << endl;
            } else if(policyLearnerRunning) {
                cout << "policy learner is running, can't run new action..." << endl;
            } else {
                ok = runAction();
            }
            ok = true;
            break;
        case COMMAND_VOCAB_STOP:
            rec = true;
            if(actionRunning) {
                stopAction();
            } 
            if(policyLearnerRunning) {
                policyLearner->stopLearning();
                policyLearnerRunning = false;
            }
            ok = true;
            break;                
        case COMMAND_VOCAB_CLEAR:
            clearControlLaw();
            rec = true;
            break;
        case COMMAND_VOCAB_OBSERVE:
            rec = true;
            observeData();
            break;
        case COMMAND_VOCAB_LEARN:
            rec = true;

            if(actionRunning) {
                cout << "action running, can't start policy learner..." << endl;
            } else if(learnPolicyMode) {

                policyLearnerRunning = policyLearner->isRunning();

                if(policyLearnerRunning) {
                    cout << "already running policy learner..." << endl;
                } else {
                    // set the number of episodes
                    cout << "number of learning episodes: ";
                    cin >> numEpisodes;
                    policyLearner->setEpisodes(numEpisodes);
                    
                    // start the policyLearner thread
                    policyLearner->startLearning();       
                    policyLearnerRunning = true;
                }
            } else {
                cout << "not in policy learning mode..." << endl;
            }
            break;
        }
        mutex.post();
        

        if (!rec)
            ok = RFModule::respond(command,reply);
        /*        
        if (!ok) {
            reply.clear();
            reply.addVocab(COMMAND_VOCAB_FAILED);
        }
        else
        */
        reply.addVocab(COMMAND_VOCAB_OK);
        
        return ok;
    }
    
    bool close() {
        handlerPort.close();
        headPort.close();
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
    

   
