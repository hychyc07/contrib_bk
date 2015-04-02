// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Ajaz Ahmad Bhat
  * email: ajaz.bhat@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file emPlotterRatethread.cpp
 * @brief Implementation of the eventDriven thread (see emPlotterRatethread.h).
 */

#include <emPlotterRatethread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 100 //ms

emPlotterRatethread::emPlotterRatethread():RateThread(THRATE) {
    robot = "icub";        
}

emPlotterRatethread::emPlotterRatethread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
}

emPlotterRatethread::~emPlotterRatethread() {
    // do nothing
}

bool emPlotterRatethread::threadInit() {

    //printf("emPlotterRatethread::threadInit \n");
    idle = false;    

    if (!inputPortcue.open(getName("/cue:i").c_str())) {
        cout << ": unable to open port to receive partial cue bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    
    if (!inputPortrem.open(getName("/rememberedX:i").c_str())) {
        cout << ": unable to open port to receive remembered bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    

    if (!inputPorthub.open(getName("/hub:i").c_str())) {
        cout << ": unable to open port to receive hub bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }  
    
    if (!inputPortplan.open(getName("/plan:i").c_str())) {
        cout << ": unable to open port to receive plan bottles "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    //printf("ports opened\n");
    
    /*if (!Network::connect("/MyRemembered:o","/emPlotter/rememberedX:i"))
        return false;    
   //printf("Connection to rememberedX");
    if (!Network::connect("/Useful/PastXperiences:o","/emPlotter/cue:i"))
        return false;
    //if (!Network::connect("/Hub:o","/emPlotter/hub:i"))
    //   return false;
    */
    //Network::connect("/PlanXplore:o","/emPlotter/plan:i");
        //printf("connection NOT successful\n");
       
       
           
    return true;
    

}

void emPlotterRatethread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string emPlotterRatethread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void emPlotterRatethread::setInputPortName(string InpPort) {
        
}

void emPlotterRatethread::setSharingBottle(Bottle *partialBot[], Bottle *remBot[], Bottle *hubBot[], Bottle* top, Bottle *bottom, Bottle *a, Bottle *b) {

    for (int i = 0; i < 5; i++) {
        pbot[i] = partialBot[i];
        rbot[i] = remBot[i];
        hbot[i] = hubBot[i];
    }
    hubTop          =   top;
    hubBottomAll    =   bottom;
    planA           =   a;
    planB           =   b;
}

void emPlotterRatethread::setSemaphore(Semaphore *pmu[], Semaphore *rmu[], Semaphore *hmu[], Semaphore *c, Semaphore *d, Semaphore *a, Semaphore *b) {
    
    for (int i = 0; i < 5; i++) {
        pmutex[i] = pmu[i];
        rmutex[i] = rmu[i];
        hmutex[i] = hmu[i];
    }
    mutexTop        =   c;
    mutexBottomAll  =   d;
    mutexA          =   a;
    mutexB          =   b;     
        
}

void emPlotterRatethread::run() {    

    cueIncoming     = inputPortcue.read(false);
    remIncoming     = inputPortrem.read(false);
    hubIncoming     = inputPorthub.read(false);
    planIncoming    = inputPortplan.read(false);
    
    if (cueIncoming!=NULL){
        printf(" bottle %s \n",cueIncoming->toString().c_str()); 
        string name;
        name = cueIncoming->get(0).asString();
                   
        if(!strcmp(name.c_str(), "cue0")) {
            if (cueIncoming->size() > 0){
                pmutex[0]->wait();
                *pbot [0] = *cueIncoming->get(1).asList();
                pmutex[0]->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "cue1")) {
            printf("correctly detected the cue1 \n");    
            if (cueIncoming->size() > 0){
                pmutex[1]->wait();
                *pbot [1] = *cueIncoming->get(1).asList();
                printf("bot1 %08x got secondary list %s \n",pbot[1], pbot[1]->toString().c_str());
                pmutex[1]->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "cue2")) {
            if (cueIncoming->size() > 0){
                pmutex[2]->wait();
                *pbot [2] = *cueIncoming->get(1).asList();
                pmutex[2]->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "cue3")) {
            if (cueIncoming->size() > 0){
                pmutex[3]->wait();
                *pbot [3] = *cueIncoming->get(1).asList();
                pmutex[3]->post();
            }                   
        }
                
        else if(!strcmp(name.c_str(), "cue4")) {
            if (cueIncoming->size() > 0){
                pmutex[4]->wait();
                *pbot [4] = *cueIncoming->get(1).asList();
                pmutex[4]->post();
            }                 
        }        
        else 
            printf("Please provide a partial cue with a name like \"cue0\"   \"cue1\"   \"cue2\"   \"cue3\"    \"cue4\" \n");          
    }

          
    if (remIncoming!=NULL){           
        printf(" \n remembered bottle %s \n",remIncoming->toString().c_str());               
        string name;
        name = remIncoming->get(0).asString();      
        if(!strcmp(name.c_str(), "rem0")) {
            if (remIncoming->size() > 0){
                rmutex[0]->wait();
                *rbot [0] = *remIncoming->get(1).asList();
                rmutex[0]->post();
            }               
        }
                
        else if(!strcmp(name.c_str(), "rem1")) {
            if (remIncoming->size() > 0){
                rmutex[1]->wait();
                *rbot [1] = *remIncoming->get(1).asList();
                rmutex[1]->post();
            }                    
        }
        
        else if(!strcmp(name.c_str(), "rem2")) {
            if (remIncoming->size() > 0){
                rmutex[2]->wait();
                *rbot [2] = *remIncoming->get(1).asList();
                rmutex[2]->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "rem3")) {
            if (remIncoming->size() > 0){
                rmutex[3]->wait();
                *rbot [3] = *remIncoming->get(1).asList();
                rmutex[3]->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "rem4")) {
            if (remIncoming->size() > 0){
                rmutex[4]->wait();
                *rbot [4] = *remIncoming->get(1).asList();
                rmutex[4]->post();
            }                    
        }
                               
        else 
            printf("Please provide a remembered experience with a name like \"rem0\"   \"rem1\"   \"rem2\"   \"rem3\"    \"rem4\" \n");
    }
    
    
    
    if (planIncoming!=NULL){
        printf(" bottle %s \n",planIncoming->toString().c_str()); 
        string name;
        name = planIncoming->get(0).asString();
                   
       if(!strcmp(name.c_str(), "plan1")) {
            if (planIncoming->size() > 0){
                mutexA->wait();
                *planA = *planIncoming->get(1).asList();
                mutexA->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "plan2")) {
            if (planIncoming->size() > 0){
                mutexB->wait();
                *planB = *planIncoming->get(1).asList();
                mutexB->post();
            }                   
        }                
               
        else 
            printf("Please provide a  plan with a name like   \"planA\"   \"planB\" \n");
        }  
        
        
        
        
        
        
        
        if (hubIncoming!=NULL){           
        printf(" hub bottle %s \n",hubIncoming->toString().c_str());               
        string name;
        name = hubIncoming->get(0).asString();      
        if(!strcmp(name.c_str(), "hub0")) {
            if (hubIncoming->size() > 0){
                hmutex[0]->wait();
                *hbot [0] = *hubIncoming->get(1).asList();
                hmutex[0]->post();
            }               
        }
                
        else if(!strcmp(name.c_str(), "hub1")) {
            if (hubIncoming->size() > 0){
                hmutex[1]->wait();
                *hbot [1] = *hubIncoming->get(1).asList();
                hmutex[1]->post();
            }                    
        }
        
        else if(!strcmp(name.c_str(), "hub2")) {
            if (hubIncoming->size() > 0){
                hmutex[2]->wait();
                *hbot [2] = *hubIncoming->get(1).asList();
                hmutex[2]->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "hub3")) {
            if (hubIncoming->size() > 0){
                hmutex[3]->wait();
                *hbot [3] = *hubIncoming->get(1).asList();
                hmutex[3]->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "hub4")) {
            if (hubIncoming->size() > 0){
                hmutex[4]->wait();
                *hbot [4] = *hubIncoming->get(1).asList();
                hmutex[4]->post();
            }                    
        }
        
        else if(!strcmp(name.c_str(), "hubTop")) {
            if (hubIncoming->size() > 0){
                mutexTop->wait();
                *hubTop = *hubIncoming->get(1).asList();
                mutexTop->post();
            }                    
        }
                
        else if(!strcmp(name.c_str(), "hubBottomAll")) {
            printf("correctly detected the cue1 \n");    
            if (hubIncoming->size() > 0){
                mutexBottomAll->wait();
                *hubBottomAll = *hubIncoming->get(1).asList();
                printf("hubBottomAll %08x got secondary list %s \n",hubBottomAll, hubBottomAll->toString().c_str());
                mutexBottomAll->post();
            }                    
        }
                
                               
        else 
            printf("Please provide a hub with a name like \"hub0\"  \"hub1\"  \"hub2\"  \"hub3\"  \"hub4\"  \"hubTop\" \"hubBottom\" \n");
    }
        
                        
        //printf("Finishing the rate thread run \n");          
}

void emPlotterRatethread::threadRelease() {


    inputPortcue.interrupt();
    inputPortcue.close();
    inputPortrem.interrupt();
    inputPortrem.close();
    inputPorthub.interrupt();
    inputPorthub.close();
    inputPortplan.interrupt();
    inputPortplan.close();
    
}


