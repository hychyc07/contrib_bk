// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Vishwanathan Mohan
  * email: Vishwanathan.Mohan@iit.it
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

#include "OPCModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool OPCModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/OPC"), 
                           "module name (string)").asString();
    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();
    robotPortName         = "/" + robotName + "/head";

    inputPortName           = rf.check("inputPortName",
			                Value(":i"),
                            "Input port name (string)").asString();
    
    
    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    handlerPortName =  "";
    handlerPortName += getName();         // use getName() rather than a literal 

    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    attach(handlerPort);                  // attach to port
    if (rf.check("config")) {
        configFile=rf.findFile(rf.find("config").asString().c_str());
        if (configFile=="") {
            return false;
        }
    }
    else {
        configFile.clear();
    }
 
	/** find model and color map
modelFile=rf.findFile(rf.find("model").asString().c_str());
	if (modelFile=="") {
            return false;
        }
    


	colormapFile=rf.findFile(rf.find("colormap").asString().c_str());
	if (colormapFile=="") {
            return false;
        }
*/
	/* create the thread and pass pointers to the module parameters */
    rThread = new OPCThread(robotName, configFile);

//    rThread->setColorPath(colormapFile);
//    rThread->setModelPath(modelFile);


	//=======================================================================
//    rThread->setName(getName().c_str());
    //rThread->setInputPortName(inputPortName.c_str());
    
    /* now start the thread to do the work */
    rThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool OPCModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool OPCModule::close() {
    handlerPort.close();
    /* stop the thread */
    printf("stopping the thread \n");
    rThread->stop();
    return true;
}

bool OPCModule::respond(const Bottle& command, Bottle& reply) 
{
    string helpMessage =  string(getName().c_str()) + 
                " commands are: \n" +  
                "help \n" +
                "quit \n";
    reply.clear(); 

   
	if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
	
// all this will feature in V1.1    
/*	switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            //reply.addString("many");    // what used to work
            reply.addString("help");
            reply.addString("commands are:");
            reply.addString(" help  : to get help");
            reply.addString(" quit  : to quit the module");
            reply.addString(" ");
            reply.addString(" ");
            reply.addString(" sus   chr : to suspend chrom thread");
            reply.addString(" sus   edg : to suspend edges thread");
            reply.addString(" res   chr : to resume chrom thread");
            reply.addString(" res   chr : to resume edges thread");
            reply.addString(" ");
            reply.addString(" ");
            reply.addString(" set w   hor <float> : to change the weightage of horizontal orientation");
            reply.addString(" set w   o45 <float> : to change the weightage of 45 deg orientation");
            reply.addString(" set w   ver <float> : to change the weightage of vertical orientation");
            reply.addString(" set w   oM45 <float> : to change the weightage of -45 deg orientation");
            reply.addString("    ");
            reply.addString(" get w   hor  : to get the weightage of horizontal orientation");
            reply.addString(" get w   o45  : to get the weightage of 45 deg orientation");
            reply.addString(" get w   ver  : to get the weightage of vertical orientation");
            reply.addString(" get w   oM45 : to get the weightage of -45 deg orientation");
            reply.addString(" get int      : to get the intensity value in fovea");
            reply.addString(" get ori 0    : to get the orientation the saliency of feature in fovea ");
            reply.addString(" get ori 45   : to get the orientation the saliency of feature in fovea ");
            reply.addString(" get ori 90   : to get the orientation the saliency of feature in fovea ");
            reply.addString(" get ori M45  : to get the orientation the saliency of feature in fovea ");
            reply.addString(" get chr      : get the value in thechrominance feature map in fovea ");
            reply.addString(" ");
            reply.addString(" ");
            //reply.addString(helpMessage.c_str());
            ok = true;
        }
        break;
    case COMMAND_VOCAB_QUIT:
        rec = true;
        {
            reply.addString("quitting");
            ok = false;
        }
        break;
    case COMMAND_VOCAB_SET:
        {
            switch(command.get(1).asVocab()){
            case COMMAND_VOCAB_WEIGHT:
                {
                    switch(command.get(2).asVocab()){
                    case COMMAND_VOCAB_HOR:
                        evThread->chromeThread->setWeightForOrientation(0,command.get(3).asDouble());
                        reply.addString("changed weight for horizontal orientation");
                        rec = true;
                        ok = true;
                        break;
                    case COMMAND_VOCAB_45:
                        evThread->chromeThread->setWeightForOrientation(1,command.get(3).asDouble());
                        reply.addString("changed weight for 45 deg orientation");
                        rec = true;
                        ok = true;
                        break;
                    case COMMAND_VOCAB_VER:
                        evThread->chromeThread->setWeightForOrientation(2,command.get(3).asDouble());
                        reply.addString("changed weight for vertical orientation");
                        rec = true;
                        ok = true;
                        break;
                    case COMMAND_VOCAB_M45:
                        evThread->chromeThread->setWeightForOrientation(3,command.get(3).asDouble());
                        reply.addString("changed weight for -45 deg orientation");
                        rec = true;
                        ok = true;
                        break;
                    case COMMAND_VOCAB_BRIGHT:
                        evThread->chromeThread->setBrightness((float)(command.get(3).asDouble()));
                        reply.addString("changed brightness for overall image");
                        rec = true;
                        ok = true;
                        break;
                    default:
                        rec = false;
                        ok  = false;
                    
                    }

                }
            break;
            }
        }
        break;

    case COMMAND_VOCAB_GET:
        {
            switch(command.get(1).asVocab()){
            case COMMAND_VOCAB_WEIGHT:
                {
                    switch(command.get(2).asVocab()){
                    case COMMAND_VOCAB_HOR:
                        wt = evThread->chromeThread->getWeightForOrientation(0);
                        reply.clear();
                        reply.addVocab(COMMAND_VOCAB_HOR); // ?? Needed
                        reply.addDouble(wt);
                        rec = true;
                        ok = true;
                        break;
                    case COMMAND_VOCAB_45:
                        wt = evThread->chromeThread->getWeightForOrientation(1);
                        reply.clear();
                        reply.addVocab(COMMAND_VOCAB_HOR); // ?? Needed
                        reply.addDouble(wt);
                        rec = true;
                        ok = true;
                        break;
                    case COMMAND_VOCAB_VER:
                        wt = evThread->chromeThread->getWeightForOrientation(2);
                        reply.clear();
                        reply.addVocab(COMMAND_VOCAB_HOR); // ?? Needed
                        reply.addDouble(wt);
                        rec = true;
                        ok = true;
                        break;
                    case COMMAND_VOCAB_M45:
                        wt = evThread->chromeThread->getWeightForOrientation(3);
                        reply.clear();
                        reply.addVocab(COMMAND_VOCAB_HOR); // ?? Needed
                        reply.addDouble(wt);
                        rec = true;
                        ok = true;
                        break;
                    case COMMAND_VOCAB_BRIGHT:
                        brightness = evThread->chromeThread->getBrightness();
                        reply.clear();
                        reply.addVocab(COMMAND_VOCAB_BRIGHT); // ?? Needed
                        reply.addDouble(brightness);
                        rec = true;
                        ok = true;
                        break;                                    
                    default:
                        rec = false;
                        ok  = false;
                    
                    }

                }
            break;
            case COMMAND_VOCAB_ORI:
                {
                    switch(command.get(2).asVocab()){
                    case COMMAND_VOCAB_P45:
                        wt = evThread->chromeThread->getFoveaOri(0);
                        reply.clear();
                        reply.addInt(wt);
                        rec = true;
                        ok = true;
                        break;
                    case COMMAND_VOCAB_N45:
                        wt = evThread->chromeThread->getFoveaOri(-45);
                        reply.clear();
                        reply.addInt(wt);
                        rec = true;
                        ok = true;
                        break;
                    case COMMAND_VOCAB_P0:
                        wt = evThread->chromeThread->getFoveaOri(0);
                        reply.clear();
                        reply.addInt(wt);
                        rec = true;
                        ok = true;
                        break;
                    case COMMAND_VOCAB_P90:
                        wt = evThread->chromeThread->getFoveaOri(90);
                        reply.clear();
                        reply.addInt(wt);
                        rec = true;
                        ok = true;
                        break;                                 
                    default:
                        rec = false;
                        ok  = false;                    
                    }

                }
            break;
            }
        }
        break;

    case COMMAND_VOCAB_SUSPEND:
        {
            switch(command.get(1).asVocab()){
            case COMMAND_VOCAB_CHROME_THREAD:
                rThread->suspend();
                reply.addString("suspending chrome thread");
                rec = true;
                ok = true;
                break;
            case COMMAND_VOCAB_EDGES_THREAD:
                evThread->edThread->suspend();
                reply.addString("suspending edges thread");
                rec = true;
                ok = true;
                break;
            default:
                rec = false;
                ok = false;
                break;
            }
        }
        break;
    case COMMAND_VOCAB_RESUME:
        {
            switch(command.get(1).asVocab()){
            case COMMAND_VOCAB_CHROME_THREAD:
                evThread->chromeThread->resume();
                reply.addString("resuming chrome thread");
                rec = true;
                ok = true;
                break;
            case COMMAND_VOCAB_EDGES_THREAD:
                evThread->edThread->resume();
                reply.addString("resuming edges thread");
                rec = true;
                ok = true;
                break;
            default:
                rec = false;
                ok = false;
                break;
            }
        }
        break;
    default:
        rec = false;
        ok  = false;
    }    

    respondLock.post(); 

    if (!rec){
        ok = RFModule::respond(command,reply);
    }
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);

    return ok;  */

    return true;
}

/* Called periodically every getPeriod() seconds */
bool OPCModule::updateModule()
{
    return true;
}

double OPCModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 1;
}

