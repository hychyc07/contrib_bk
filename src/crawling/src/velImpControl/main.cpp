// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/Os.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include "velImpControlThread.h"
#include "yarp/os/Module.h"
#include <string.h>
#include <string>
#include <math.h>
#include <yarp/os/RFModule.h>

#include <iostream>

/*default rate for the control loop*/
const int CONTROL_RATE=20;

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

class VelControlModule : public yarp::os::RFModule {
//CTmodified: class VelControlModule: public Module {
private:
    PolyDriver driver;
    velImpControlThread *vc;
    char partName[255];
    char robotName[255];
  
    Port input_port;

public:
  
    virtual bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
    {
        fprintf(stderr,"receiving command from port\n");
        int index = 0;
        int cmdSize = command.size();
    
        while(cmdSize>0)
            {
                switch(command.get(index).asVocab())  {
                case  VOCAB4('s','u','s','p'):
                    {
                        reply.addVocab(Vocab::encode("ack"));
                        vc->halt();
                        cmdSize--;
                        index++;
                        break;
                    }
                case VOCAB3('r','u','n'):
                    {
                        reply.addVocab(Vocab::encode("ack"));
                        vc->go();
                        cmdSize--;
                        index++;
                        break;
                    }
                case VOCAB3('s','e','t'):
                    {
                        if (command.size()>=3)
                            {
                                int i=command.get(index+1).asInt();
                                double pos=command.get(index+2).asDouble();
                                vc->setRef(i, pos);
                                index +=3;
                                cmdSize-=3;
                            }
                        else
                            {
                                cmdSize--;
                                index++;
                                fprintf(stderr, "Invalid set message, ignoring\n");
                            }
                        reply.addVocab(Vocab::encode("ack"));
                        break;
                    }
                case VOCAB4('s','v','e','l'):
                    {
                        if(command.size()>=3)
                            {
                                int i=command.get(index+1).asInt();
                                double vel = command.get(index+2).asDouble();
                                vc->setVel(i,vel);
                                index += 3;
                                cmdSize-=3;;
                                reply.addVocab(Vocab::encode("ack"));
                            }
                        else
                            {
                                cmdSize--;
                                index++;
                                fprintf(stderr,"Invalid set vel message, ignoring\n");
                                reply.addVocab(Vocab::encode("fail"));
                            }
                        break;
                    }
                case VOCAB4('g','a','i','n'):
                    {
                        if(command.size()>=3)
                            {
                                int i=command.get(index+1).asInt();
                                double gain = command.get(index+2).asDouble();
                                vc->setGain(i,gain);
                                index+=3;
                                cmdSize-=3;
                                reply.addVocab(Vocab::encode("ack"));
                            }
                        else
                            {
                                cmdSize--;
                                index++;
                                fprintf(stderr,"Invalid set gain message, ignoring\n");
                                reply.addVocab(Vocab::encode("fail"));
                            }
                        break;
                    }
                default:
                    {
                        cmdSize--;
                        index++;
                        return RFModule::respond(command, reply); // call default
                    }
                }
                return true;
            }

        return false;
    }
    //CTmodified: virtual bool open(yarp::os::Searchable &s)
	virtual bool configure(yarp::os::ResourceFinder &rf) 
    {
        //CTmodified: Property options(s.toString());
		Property options(rf.toString());
        Property stiffnessOptions;

        char robotName[255];
        Time::turboBoost();    
        options.put("device", "remote_controlboard");
        if(options.check("robot"))
            strncpy(robotName, options.find("robot").asString().c_str(),sizeof(robotName));
        else
            strncpy(robotName, "icub", sizeof(robotName));
	
        if(options.check("part"))
            {
                char tmp[255];
                sprintf(tmp, "/%s/vc/%s/client",
                        robotName,
                        options.find("part").asString().c_str());
                options.put("local",tmp);
	    
                sprintf(tmp, "/%s/%s", 
                        robotName,
                        options.find("part").asString().c_str());
                options.put("remote", tmp);
	    
                sprintf(tmp,"/%s/vc/%s/input",
                        robotName,
                        options.find("part").asString().c_str());
                input_port.open(tmp);
            
                options.put("carrier", "mcast");
            
				attach(input_port); 
           }
        else
            {
                fprintf(stderr, "Please specify part (e.g. --part head)\n");
                return false;
            }
            
            
        ////end of the modif////////////
    
        if (!driver.open(options))
            {
                fprintf(stderr, "Error opening device, check parameters\n");
                return false;
            }

        ///we start the thread
        int period = CONTROL_RATE;
        if(options.check("period"))
            period = options.find("period").asInt();
        
        printf("control rate is %d ms",period);

        if (!options.check("part"))
            return false;
        
        sprintf(partName, "%s", options.find("part").asString().c_str());

        vc=new velImpControlThread(period);
        
		if(options.check("file"))
		{
			stiffnessOptions.fromConfigFile(options.find("file").asString().c_str());
			
		}
		else
		{
			const char *cubPath;
			cubPath = yarp::os::getenv("ICUB_ROOT"); //previously set to ICUB_DIR
			if(cubPath == NULL) 
			{
				printf("velImpControl::init>> ERROR getting the environment variable ICUB_DIR, exiting\n");
				return false;
			}
			string cubPathStr(cubPath);
			//stiffnessOptions.fromConfigFile((cubPathStr + "/app/crawlingApplication/conf/" + partName + "StiffnessConfig.ini").c_str());
			//CTModified: stiffnessOptions.fromConfigFile((cubPathStr + "/contrib/src/crawlingTest/app/conf/" + partName + "StiffnessConfig.ini").c_str());
			stiffnessOptions.fromConfigFile((rf.getContextPath().c_str() + (std::string) "/" + partName + "StiffnessConfig.ini").c_str());
		}

		bool impedance_enabled = true;
		if(stiffnessOptions.check("impedance_disabled"))
		{
			impedance_enabled=false;
			printf("Disabling impedance commands\n");
		}

		if(stiffnessOptions.check("njoints"))
		{
			vc->njoints = stiffnessOptions.find("njoints").asInt();
			printf("\nControlling %d dofs\n", vc->njoints);
		}
		else
		{
			printf("Please specify the number of joints in the config file");
			return false;
		}
		
		vc->impContr.resize(vc->njoints);
		vc->swingStiff.resize(vc->njoints);
		vc->stanceStiff.resize(vc->njoints);
		vc->requestedStiff.resize(vc->njoints);
		vc->requestedDamp.resize(vc->njoints);
		vc->currStiff.resize(vc->njoints);
		vc->currDamp.resize(vc->njoints);
		vc->swingDamp.resize(vc->njoints);
		vc->stanceDamp.resize(vc->njoints);
		
		if(stiffnessOptions.check("ImpJoints"))
		{
			printf("Joints controlled with impedance: ");
			
			Bottle& bot = stiffnessOptions.findGroup("ImpJoints");
			for(int i=0; i<vc->njoints; i++)
			{
				vc->impContr[i] = bot.get(i+1).asDouble();
				printf("%4.2f ", vc->impContr[i]);
			}
			printf("\n");
		}
		else
		{
			printf("Please specify which joints are controlled with impedance in the config file");
			return false;
		}
		
		if(stiffnessOptions.check("SwingStiff"))
		{
			printf("Stiffness swing : ");
			Bottle& bot = stiffnessOptions.findGroup("SwingStiff");
			for(int i=0; i<vc->njoints; i++)
			{
				vc->swingStiff[i] =  bot.get(i+1).asDouble();
				printf("%4.2f ", vc->swingStiff[i]);
			}
			printf("\n");
		}
		else
		{
			printf("Please specify the stiffness for the swing in the config file\n");
			return false;
		}
		
		if(stiffnessOptions.check("StanceStiff"))
		{
			printf("Stiffness stance: ");
			Bottle& bot = stiffnessOptions.findGroup("StanceStiff");
			for(int i=0; i<vc->njoints; i++)
			{
				vc->stanceStiff[i] =  bot.get(i+1).asDouble();
				printf("%4.2f ", vc->stanceStiff[i]);
			}
			printf("\n");
		}
		else
		{
			printf("Please specify the stiffness for the stance in the config file\n");
			return false;
		}
		
		if(stiffnessOptions.check("SwingDamp"))
		{
			printf("Damping swing ");
			Bottle& bot = stiffnessOptions.findGroup("SwingDamp");
			for(int i = 0; i < vc->njoints; i++)
			{
				vc->swingDamp[i] =  bot.get(i+1).asDouble();
				printf("%4.2f ", vc->swingDamp[i]);
			}
			printf("\n");
		}
		else
		{
			printf("Please specify the damping for the swing in the config file\n");
			return false;
		}
		
		if(stiffnessOptions.check("StanceDamp"))
		{
			printf("Damping stance: ");
			Bottle& bot = stiffnessOptions.findGroup("StanceDamp");
			for(int i=0; i<vc->njoints; i++)
			{
				vc->stanceDamp[i] = bot.get(i+1).asDouble();
				printf("%4.2f ", vc->stanceDamp[i]);
			}
			printf("\n");
		}
		else
		{
			printf("Please specify the Dampness for the stance in the config file\n");
			return false;
		}
		
        vc->init(&driver, partName, robotName, impedance_enabled);

        vc->start();
        return true;
    }

    virtual bool close()
    {
        fprintf(stderr, "Closing module [%s]\n", partName);
        vc->stop();
        vc->halt();
        delete vc;
        fprintf(stderr, "Thead [%s] stopped\n", partName);

        driver.close();
        input_port.close();

        fprintf(stderr, "Module [%s] closed\n", partName);

        return true;
    }

    virtual double getPeriod()
    { return 5.0; } // module period

    virtual bool updateModule()
    {
        return true;
    }
};

int main(int argc, char *argv[])
{
    Network yarp;
    VelControlModule mod;
    /* CT: prepare and configure the RF*/
	ResourceFinder rf;
	rf.setVerbose(true);
	//CTAdded:
	rf.setDefaultContext("crawlingApplication/conf");
	rf.configure("ICUB_ROOT", argc, argv);
    return mod.runModule(rf);
}
