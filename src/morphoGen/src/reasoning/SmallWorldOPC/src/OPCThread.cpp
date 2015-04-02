
#include <OPCThread.h>
#include <cstring>
#include <string>
#include<time.h>
#include <math.h>
#include <iostream>
#include <fstream>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

OPCThread::OPCThread() {
    robot = "icub";        
}

OPCThread::OPCThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

OPCThread::~OPCThread() {
    // do nothing
}

bool OPCThread::threadInit() {
   	   
		if (!OPCServer.open(getName("/OPCServer:io").c_str())) {
			cout << ": unable to open port to send unmasked events "  << endl;
			return false;  // unable to open; let RFModule know so that it won't run
		}  

    	if (!WorldSnap.open(getName("/input/snapshot:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }  

		//OPC server listens to the observer and posts snapshots when triggered
   
	 return true;
}

void OPCThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string OPCThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void OPCThread::setInputPortName(string InpPort) {
    
}

void OPCThread::run()
{   
	 while (isStopping() != true) {

        if (OPCServer.getInputCount()) {        
			Bottle OPCReq, OPCResp;
         
			OPCServer.read(OPCReq,true);
			
			if(!OPCReq.isNull()) {
				printf("%s \n",OPCReq.toString().c_str());
				cout<<"OPCReq!=NULL"<<endl;
				// request present
				//reading
				int cmd = OPCReq.get(0).asVocab();
				cout<< "Received microgoal FIND from Client:Observer" <<cmd<<endl;
				Network::connect("/colordata:o", "/input/snapshot:i"); 
                //===========================================================================
                for(int i=0;i<10;i++)
					{
						for(int j=0;j<18;j++)
							{
							  Eighteens[i][j]=0;
							}
					  }
				 for(int i=0;i<10;i++)
					{
						IOD[i]=0;
					}
				 NumObject=0;
				 int ctrr=0;
				 //===========================================================================
					if (WorldSnap.getInputCount())
							{            
							   Bottle* ObjIdd = WorldSnap.read(true);
							   NumObject = (int) ObjIdd->get(ctrr).asDouble();
							   ctrr=ctrr+1;
							   for(int i=0;i<NumObject;i++)
									{
										IOD[i]=(int) ObjIdd->get(ctrr).asDouble();
                                        ctrr=ctrr+1; 
									}	 

								for(int i=0;i<NumObject;i++)
									{
       									for(int j=0;j<18;j++)
											{
												Eighteens[i][j]=ObjIdd->get(ctrr).asDouble();
												ctrr=ctrr+1; 
											}	
								     }
      						}

			 //============================================================================
			  cout << "Sending out Visual Snapshot to Client Observer" << endl;

     		  OPCResp.addInt(428);
			  OPCResp.addInt(NumObject);
				
								for(int i=0;i<NumObject;i++)
									{
									  OPCResp.addInt(IOD[i]);
									}
				 				for(int i=0;i<NumObject;i++)
									{
       									for(int j=0;j<18;j++)
											{
											   OPCResp.addDouble(Eighteens[i][j]);
											}	
								     }
				
				OPCServer.reply(OPCResp);
				Time::delay(3);
			}
			else {
				cout<<"null request"<<endl;
			}
//========================================================================================  
		}

		Time::delay(5);          	     
	} 
}

void OPCThread::threadRelease() {
    // nothing
	//EpimCtrlPort.close();
     
}


void OPCThread::onStop() {
    
//    outputPort.interrupt();
  //  outputPort.close();
}


