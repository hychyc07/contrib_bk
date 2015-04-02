#include <iostream>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include "Darwin/Demo1Module.h"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;

using namespace Darwin::Demo1;

int main()
{
	Network::init();

	// open an rpc interface to receive user's commands
	Port speaker;
	speaker.setRpcMode(true);
	int i=0;
	while(!speaker.open("/IITdemo/rpc"))
	{
		if(i>=4) {cout << "Error in creating rpc port" << endl; Network::fini(); return -1;}
		Time::delay(0.25);
		i++;
	}

	// open a face interface to dialog with expressions
	Port face;
	while(!face.open("/IITdemo/face:o"))
	{
		if(i>=4) {cout << "Error in creating face port" << endl; speaker.close(); Network::fini(); return -1;}
		Time::delay(0.25);
		i++;
	}

	// open a planner instance that executes user's commands
	Demo1Module * planner;
	ResourceFinder rf;
	rf.setVerbose(true); // print to a console conf info
	rf.setDefaultConfigFile("C:/DARWIN/ConXML/conf/Demo1Configuration.ini");
	rf.setDefaultContext("../conf"); // dove sono i file .ini
	rf.configure("ICUB_ROOT",0,NULL);

	planner = new Demo1Module(rf,20);
	if(!planner->threadInit())
	{
		fprintf(stderr, "Error configuring module returning\n");
		face.close();
		return -1;
	}

	if(!planner->start())
	{
		planner->threadRelease();
		fprintf(stderr, "Error configuring module returning\n");
		face.close();
		return -1;
	}

	// wait for the port to be connected

	//send a welcome message:

	Network::connect(face.getName().c_str(),"/icub/face/emotions/in");
	Bottle faceCmd,test,reply;
	test.clear();
	faceCmd.append("set mou sur");
	face.write(faceCmd,test);

	Bottle welcome;
	welcome.addString("--> Hello! Are you ready to play with me?");
		
	speaker.read(test,true);
	reply.append(welcome);
	speaker.reply(reply);

	test.clear();
	speaker.read(test,true);
	
	if (test.get(0).asString() == "no") 
	{
		faceCmd.clear();
		faceCmd.append("set mou sad");
		face.write(faceCmd,test);

		welcome.clear();
		welcome.addString("--> Bye");
		speaker.replyAndDrop(welcome);

		planner->stop();
		speaker.close();
		face.close();
		Network::fini(); 
		return 0;
	}
	
	faceCmd.clear();
	faceCmd.append("set all hap");
	face.write(faceCmd,test);

	// mettere icub in posizione relax
	planner->Relax();
	if(!planner->startPMP())
	{
		welcome.clear();
		welcome.addString("--> Did you connect my ports?");
		speaker.reply(welcome);
	}

	planner->suspend();
	welcome.clear();
	welcome.addString("--> What should I do?");
	speaker.reply(welcome);
	
	Bottle cmd;

	while(true)
	{
		reply.clear();
		speaker.read(cmd,true);
		if (!planner->arePortsConnected() && cmd.get(0).asString() != "quit")
		{
			reply.append("--> Have you connected all my ports?");
			speaker.reply(reply);
		}
		else
		{
			string first = cmd.get(0).asString().c_str();
		
			if (first == "stop")
			{
				if(planner->stopMotion()) reply.addString("--> OK, I'm freezed");
				else					  reply.addString("--> Sorry, I'm not freezed");
				speaker.reply(reply);
			}
			else if(first == "save")
			{
				int done;
				if(cmd.size() != 5 || !cmd.get(4).isString())
					done = -1;
				else
					done = planner->add2map(cmd.get(4).asString().c_str(),cmd.get(1).asDouble(),cmd.get(2).asDouble(),cmd.get(3).asDouble());
				switch(done)
				{ 
				case 1:		reply.addString("--> Ok, I got it! But it was too low, I have saved it higher");break;
				case 0:		reply.addString("--> Ok, now I know this object!"); break;
				case -1:	reply.addString("--> Sorry, I can't understand");break;
				case -2:	reply.addString("--> Hey, I already knew this object!"); break;
				}
				speaker.reply(reply);
			}

			else if(first == "userRecord")
			{
				int done;
				bool left = (cmd.get(1).asString() == "left");

				// record using user input to stop
				if (cmd.size() == 2)
				{
					if(!left)
						done = planner->userRecordStart(false);
					else
						done = planner->userRecordStart();
				}

				if (done == 0)
				{
					reply.addString("--> Ok, you can guide me!");
					speaker.reply(reply);
					Bottle stop;
					speaker.read(stop,true);

					if(!left)
						done = planner->userRecordStop(false);
					else
						done = planner->userRecordStop();
				}

				switch(done)
				{
				case 1: 
						reply.addString("--> Ok, I got it! But it was too low, I have saved it higher. How is this object called (label <name>)?");
						break;
				case 0: 
						reply.addString("--> Ok, I got it! How is this object called (label <name>)?");
						break;
				case -3: reply.addString("--> Sorry, I did not understand what you said");break;
				case -4: reply.addString("--> Sorry, I could not stop my motion");break;
				case -5: reply.addString("--> Sorry, I could not start recording");break;
				case -6: reply.addString("--> Sorry, I don't know where my arm is");break;
				}

			}
			else if(first == "record")
			{
				int done;

				// record using user input to stop
				if (cmd.size() == 2)
				{
					if(cmd.get(1).asString() == "right")
						done = planner->Record(false);
					else
						done = planner->Record();
				}
				else
					done = planner->Record();
				
				switch(done)
				{
				case 1: 
						// wait for the point to be recorded: if z<=SAFETY_Z, z = z+0.1
						// once it has been recorded ask for the object label
						planner->pointRecorded.wait();
						Time::delay(5);
						planner->Relax();
						reply.addString("--> Ok, I got it! But it was too low, I have saved it higher. How is this object called (label <name>)?");
						break;
				case 0: 
						// wait for the point to be recorded
						// once it has been recorded ask for the object label
						planner->pointRecorded.wait();
						Time::delay(5);
						planner->Relax();
						reply.addString("--> Ok, I got it! How is this object called (label <name>)?");
						break;
				case -3: reply.addString("--> Sorry, I did not understand what you said");break;
				case -4: reply.addString("--> Sorry, I could not stop my motion");break;
				case -5: reply.addString("--> Sorry, I could not start recording");break;
				case -6: reply.addString("--> Sorry, I don't know where my arm is");break;
				}
		
				speaker.reply(reply);
			}
			else if (first == "label")
			{
				if (cmd.size() != 2)
					reply.addString("--> Sorry, I did not understand the label");
				else
				{
					int done = planner->Label(cmd.get(1).asString().c_str());
					switch(done)
					{ 
					case 0:		reply.addString("--> Ok, now I know this object!"); break;
					case -1:	reply.addString("--> Sorry, I don't have any point to label"); break;
					case -2:	reply.addString("--> Hey, I already knew this object!"); break;
					}
				}
				speaker.reply(reply);
			}

			else if (first == "forget")
			{
				if (cmd.size() != 2)
					reply.addString("--> Sorry, I did not understand the label");
				else
				{
					int done = planner->Forget(cmd.get(1).asString().c_str());
					switch(done)
					{ 
					case 0:		reply.addString("--> Ok, object forgotten!"); break;
					case -1:	reply.addString("--> Sorry, I don't remember this object"); break;
					}
				}
				speaker.reply(reply);
			}

			else if (first == "recall")
			{
				reply = planner->Recall();
				if (reply.size() == 0)
					reply.addString("--> Sorry, tabula rasa");
				speaker.reply(reply);
			}

			else if (first == "relax")
			{
				int done = planner->Relax();
				switch(done)
					{ 
					case 0:		reply.addString("--> Ok, I relax!"); break;
					case -1:	reply.addString("--> Sorry, I can't relax"); break;
					}
				speaker.reply(reply);
			}

			else if (first == "prepare")
			{
				int done = planner->Prepare();
				switch(done)
					{ 
					case 0:		reply.addString("--> Ok, I'm ready!"); break;
					case -1:	reply.addString("--> Sorry, something went wrong"); break;
					}
				speaker.reply(reply);
			}
			// sequence reaching
			else if (first == "reach")
			{
				if(cmd.size() == 1)
					reply.addString("--> Sorry, I don't know what I should reach with my finger");
				else
				{
					int done = planner->SequenceReach(cmd.tail());
					switch(done)
					{
					case 0: reply.addString("--> Ok, I have reached the object!"); break;
					case -1: reply.addString("--> Sorry, I don't know this object"); break;
					case -2: reply.addString("--> Sorry, I don't know where my hands are"); break;
					case -3: reply.addString("--> Sorry, I did not understand what you said"); break;
					case -4: reply.addString("--> Sorry, my ports are not connected"); break;
					case -5: reply.addString("--> Sorry, I don't know this command"); break;
					case -8: reply.addString("--> Sorry, I couldn't reach the object"); break;
					case -9: reply.addString("--> Sorry, my wrist orientation is wrong"); break;
					}
				}
				speaker.reply(reply);
			}

			// finger reaching
			else if (first == "reach1")
			{
				if(cmd.size() != 3 && cmd.size() != 2)
					reply.addString("--> Sorry, I don't know what I should reach with my finger");
				else
				{
					
					int done = planner->Prepare();
					if(done != 0) reply.addString("--> Sorry, I'm lazy");
					else if (cmd.size() == 3)
					{
						if (cmd.get(2).asString() == "top" || cmd.get(2).asString() == "side")
						{
							done = planner->FingerReach(cmd.get(1).asString().c_str(),"",cmd.get(2).asString().c_str(),false);
							if (done == 0)
							{
								cout << "check reaching performance" << endl;
								done = planner->checkReachingSuccess(cmd.get(1).asString().c_str());
							}
						}
						else
							done = -9;
					}						
					else
					{
						done = planner->FingerReach(cmd.get(1).asString().c_str(),"","top",false);
						if (done == 0)
						{
							cout << "check reaching performance" << endl;
							done = planner->checkReachingSuccess(cmd.get(1).asString().c_str());
						}
					}

					switch(done)
					{
					case 0: 
						reply.addString("--> Ok, I have reached the object!");
						//done = planner->Lift(planner->activeChain);
						//if (done !=0 )
						//{reply.addString("But I could not lift my hand"); cout << "done: " << done << endl;break;}
						done = planner->goFar(planner->activeChain);
						if (done !=0 )
						{reply.addString("But I could not go farer"); cout << "done: " << done << endl;}
						planner->Relax();
						break;
					case -1: reply.addString("--> Sorry, I don't know this object"); break;
					case -2: reply.addString("--> Sorry, I don't know where my hands are"); break;
					case -3: reply.addString("--> Sorry, I did not understand what you said"); break;
					case -4: reply.addString("--> Sorry, my ports are not connected"); break;
					case -5: reply.addString("--> Sorry, I don't know this command"); break;
					case -8: reply.addString("--> Sorry, I couldn't reach the object"); break;
					case -9: reply.addString("--> Sorry, my wrist orientation is wrong"); break;
					}
				}

				speaker.reply(reply);
			}
			// reach to grasp function
			else if (first == "grasp")
			{
				if(cmd.size() != 2)
				{
					reply.addString("--> Sorry, I don't know what I should grasp");
				}
				else
				{
					cout << "prepare for grasping" << endl;
					// this function will return 1 if no reaching is needed, 0 otherwise
					int done = planner->prepare4Grasping(cmd.get(1).asString().c_str());
					if (done == 0)
					{
						// check that motion is finished successfully
						cout << "check reaching performance" << endl;
						done = planner->checkReachingSuccess(cmd.get(1).asString().c_str());
					}
					if(done == 0 || done == 1)
					{
						cout << "grasp" << endl;
						done = (planner->Grasp()).get(0).asInt();
					}
					if(done == 0)
					{
						done = planner->Lift(planner->activeChain,true);
						// check if contact is mantained
						if(done == 0)
						{
							done = planner->Release(planner->activeChain);
							if (done == 0)				// reinitialize
								done = planner->Relax();
						}
					}
					switch(done)
					{
					case 0:
					case 1:  reply.addString("--> Ok, I grasped the object!"); break;
					case -1: reply.addString("--> Sorry, I don't know this object"); break;
					case -2: reply.addString("--> Sorry, I don't know where my hands are"); break;
					case -3: reply.addString("--> Sorry, I did not understand what you said"); break;
					case -4: reply.addString("--> Sorry, my ports are not connected, I can't grasp"); break;
					case -5: reply.addString("--> Sorry, I don't know this command"); break;
					case -6: reply.addString("--> Sorry, I don't have this hand"); break;
					case -7: reply.addString("--> Sorry, I couldn't grasp the object"); break;
					case -8: reply.addString("--> Sorry, I couldn't reach the object"); break;
					case -9: reply.addString("--> Hey, I have lost the object!"); break;
					}
				}
				speaker.reply(reply);
			}

			else if (first == "release")
			{
				if(cmd.size() != 2)
					reply.addString("--> Sorry, I don't know what hand to open");
				else
				{
					int done = planner->Release(cmd.get(1).asString().c_str());
					switch(done)
						{ 
						case 0:		reply.addString("--> Ok, I released the object!"); break;
						case -1:	reply.addString("--> Sorry, I can't release the object"); break;
						}
					if (done == 0)	planner->Relax();//planner->initIcubUp();
				}
				speaker.reply(reply);
			}

			else if (first == "lift")
			{
				if(cmd.size() != 2)
					reply.addString("--> Sorry, I don't know what hand to lift");
				else
				{
					int done = planner->Lift(cmd.get(1).asString().c_str());
					switch(done)
					{		
					case 0:  reply.addString("--> Ok, I have lifted my hand!"); break;
					case -2: reply.addString("--> Sorry, I don't know where my hands are"); break;
					case -3: reply.addString("--> Sorry, I did not understand what you said"); break;
					case -4: reply.addString("--> Sorry, my ports are not connected"); break;
					case -5: reply.addString("--> Sorry, I don't know this command"); break;
					case -6: reply.addString("--> Sorry, I don't have this hand"); break;
					case -9: reply.addString("--> Hey, I have lost the object!"); break;
					}
					
				}

				speaker.reply(reply);
			}

			else if (first == "carry")
			{
				if(cmd.size() != 2)
				{
					reply.addString("--> Sorry, I don't know what I should grasp");
				}
				else
				{
					

				}
				speaker.reply(reply);
			}

			else if (first == "recycle")
			{
				if(cmd.size() != 2)
				{
					reply.addString("--> Sorry, I don't know what I should throw in the garbage");
				}
				else
				{
					int done = planner->Recycle(cmd.get(1).asString().c_str());

					switch(done)
					{		
					case 0:  reply.addString("--> Ok, I have recycled the object!"); break;
					case -1: reply.addString("--> Sorry, I don't know this object"); break;
					case 1 : reply.addString("--> Sorry, I don't know where the litter is (label = bin)"); break;
					case -2: reply.addString("--> Sorry, I don't know where my hands are"); break;
					case -3: reply.addString("--> Sorry, I did not understand what you said"); break;
					case -4: reply.addString("--> Sorry, my ports are not connected"); break;
					case -5: reply.addString("--> Sorry, I don't know this command"); break;
					case -6: reply.addString("--> Sorry, I don't have this hand"); break;
					case -7: reply.addString("--> Sorry, I couldn't grasp the object"); break;
					case -8: reply.addString("--> Sorry, I couldn't reach the object"); break;
					case -9: reply.addString("--> Sorry, my wrist orientation is wrong"); break;
					case -10: reply.addString("--> Hey, I have lost the object!"); break;
					}
					planner->Relax();
				}
				speaker.reply(reply);
			}
			else if (first == "geometry")
			{
				if(cmd.size() < 2)
					reply.addString("--> Sorry, I don't understand");

				if(cmd.get(1).asString() == "get")
					reply.append(planner->getObjectGeometry());
				else if (cmd.get(1).asString() == "set" && cmd.size() == 4)
				{
					planner->setObjectGeometry(cmd.get(2).asDouble(),cmd.get(3).asDouble());
					reply.addString("--> OK, now I have new geometry values");
				}
				else
					reply.addString("--> Sorry, I don't understand");

				speaker.reply(reply);
			}
			else if (first == "bimanual")
			{
				if(cmd.size() < 2)
					reply.addString("--> Sorry, I don't know what I should do with my hands");
				else
				{
					int done;
					if (cmd.get(1).asString() == "reach")
					{
						switch(cmd.size())
						{
						case 3: // use the geometrical info abt the object to reach it
							done = planner->initBimanualReach(cmd.get(2).asString().c_str(),cmd.get(2).asString().c_str());
							planner->BimanualRelease();
							break;
						case 4: // reach from top two different objects
							done = planner->initBimanualReach(cmd.get(2).asString().c_str(),cmd.get(3).asString().c_str());
							break;
						}
						// get the label of the object/s to reach: if one 
					}
					else if (cmd.get(1).asString() == "grasp" && cmd.size() == 3)
					{
						done = planner->initBimanualReach(cmd.get(2).asString().c_str(),cmd.get(2).asString().c_str());
						if (done == 0)
						{
							done = planner->checkBimanualReachingSuccess(cmd.get(2).asString().c_str(),cmd.get(2).asString().c_str());
							if (done == 0)
							{
								done = planner->BimanualGrasp();
								if (done == 0)
									done = planner->BimanualLift(false);
							}
						}

						Time::delay(1);
						planner->BimanualRelease();

					}
					else if(cmd.get(1).asString() == "carry")
					{
					}

					switch(done)
					{		
					case 0:  reply.addString("--> Ok, I have recycled the object!"); break;
					case -1: reply.addString("--> Sorry, I don't know this object"); break;
					case 1 : reply.addString("--> Sorry, I don't know where the litter is (label = bin)"); break;
					case -2: reply.addString("--> Sorry, I don't know where my hands are"); break;
					case -3: reply.addString("--> Sorry, I did not understand what you said"); break;
					case -4: reply.addString("--> Sorry, my ports are not connected"); break;
					case -5: reply.addString("--> Sorry, I don't know this command"); break;
					case -6: reply.addString("--> Sorry, I don't have this hand"); break;
					case -7: reply.addString("--> Sorry, I couldn't grasp the object"); break;
					case -8: reply.addString("--> Sorry, I couldn't reach the object"); break;
					case -9: reply.addString("--> Sorry, my wrist orientation is wrong"); break;
					case -10: reply.addString("--> Hey, I have lost the object!"); break;
					}
				}
				speaker.reply(reply);
			}

			else if (first == "quit")
			{
				planner->Relax();
				//if(planner->isSuspended())	planner->resume();
				planner->stop();
				reply.addString("--> Bye");
				speaker.replyAndDrop(reply);
				break;
			}
			else
			{
				reply.addString("--> I did not understand what you said");
				speaker.reply(reply);
			}
		}

		cmd.clear();
		reply.clear();
	}

	face.interrupt();
	face.close();
	speaker.close();
	Network::fini();

	return 0;
}
