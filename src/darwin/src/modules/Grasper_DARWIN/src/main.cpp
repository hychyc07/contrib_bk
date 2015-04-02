#include "darwin/Grasp.h"
#include <time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Event.h>
#include <yarp/os/Bottle.h>

using namespace std;

using namespace yarp::os;
using namespace yarp::math;

using namespace Darwin::grasper;


/* codice errore:
-2 : errore di sintassi
-3 : encoders non letti
-4 : errore di connessione
-5 : primitiva non riconosciuta
-6 : errore nello specificare il side
-7 : errore nel grasping: comando non provoca movimento
-8 : errore nell'attivare/disattivare touch
*/

class GraspingModule: public RFModule
{
public:
	Grasper *grasp;
	Port rpcPort;
	string moduleName;
	Bottle primitives;
	bool graspSucceeded;
private:
	Event isGrasping;
	string activeHand;

public:
	bool configure(ResourceFinder &rf)
	{
                moduleName = rf.find("moduleName").asString().c_str();
                string inputPortName = rf.find("inputPortName").asString().c_str();
                string outputPortName = rf.find("outputPortName").asString().c_str();
		primitives = rf.findGroup("primitives");

		grasp = new Grasper(("/"+moduleName+"/"+outputPortName));
		if(!rpcPort.open(("/"+moduleName+"/"+inputPortName).c_str()))
		{
			cout << "Error: unable to open rpc port, closing module" << endl;
			return false;
		}

		// set grasp speeds
		Bottle *speeds =NULL;
		if (!rf.check("speeds") || (rf.find("speeds").asList())->size()!= 9)
		{
			for (int i=0; i<9; i++)	
				speeds->addDouble(0.0);
		}
		else
			speeds = rf.find("speeds").asList();
		grasp->setGraspSpeeds(*speeds);
		Bottle *offset=NULL;
		// set tightening offset
		if (!rf.check("offset") || (rf.find("offset").asList())->size()!= 9)
		{
			for (int i=0; i<9; i++)	
				offset->addDouble(0.0);
		}
		else
			offset = rf.find("offset").asList();
		grasp->setOffset(*offset);

		graspSucceeded = false;
		cout << "--> module successfully configured" << endl;
		bool done = grasp->StartPorts();
		if(done)
			attach(rpcPort);
		
		return done;
	}

	double getPeriod()
	{
		return 0.25;
	}
	
	bool updateModule()
	{		
		isGrasping.wait();
		if(isStopping()) return true;

		bool end = false;
		while (!end)
		{
			Time::delay(0.25);
			end = grasp->hasFinished();
			cout << "end" << std::boolalpha << end << endl;
		}
		cout << "out of while" << endl;
		graspSucceeded = grasp->graspSucceeded();
		cout << "succeeded " << std::boolalpha << graspSucceeded << endl;
		//if (graspSucceeded)
			Bottle result = grasp->Tighten(activeHand);
		//else
		//isGrasping.reset();		
		
		return true;
	}

	bool interruptModule()
	{
		grasp->InterruptPorts();
		isGrasping.signal();
		return true;
	}

	bool close()
	{
		grasp->ClosePorts();
		delete grasp;
		return true;
	}

	bool respond(const Bottle &command, Bottle &reply)
	{
		reply.clear();

		if (command.get(0).asString() == "checkGraspSucceeded")
		{
			if (graspSucceeded)		reply.append("true");
			else					reply.append("false");

			//graspSucceeded = false;
		}
		else if (command.get(0).asString() == "grasp")
		{
			Bottle answer;

			if(command.size() == 4)
			{
				if(command.get(1).asString() != "safe")
				{
					reply.addInt(-2);
					reply.addString("Error: wrong keyword");
					return false;
				}

				graspSucceeded = false;
				activeHand = command.get(3).asString().c_str();
				answer = grasp->Grasp(primitives,command.get(2).asString().c_str(),activeHand, true);
				if (answer.get(0).asInt() == 0)		isGrasping.signal();

			}
			else if (command.size() == 3)
			{
				activeHand = command.get(2).asString().c_str();
				answer = grasp->Grasp(primitives,command.get(1).asString().c_str(),activeHand, false);
			}
			else
			{
				reply.addInt(-2);
				reply.addString("Error: wrong syntax");
				return false;
			}
			
			// check if grasp returned an error
			cout << answer.toString() << endl;
			reply.append(answer);
		}

		else if (command.get(0).asString() == "release")
		{
			if(command.size() != 2)
			{
				reply.addInt(-2);
				reply.addString("Error: wrong syntax");
				return false;
			}

			activeHand = command.get(1).asString().c_str();
			Bottle answer = grasp->Release(primitives,activeHand);
			reply.append(answer);
		}

		else if (command.get(0).asString() == "tighten")
		{	
			if(command.size() == 1)
				reply = grasp->Tighten(activeHand);
			
			else if(command.size() == 2)
			{
				activeHand = command.get(1).asString().c_str();
				Bottle answer = grasp->Tighten(activeHand);
				reply.append(answer);
			}
			else
			{
				reply.addInt(-2);
				reply.addString("Error: wrong syntax");
				return false;
			}				
		}

		else if (command.get(0).asString() == "offset")
		{
			if (command.size() == 2 && command.get(1).asString() == "get")
				reply = grasp->getOffset();

			else if (command.size() == 3 && command.get(1).asString() == "set")
			{			
				Bottle * list = command.get(2).asList();
				if(list->size() != 9)
				{
					reply.addInt(-2);
					reply.addString("Error: wrong argument number");
					return false;
				}
				grasp->setOffset(*list);
				reply.addInt(0);
			}
			else
			{
				reply.addInt(-2);
				reply.addString("Error: wrong syntax");
				return false;
			}
		}

		else if (command.get(0).asString() == "speed")
		{
			if (command.size() == 2 && command.get(1).asString() == "get")
				reply = grasp->getGraspSpeeds();

			else if (command.size() == 3 && command.get(1).asString() == "set")
			{			
				Bottle * list = command.get(2).asList();
				if(list->size() != 9)
				{
					reply.addInt(-2);
					reply.addString("Error: wrong argument number");
					return false;
				}
				grasp->setGraspSpeeds(*list);
				reply.addInt(0);
			}
			else
			{
				reply.addInt(-2);
				reply.addString("Error: wrong syntax");
				return false;
			}
				
		}
		else if (command.get(0).asString() == "quit")
		{
			reply.append("quit");
			return true;
		}
		else
		{
			reply.addInt(-2);
			return false;
		}

		return true;
	}
};

int main()
{
	Network::init();

	GraspingModule G;
	ResourceFinder rf;

	rf.setVerbose(true); // print to a console conf info
	rf.setDefaultConfigFile("GrasperConfiguration.ini");
	rf.setDefaultContext("Grasper_DARWIN/conf"); // dove sono i file .ini
	rf.configure("ICUB_ROOT",0,NULL);

	if(!G.configure(rf))
	{
		fprintf(stderr, "Error configuring module returning\n");
		return -1;
	}
	
	G.runModule();

	Network::fini();

	return 0;
}
