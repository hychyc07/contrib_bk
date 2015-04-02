// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include "cartesianImpedance.h"

#include <iostream>
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>

using namespace std;
using namespace yarp::os;

class MyModule:public RFModule
{
	cartesianImpedance imp;
public:

	double getPeriod()
	{
		return 0.02; //module periodicity (seconds)
	}

	/*
	 * This is our main function. Will be called periodically every getPeriod() seconds.
	 */
	bool updateModule()
	{
//		        cout << " updateModule... "<<endl;
//		double t1 = Time::now();
		if(imp.send_cmd)
			imp.loop();
		else
			fflush(stdout);

//		double t2 = Time::now();
//		printf("Time: %lf\n",t2-t1);
		return true;
	}

	/*
	 * Message handler. Just echo all received messages.
	 */
	bool respond(const Bottle& command, Bottle& reply)
	{

		fflush(stdout);
		if (command.get(0).asString()=="quit")
			return false;
		else if(command.get(0).asString()=="start")
		{
			imp.send_cmd = true;
			reply.addString("[OK]");
			fflush(stdout);
			return true;
		}
		else if(command.get(0).asString()=="stop")
		{
			imp.send_cmd = false;
			reply.addString("[OK]");
			fflush(stdout);
			return true;
		}
		else if(command.get(0).asString()=="set")
		{
			if(command.get(1).asString()=="stiff")
			{
				if(command.get(2).asString() == "val")
				{
					if(command.size() != 6)
					{
						reply.addString("Need 3 arguments to val");
						fflush(stdout);
						return false;
					}
					imp.stiff_1 = command.get(3).asDouble();
					imp.stiff_2 = command.get(4).asDouble();
					imp.stiff_3 = command.get(5).asDouble();

					reply.addString("[OK]");
					fflush(stdout);
					return true;
				}
				else if(command.get(2).asString() == "dir")
				{
					if(command.size() != 12)
					{
						reply.addString("Need 9 arguments to dir");
						fflush(stdout);
						return false;
					}
					imp.eigvec_mat(0,0) = command.get(3).asDouble();
					imp.eigvec_mat(1,0) = command.get(4).asDouble();
					imp.eigvec_mat(2,0) = command.get(5).asDouble();

					imp.eigvec_mat(0,1) = command.get(6).asDouble();
					imp.eigvec_mat(1,1) = command.get(7).asDouble();
					imp.eigvec_mat(2,1) = command.get(8).asDouble();

					imp.eigvec_mat(0,2) = command.get(9).asDouble();
					imp.eigvec_mat(1,2) = command.get(10).asDouble();
					imp.eigvec_mat(2,2) = command.get(11).asDouble();

					int status = imp.checkOrtho();
					if(status == 0)
					{
						reply.addString("[OK]");
						fflush(stdout);
						return true;
					}
					else if(status == 1)
					{
						reply.addString("Directions orthogonalized.");
						fflush(stdout);
						return true;
					}
					else if(status ==2)
					{
						reply.addString("Directions not orthogonal...not set.");
						fflush(stdout);
						return true;
					}
				}
				else
				{
					reply.addString("Sub command not understood");
					fflush(stdout);
					return false;
				}
				fflush(stdout);
				return true;
			}
		}
		else if(command.get(0).asString()=="inc")
		{
			if(command.get(1).asString()=="stiff")
			{
				imp.stiff_1 += command.get(2).asDouble();
				imp.stiff_2 += command.get(3).asDouble();
				imp.stiff_3 += command.get(4).asDouble();

				reply.addString("[OK]");
				return true;

			}
			else
			{
				reply.addString("Sub command not understood");
				fflush(stdout);
				return true;
			}
		}
		else if(command.get(0).asString()=="get")
		{
			if(command.get(1).asString()=="stiff")
			{
				if(command.get(2).asString() == "val")
				{
				reply.addDouble(imp.stiff_1);
				reply.addDouble(imp.stiff_2);
				reply.addDouble(imp.stiff_3);
				fflush(stdout);
				return true;
				}
				else if(command.get(2).asString() == "dir")
				{
					reply.addString(imp.eigvec_mat.toString().c_str());
					fflush(stdout);
					return true;
				}
				else
				{
					reply.addString("Sub command not understood");
					fflush(stdout);
					return true;
				}
			}
		}
		else if(command.get(0).asString() == "verbose")
		{
			if(command.get(1).asString() == "on")
				imp.verbose = true;
			else if(command.get(1).asString() == "off")
				imp.verbose = false;
			else
				reply.addString("Sub-command not understood");

			fflush(stdout);
			return true;

		}
		else if(command.get(0).asString() == "traj")
		{
			if(command.get(1).asString() == "on")
			{
				imp.traj = true;

				imp.encs->getEncoders(imp.home_deg.data());
				imp.home_rad = M_PI/180*imp.home_deg;

				imp.init_cart_pos = (imp.limb->EndEffPose(imp.home_rad)).subVector(0,5);

				reply.addString("[OK]");
			}
			else if(command.get(1).asString() == "off")
			{
				imp.traj = false;
				reply.addString("[OK]");
			}
			else
			{
				reply.addString("Sub-command not understood");
			}
			fflush(stdout);
			return true;
		}
		else
		{
			reply.addString("Command not understood");
			fflush(stdout);
			return true;
		}

	}

	/*
	 * Configure function. Receive a previously initialized
	 * resource finder object. Use it to configure your module.
	 * Open port and attach it to message handler.
	 */
	bool configure(yarp::os::ResourceFinder &rf)
	{

		imp.controlled_part = rf.find("part").asString();
		imp.robotname = rf.find("robot").asString();

		if(imp.controlled_part == "right_arm")
		{
			imp.limb = new iCubArm("right");
			imp.gcomp_port = GCOMP_PORT_RIGHT_ARM;
		}
		else if(imp.controlled_part == "left_arm")
		{
			imp.limb = new iCubArm("left");
			imp.gcomp_port = GCOMP_PORT_LEFT_ARM;
		}
		else
		{
			fprintf(stderr, "part not recognised\n");
			return false;
		}

		if (!imp.open())
		{
			fprintf(stderr, "Error opening detector\n");
			return false;
		}
		fflush(stdout);
		return true;
	}

	/*
	 * Interrupt function.
	 */
	bool interruptModule()
	{
		cout<<"Interrupting your module, for port cleanup"<<endl;
		close();
		return true;
	}

	/*
	 * Close function, to perform cleanup.
	 */
	bool close()
	{
		imp.close();
		detachTerminal();
		return true;
	}
};

int main(int argc, char * argv[])
{
	Network yarp;

	MyModule module;
	ResourceFinder rf;
	rf.configure("ICUB_ROOT", argc, argv);
	rf.setVerbose(true);

	cout<<"Configure module..."<<endl;
	fflush(stdout);
	if(!module.configure(rf))
	{
		printf("Error configuring module\n");
		return 1;
	}
	printf("done.\n");
	fflush(stdout);
	module.attachTerminal();
	fflush(stdout);
	module.runModule();
	cout<<"Main returning..."<<endl;
	return 0;
}
