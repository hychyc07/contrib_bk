// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include "graspIn.h"
#include "string.h"

/**
 * instance two grasping objects, one for each arm. Open the yarp port '/grasp/doGrasp'
 * and listen for grasping commands. Possible commands are (left|right) (pregrasp|grasp).
 */
int main(int argc, char *argv[]) 
{
	string robot="icub";
	graspIn left_hand;
	graspIn right_hand;
	
	if(left_hand.Init(robot,0.3,0.3,eLEFT_HAND) && right_hand.Init(robot,0.3,0.3,eRIGHT_HAND))
	{
		BufferedPort<Bottle> port;
        port.open("/grasp/doGrasp");

		Vector init_pos;
		init_pos.resize(16);
		init_pos=0;
		left_hand.velocity_move(init_pos);
		init_pos=left_hand.get_velocity();
		init_pos=left_hand.get_encs();
		init_pos=left_hand.get_amps();
		init_pos=left_hand.get_pid();

		right_hand.velocity_move(init_pos);
		init_pos=right_hand.get_velocity();
		init_pos=right_hand.get_encs();
		init_pos=right_hand.get_amps();
		init_pos=right_hand.get_pid();				
		
		while (true) 
		{
            Bottle *input = port.read();
            
    		cout << "got " << input->toString().c_str() << endl;
    		left_hand.checkCurrents();
    		left_hand.checkPidErrors();
    		left_hand.checkCollisions();
    		
    		right_hand.checkCurrents();
    		right_hand.checkPidErrors();
    		right_hand.checkCollisions();

        	if (strcmp(input->toString().c_str(),"left pregrasp")==0) 
        	{
        		printf("left pregrasping\n");
        		left_hand.move_to_pregrasp_pos(); 
        		Bottle& output = port.prepare();
            	output.clear();
            	output.addInt(1);
            	output.addString("left hand moved to Pregrasp position");
            	port.write();
            	
            }
            else if (strcmp(input->toString().c_str(),"left grasp")==0) 
            {
            	
        		bool grasp_success=left_hand.doGrasp();
        		Bottle& output = port.prepare();
            	output.clear();
            	if (grasp_success)
            	{
            		output.addInt(1);
            		output.addString("left grasp successfull");
            	}
            	else
            	{
            		output.addInt(0);
            		output.addString("left grasp not successfull");
            	}
				port.write();
            }
            else if (strcmp(input->toString().c_str(),"right pregrasp")==0) 
        	{
        		printf("right pregrasping\n");
        		right_hand.move_to_pregrasp_pos(); 
        		Bottle& output = port.prepare();
            	output.clear();
            	output.addInt(1);
            	output.addString("right hand moved to Pregrasp position");
            	port.write();
            	printf("right hand moved to Pregrasp position\n");
            }
            else if (strcmp(input->toString().c_str(),"right grasp")==0) 
            {
            	
        		bool grasp_success=right_hand.doGrasp();
        		Bottle& output = port.prepare();
            	output.clear();
            	if (grasp_success)
            	{
            		output.addInt(1);
            		output.addString("right grasp successfull");
            	}
            	else
            	{
            		output.addInt(0);
            		output.addString("right grasp not successfull");
            	}
				port.write();
            }
             else if (strcmp(input->toString().c_str(),"chicken")==0) 
            {
            	
        		Bottle& output = port.prepare();
            	output.clear();
            	output.addString("Chicken chicken!");
            	
				port.write();
            }
            Time::delay(0.1);
        }
	
	}
    return 0;
}


