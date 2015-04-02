// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
*
\author Unknown

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/canBusSniffer/main.cpp.
**/

#include <stdio.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <C:/Software/iCub/main/src/libraries/iCubDev/include/iCub/DebugInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Network.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <bitset>
#include <vector>

#define USE_ESTIMATED_DATA 1
#define LOG_ENABLED        1
#define VALIDATION         1

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

using namespace std;

const int SNIFFER_THREAD_RATE=5; //ms

const int CAN_DRIVER_BUFFER_SIZE=2047;
const int localBufferSize=2048;
bool done=false;

class SnifferThread: public RateThread
{
    PolyDriver ECANdriver;
	PolyDriver* control_board_driver;
	PolyDriver* debug_driver;
    ICanBus *iCanBus;
    ICanBufferFactory *iBufferFactory;
    CanBuffer messageBuffer;
	unsigned long int cnt;    
	FILE *fp0;
	FILE *fp1;
	FILE *fp2;
	FILE *fp3;
	IDebugInterface   *iDbg;
	IEncoders         *iEnc;
	IPositionControl  *iPos;
	IPidControl		  *iPid;
	/* dimension of local buffer, number of recieved messages */
    unsigned int messages, readMessages;

	/* variables to be sniffed from the Can Bus */
	signed int position[2];
	signed short speed[2];
	signed short pwm[2];
	signed short pid[2];
	signed short dutyCycle[2];
	signed short torque[2];
	signed short commut[2];
	bool finished;

	int    ms_counter;
	int    stage_counter;
	int    command_number;
	double joint_command[4];
	double joint_angle[4];
	double joint_error[4];
    int _param_a10_coeff;
	int _param_a11_coeff;
	int _param_a20_coeff;
	int _param_a21_coeff;
	int _param_a22_coeff;
	static const int ms_max = 1000;
	struct params
	{
	 int a10;
	 int a11;
	 int a20;
	 int a21;
	 int a22;
	};
	std::vector <params> params_vector;
	std::vector <Pid>    pid_vector;

public:

	void arm_goto (double p1, double p2, double p3, double p4, double speed=70)
	{
		fprintf(stdout,"sending new command\n");
		joint_command[0]=p1;
		joint_command[1]=p2;
		joint_command[2]=p3;
		joint_command[3]=p4;
		iPos->setRefSpeed(0,speed);
		iPos->setRefSpeed(1,speed);
		iPos->setRefSpeed(2,speed);
		iPos->setRefSpeed(3,speed);
		iPos->positionMove(0,p1);
		iPos->positionMove(1,p2);
		iPos->positionMove(2,p3);
		iPos->positionMove(3,p4);
	}

	void print_out(FILE* out_file)
	{
		int i=0;
		for (i=0; i<3; i++)
			fprintf(out_file,"%+7.2f ",joint_command[i]);
		fprintf(out_file,"     ");
		for (i=0; i<3; i++)
			fprintf(out_file,"%+7.2f ",joint_angle[i]);
		fprintf(out_file,"     ");
		for (i=0; i<3; i++)
			fprintf(out_file,"%+7.2f ",joint_error[i]);
		fprintf(out_file,"    ");
		fprintf(out_file," %d ",_param_a10_coeff);	
		fprintf(out_file," %d ",_param_a11_coeff);
		fprintf(out_file," %d ",_param_a20_coeff);
		fprintf(out_file," %d ",_param_a21_coeff);
		fprintf(out_file," %d ",_param_a22_coeff);
		fprintf(out_file,"\n   ");
	}

	void generate_parameter_vector2()
	{
		_param_a10_coeff = -1645;
		_param_a11_coeff =  1645;
		_param_a20_coeff = -1645;
		_param_a21_coeff =  1645;
		_param_a22_coeff =  1645;
		int p1=0;
		int p2=0;
		int p3=1500;
		
		for (p1=100; p1<1000; p1+=100)					
		{
			params t;
			t.a10=-p1;
			t.a11= 2*p1;
			t.a20=-p3; //p1
			t.a21= p3; //p2
			t.a22= p3;
			params_vector.push_back(t);
		}
	}

	void generate_parameter_vector()
	{
		_param_a10_coeff = -1645;
		_param_a11_coeff =  1645;
		_param_a20_coeff = -1645;
		_param_a21_coeff =  1645;
		_param_a22_coeff =  1645;
		int p1=0;
		int p2=0;
		int p3=0;
		
		for (p1=750; p1<2001; p1+=250)
		{
			for (p2=750; p2<2001; p2+=250)
			{
					p3=1500;
					//for (int p3=1000; p3<2001; p3+=500)
					{
						params t;
						t.a10=-p1;
						t.a11= p2;
						t.a20=-p3; //p1
						t.a21= p3; //p2
						t.a22= p3;
						params_vector.push_back(t);
					}
			}
		}
	}

    SnifferThread(int r=SNIFFER_THREAD_RATE): RateThread(r)
    {
		messages = localBufferSize;
		stage_counter=0;
		ms_counter=0;
		command_number=0;
		finished = false;
		
		//generate_parameter_vector();
		generate_parameter_vector2();

		for(int i=0; i<2; i++)
		{
			position[i] = 0;
			speed[i] = 0;
			pwm[i] = 0;
			pid[i] = 0;
			dutyCycle[i] = 0;
			torque[i] = 0;
			commut[i] = 0;
			joint_command[i] = 0;
		}

		for(int i=0; i<4; i++)
		{
			joint_command[i]=0;
			joint_angle[i]=0;
			joint_error[i]=0;
		}
	}

    bool threadInit()
    {
		Property control_board_options("(device remote_controlboard)");
        control_board_options.put("remote","/icubV2/right_arm");
        control_board_options.put("local","/armV2Sniffer");
		control_board_driver=new PolyDriver;
        if (!control_board_driver->open(control_board_options))
        {
			fprintf(stderr,"ERROR: cannot open control board driver...\n");
            delete control_board_driver;    
            return false;
        }
		control_board_driver->view(iEnc);
		control_board_driver->view(iPos);
		control_board_driver->view(iPid);

		Pid o_pid;
		//iPid->getPid(0,&o_pid);
		o_pid.kp=32000;
		o_pid.kd=50;
		o_pid.ki=60;
		o_pid.max_int=1333;
		o_pid.scale=11;
		o_pid.max_output=1333;
		o_pid.offset=0;

		/*for (int pp=0; pp<28; pp++)
		{
			
			//o_pid.kd=50+pp*1000; //max value =28500
			//pid_vector.push_back(o_pid);
			o_pid.kd=pp*2; //max value =28500
			pid_vector.push_back(o_pid);

		}*/
		for (int pp=0; pp<28; pp++)
		{
			o_pid.ki=pp*8; //max value =28500
			pid_vector.push_back(o_pid);
		}
		Property debug_options("(device debugInterfaceClient)");
        debug_options.put("remote","/icubV2/right_arm");
        debug_options.put("local","/armV2DebugClient");
		debug_driver=new PolyDriver;
        if (!debug_driver->open(debug_options))
        {
			fprintf(stderr,"ERROR: cannot open debug driver...\n");
            delete debug_driver;    
            return false;
        }
		debug_driver->view(iDbg);
		
		// load configuration parameters into the options property collector
        Property prop;

		// set driver properties
/*		prop.put("device", "ecan");

        prop.put("CanTxTimeout", 500);
        prop.put("CanRxTimeout", 500);
        prop.put("CanDeviceNum", 0);

		prop.put("CanTxQueue", CAN_DRIVER_BUFFER_SIZE);
        prop.put("CanRxQueue", CAN_DRIVER_BUFFER_SIZE);

		// open driver
        ECANdriver.open(prop);

        if (!ECANdriver.isValid())
        {
            fprintf(stderr, "Error opening PolyDriver check parameters\n");
            return false;
        }

        ECANdriver.view(iCanBus);
        ECANdriver.view(iBufferFactory);

		// set the baud rate (0 is defaul for 1Mb/s) 
		if (!iCanBus->canSetBaudRate(0))
			fprintf(stderr, "Error setting baud rate\n");

		// add the id of can messages to be read
		iCanBus->canIdAdd(0x3CA);
		iCanBus->canIdAdd(0x3CB);
        messageBuffer=iBufferFactory->createBuffer(localBufferSize);
*/

		cnt = 0;
		fp0 = fopen("j0_log.txt","w");
		fp1 = fopen("j1_log.txt","w");
		fp2 = fopen("j2_log.txt","w");
		fp3 = fopen("j3_log.txt","w");

		_param_a10_coeff = -1645;
		_param_a11_coeff =  1645;
		_param_a20_coeff = -1645;
		_param_a21_coeff =  1645;
		_param_a22_coeff =  1645;

		for (int j=0; j<4; j++)
		{
			iDbg->setDebugParameter(j,10,_param_a10_coeff);
			iDbg->setDebugParameter(j,11,_param_a11_coeff);
			iDbg->setDebugParameter(j,12,_param_a20_coeff);
			iDbg->setDebugParameter(j,13,_param_a21_coeff);
			iDbg->setDebugParameter(j,14,_param_a22_coeff);
		}

		const int speed=20;
		arm_goto( 0,  20, 0,80,speed);
		Time::delay(5);
		return true;
    }

	void apply_param()
	{
		if (params_vector.empty() == true)
		{
			finished =true;
			return;
		}
		else
		{
			params t = params_vector.back();
			params_vector.pop_back();
			for (int j=0; j<4; j++)
			{
				_param_a10_coeff = t.a10;
				_param_a11_coeff = t.a11;
				_param_a20_coeff = t.a20;
				_param_a21_coeff = t.a21;
				_param_a22_coeff = t.a22;

				iDbg->setDebugParameter(j,10,_param_a10_coeff);
				iDbg->setDebugParameter(j,11,_param_a11_coeff);
				iDbg->setDebugParameter(j,12,_param_a20_coeff);
				iDbg->setDebugParameter(j,13,_param_a21_coeff);
				iDbg->setDebugParameter(j,14,_param_a22_coeff);
			}
		}
	}

	void apply_param2()
	{
		if (pid_vector.empty() == true)
		{
			finished =true;
			return;
		}
		else
		{
			Pid t = pid_vector.back();
			pid_vector.pop_back();
			iPid->setPid(0,t);
		}
	}

	void test1()
	{
		/*
		//initial values
		_param_a10_coeff = -1645;
		_param_a11_coeff =  1645;
		_param_a20_coeff = -1645;
		_param_a21_coeff =  1645;
		_param_a22_coeff =  1645;
		*/

		if (_param_a10_coeff<=-400)
		{
			_param_a10_coeff += 50;
			_param_a20_coeff += 50;
		}
		else
		{
			finished=true;
		}
	}

	void test2()
	{
		/*
		//initial values
		_param_a10_coeff = -1645;
		_param_a11_coeff =  1645;
		_param_a20_coeff = -1645;
		_param_a21_coeff =  1645;
		_param_a22_coeff =  1645;
		*/

		if (_param_a11_coeff>=400 && _param_a11_coeff<5000)
		{
			_param_a11_coeff += 100;
			_param_a21_coeff += 100;
		}
		else
		{
			finished=true;
		}
	}

    void run()
    {
		int i=0;

		iEnc->getEncoder(0,&joint_angle[0]);
		iEnc->getEncoder(1,&joint_angle[1]);
		iEnc->getEncoder(2,&joint_angle[2]);
		//iEnc->getEncoder(3,&joint_angle[3]);
		iPid->getError(0,&joint_error[0]);
		iPid->getError(1,&joint_error[1]);
		iPid->getError(2,&joint_error[2]);
		//iPid->getError(3,&joint_error[3]);
       
		// read from the Can Bus messages with the id that has been specified
		/*readMessages = 0; 
		bool res=iCanBus->canRead(messageBuffer, messages, &readMessages);
		
		// parse the messages 
		for(int i=0; i<readMessages; i++)
		{
			if (messageBuffer[i].getId() == 0x3Ca)
			{
				unsigned_gaugeData[0] = (messageBuffer[i].getData()[1]<<8)|messageBuffer[i].getData()[0];
				unsigned_gaugeData[1] = (messageBuffer[i].getData()[3]<<8)|messageBuffer[i].getData()[2];
				unsigned_gaugeData[2] = (messageBuffer[i].getData()[5]<<8)|messageBuffer[i].getData()[4];
			}
			if (messageBuffer[i].getId() == 0x3Cb)
			{
				unsigned_gaugeData[3] = (messageBuffer[i].getData()[1]<<8)|messageBuffer[i].getData()[0];
				unsigned_gaugeData[4] = (messageBuffer[i].getData()[3]<<8)|messageBuffer[i].getData()[2];
				unsigned_gaugeData[5] = (messageBuffer[i].getData()[5]<<8)|messageBuffer[i].getData()[4];
			}
		}
		*/
	
		static int file_out=0;
		{
			static double time_old_wd=Time::now();
			double time_wd=Time::now();
			fprintf(stdout,"ms:%d st:%d t:%+3.3f |", ms_counter, stage_counter, time_wd-time_old_wd);
			print_out(stdout);
			time_old_wd=time_wd;
			if      (file_out==0) {print_out(fp0);}
			else if (file_out==1) {print_out(fp1);}
			else if (file_out==2) {print_out(fp2);}
			else if (file_out==3) {print_out(fp3);}
	
		}
		
		if (ms_counter==0)
		{
			/*
			// move joint 2 only
			if      (command_number==0) {arm_goto( -30,  30, -30,60); command_number=1;}
			else if (command_number==1) {arm_goto( -30,  30,  30,60); command_number=0;}
			*/
			// move joint 0 only
			const int speed = 70;
			if      (command_number==0) {arm_goto( -80,  20, 0,80,speed); command_number=1; file_out=0; apply_param();}
			else if (command_number==1) {arm_goto( -80, 100, 0,80,speed); command_number=2; file_out=1;}
			else if (command_number==2) {arm_goto(   0, 100, 0,80,speed); command_number=3; file_out=2;}
			else if (command_number==3) {arm_goto(   0,  20, 0,80,speed); command_number=0; file_out=3;}
/*
			if      (command_number==0) {arm_goto( -80,  20, 0,80,speed); command_number=1; file_out=0; apply_param2();}
			else if (command_number==1) {arm_goto(   0,  20, 0,80,speed); command_number=2; file_out=1;}
			else if (command_number==2) {arm_goto( -80,  20, 0,80,speed); command_number=3; file_out=2; apply_param2();}
			else if (command_number==3) {arm_goto(   0,  20, 0,80,speed); command_number=0; file_out=3;}
*/
			//if (stage_counter==8) finished=true;

			for (int j=0; j<4; j++)
			{
				iDbg->setDebugParameter(j,10,_param_a10_coeff);
				iDbg->setDebugParameter(j,11,_param_a11_coeff);
				iDbg->setDebugParameter(j,12,_param_a20_coeff);
				iDbg->setDebugParameter(j,13,_param_a21_coeff);
				iDbg->setDebugParameter(j,14,_param_a22_coeff);
			}		

			stage_counter++;
		}
		if (ms_counter>=ms_max)
		{
			ms_counter=0;
		}
		else
		{
			ms_counter++;
		}
		
		if (finished ==true)
		{
			fclose(fp0);
			fclose(fp1);
			fclose(fp2);
			fclose(fp3);
			this->stop();
		}
	}

    void threadRelease()
    {
        iBufferFactory->destroyBuffer(messageBuffer);
        ECANdriver.close();
		fclose(fp0);
		fclose(fp1);
		fclose(fp2);
		fclose(fp3);
    }
};

int main(int argc, char *argv[]) 
{
	yarp::os::Network yarp;

    if (!yarp.checkNetwork())
	{
		fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
	}

	YARP_REGISTER_DEVICES(icubmod)

    SnifferThread thread;
    thread.start();

    std::string input;
    while(!done)
    {
/*        std::cin>>input;
        if (input=="quit")
            done=true;*/
    }

    thread.stop();
}