// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
*
\author Marco Randazzo

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/canBusSniffer/main.cpp.
**/

#include <stdio.h>
#include <yarp/dev/ControlBoardInterfaces.h>
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

#define USE_ESTIMATED_DATA 1
#define LOG_ENABLED        1
#define VALIDATION         1

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

using namespace std;

#if LOG_ENABLED == 1
const int SNIFFER_THREAD_RATE=100; //ms
#else
const int SNIFFER_THREAD_RATE=100; //ms
#endif

const int CAN_DRIVER_BUFFER_SIZE=2047;
const int localBufferSize=2048;
bool done=false;


bool log_start=LOG_ENABLED;
class SnifferThread: public RateThread
{
    PolyDriver ECANdriver;
	PolyDriver* control_board_driver;
    ICanBus *iCanBus;
    ICanBufferFactory *iBufferFactory;
    CanBuffer messageBuffer;
	unsigned long int cnt;    
	FILE *fp;
	IEncoders         *ienc;
	IPositionControl  *ipos;
	/* dimension of local buffer, number of recieved messages */
    unsigned int messages, readMessages;

	/* variables to be sniffed from the Can Bus */
	signed int position[2];
	signed short speed[2];
	signed short pwm[2];
	signed short pid[2];
	signed short sin_frequency[2];
	signed short sin_amplitude[2];
	signed short dutyCycle[2];
	signed short torque[2];
	signed short commut[2];

	unsigned short unsigned_gaugeData[6];
	signed short signed_gaugeData[6];
	signed short dataBuffer[6];

	int    stage_counter;
	int    stage;
	double joint_angle[4];
	double joint_sensor_data[6];
	double joint_estimated_sensor_data[6];
	double FT_sensor_data[6];

public:

	void jnt_goto (double p1, double p2, double p3, double p4)
	{
		ipos->setRefSpeed(0,20);
		ipos->positionMove(0,p1);
	}

	void print_out(FILE* out_file)
	{
		int i=0;
		for (i=0; i<1; i++)
			fprintf(out_file,"%+7.2f ",joint_angle[i]);

		fprintf(out_file,"     ");
			fprintf(out_file,"%+7.0f ",joint_sensor_data[2]);
			fprintf(out_file,"%+7.0f ",joint_sensor_data[3]);
		fprintf(out_file,"     ");

		fprintf(out_file,"\n");
	}

    SnifferThread(int r=SNIFFER_THREAD_RATE): RateThread(r)
    {
		messages = localBufferSize;

		for(int i=0; i<6; i++)
		{
			unsigned_gaugeData[i] = 0;
			signed_gaugeData[i] = 0;
			dataBuffer[i] = 0;
			joint_sensor_data[i]=0;
			joint_estimated_sensor_data[i]=0;
			FT_sensor_data[i]=0;
		}

		for(int i=0; i<2; i++)
		{
			position[i] = 0;
			speed[i] = 0;
			pwm[i] = 0;
			pid[i] = 0;
			dutyCycle[i] = 0;
			torque[i] = 0;
			commut[i] = 0;
			sin_frequency[i] = 0;
			sin_amplitude[i] = 0;
		}
		stage_counter=0;
#if LOG_ENABLED == 1
		stage=0;
#else
		stage=1;
#endif 
	}

    bool threadInit()
    {
		Property control_board_options("(device remote_controlboard)");
        control_board_options.put("remote","/icub/left_leg");
        control_board_options.put("local","/legV2Sniffer");
		control_board_driver=new PolyDriver;
        if (!control_board_driver->open(control_board_options))
        {
			fprintf(stderr,"ERROR: cannot open control board driver...\n");
            delete control_board_driver;    
            return false;
        }
		control_board_driver->view(ienc);
		control_board_driver->view(ipos);

		// load configuration parameters into the options property collector
        Property prop;

		// set driver properties
		prop.put("device", "ecan");

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

		//send the start message 
		messageBuffer[0].setId(0x20C);
		messageBuffer[0].setLen(2);
		messageBuffer[0].getData()[0]=7;
		messageBuffer[0].getData()[1]=3;
		iCanBus->canWrite(messageBuffer,1,&readMessages);

		cnt = 0;
		fp = fopen("output.dat","w");
		return true;
    }

    void run()
    {
		int i=0;

		ienc->getEncoder(0,&joint_angle[0]);	
       
		// read from the Can Bus messages with the id that has been specified
		readMessages = 0; 
		bool res=iCanBus->canRead(messageBuffer, messages, &readMessages);
		
		// parse the messages 
		for(int i=0; i<readMessages; i++)
		{
			if (messageBuffer[i].getId() == 0x3Ca)
			{
				unsigned_gaugeData[0] = (messageBuffer[i].getData()[1]<<8)|messageBuffer[i].getData()[0];
				unsigned_gaugeData[1] = (messageBuffer[i].getData()[3]<<8)|messageBuffer[i].getData()[2];
				unsigned_gaugeData[2] = (messageBuffer[i].getData()[5]<<8)|messageBuffer[i].getData()[4];
				signed_gaugeData[0] = unsigned_gaugeData[0]-0x7fff;
				signed_gaugeData[1] = unsigned_gaugeData[1]-0x7fff;
				signed_gaugeData[2] = unsigned_gaugeData[2]-0x7fff;
			}
			if (messageBuffer[i].getId() == 0x3Cb)
			{
				unsigned_gaugeData[3] = (messageBuffer[i].getData()[1]<<8)|messageBuffer[i].getData()[0];
				unsigned_gaugeData[4] = (messageBuffer[i].getData()[3]<<8)|messageBuffer[i].getData()[2];
				unsigned_gaugeData[5] = (messageBuffer[i].getData()[5]<<8)|messageBuffer[i].getData()[4];
				signed_gaugeData[3] = unsigned_gaugeData[3]-0x7fff;
				signed_gaugeData[4] = unsigned_gaugeData[4]-0x7fff;
				signed_gaugeData[5] = unsigned_gaugeData[5]-0x7fff;
			}
			/*
			if (messageBuffer[i].getId() == 0x12B) 
			{
				sin_frequency[0] = (messageBuffer[i].getData()[1]<<8) | messageBuffer[i].getData()[0];
				sin_amplitude[0] = (messageBuffer[i].getData()[3]<<8) | messageBuffer[i].getData()[2];
				dutyCycle[0]     = (messageBuffer[i].getData()[5]<<8) | messageBuffer[i].getData()[4];
			}

			if (messageBuffer[i].getId() == 0x12A) 
			{
				position[0]  = (messageBuffer[i].getData()[1]<<8) | messageBuffer[i].getData()[0];
				speed[0]	 = (messageBuffer[i].getData()[3]<<8) | messageBuffer[i].getData()[2];
				pid[0]		 = (messageBuffer[i].getData()[5]<<8) | messageBuffer[i].getData()[4];
				torque[0]    = (messageBuffer[i].getData()[7]<<8) | messageBuffer[i].getData()[6];
			}*/
		}
		joint_sensor_data[0]=signed_gaugeData[0];
		joint_sensor_data[1]=signed_gaugeData[1];
		joint_sensor_data[2]=signed_gaugeData[2];
		joint_sensor_data[3]=signed_gaugeData[3];
		joint_sensor_data[4]=signed_gaugeData[4];
		joint_sensor_data[5]=signed_gaugeData[5];
	
		if(stage >=0)
		{
			if (log_start && stage_counter>=30 && stage_counter<40) print_out(fp);

			static double time_old_wd=Time::now();
			double time_wd=Time::now();
			fprintf(stdout,"s:%d, count: %d, t:%+3.3f |", stage, stage_counter, time_wd-time_old_wd);
			print_out(stdout);
			time_old_wd=time_wd;
		}
		
		if (log_start && stage_counter==0)
		{
			switch (stage)
			{
				case  0: jnt_goto(  0,  0,  0, 0); break;
				case  1: jnt_goto( 10,  0,  0, 0); break;
				case  2: jnt_goto( 20,  0,  0, 0); break;
				case  3: jnt_goto( 30,  0,  0, 0); break;
				case  4: jnt_goto( 40,  0,  0, 0); break;
				case  5: jnt_goto( 50,  0,  0, 0); break;
				case  6: jnt_goto( 60,  0,  0, 0); break;
				case  7: jnt_goto( 70,  0,  0, 0); break;
				case  8: jnt_goto( 80,  0,  0, 0); break;
				case  9: jnt_goto( 90,  0,  0, 0); break;

				default:
					fprintf(stdout,"Invalid stage: %d\n",stage);
					fprintf(stdout,"Finished\n");
					jnt_goto(  0,  0,  0, 0);
					log_start =false;
					return;
				break;
			}
		}

		if (stage_counter>=40) //4 seconds
		{
			stage++;
			stage_counter=0;
		}
		else
		{
			stage_counter++;
		}
	}

    void threadRelease()
    {
        iBufferFactory->destroyBuffer(messageBuffer);
        ECANdriver.close();
		fclose(fp);
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