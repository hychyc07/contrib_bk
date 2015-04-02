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

//%%%%%%%%%%%%%%%%%%%%%%%%OPTIONS%%%%%%%%%%%%%%%%%%%%%
#define ONLY_SCREEN
//%%%%%%%%%%%%%%%%%%%%%%%%OPTIONS%%%%%%%%%%%%%%%%%%%%%

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
	BufferedPort<Bottle>            joint_estimated_sensor_port;
	BufferedPort<Bottle>            joint_model_data_port;
	BufferedPort<Bottle>            jacobian_port;
	BufferedPort<Bottle>            FT_calibrated_port;
	BufferedPort<Bottle>            FT_ext_estim_port;
	BufferedPort<Bottle> 			proj_wrench;
	BufferedPort<Vector>            port_out_jnt_calib;
	BufferedPort<Vector>            port_end_eff;

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
	signed short   signed_gaugeData[6];
	signed short dataBuffer[6];
	Vector externalSensorOffset;
	double end_effector_data [6];

	int    stage_counter;
	int    stage;
	double joint_angle[4];
	double joint_sensor_data_after_calib[6];
	double joint_sensor_data[6];
	double joint_estimated_sensor_data[6];
	double joint_model_data[6];
	double FT_sensor_data[6];
	double FT_virtual_sensor_data[6];
	double joint_jacobian[6];
	double j3offset;
	double sj3offset;
	double proj_wrench_data[6];
	int cal;

public:

	void jnt_goto (double p1, double p2, double p3, double p4)
	{
		ipos->positionMove(0,p1);
		ipos->positionMove(1,p2);
		ipos->positionMove(2,p3);
		ipos->positionMove(3,p4);
	}

	void print_out(FILE* out_file)
	{
		int i=0;
		for (i=0; i<4; i++)
			fprintf(out_file,"%+7.2f ",joint_angle[i]); //parto da 1

		fprintf(out_file,"     ");
			fprintf(out_file,"%+7.0f ",joint_sensor_data[0]); //5
			fprintf(out_file,"%+7.0f ",joint_sensor_data[1]); 
			fprintf(out_file,"%+7.0f ",joint_sensor_data[2]);
			fprintf(out_file,"%+7.0f ",joint_sensor_data[3]);
			fprintf(out_file,"%+7.0f ",joint_sensor_data[4]);
			fprintf(out_file,"%+7.0f ",joint_sensor_data[5]); //10
		fprintf(out_file,"     ");
			fprintf(out_file,"%+7.4f ",joint_estimated_sensor_data[0]); //11
			fprintf(out_file,"%+7.4f ",joint_estimated_sensor_data[1]);
			fprintf(out_file,"%+7.4f ",joint_estimated_sensor_data[2]);
			fprintf(out_file,"%+7.4f ",joint_estimated_sensor_data[3]);
		fprintf(out_file,"     ");
			fprintf(out_file,"%+7.4f ",joint_model_data[0]); //15
			fprintf(out_file,"%+7.4f ",joint_model_data[1]);
			fprintf(out_file,"%+7.4f ",joint_model_data[2]);
			fprintf(out_file,"%+7.4f ",joint_model_data[3]);
		fprintf(out_file,"     ");
			fprintf(out_file,"%+7.4f ",joint_sensor_data_after_calib[0]); //19
			fprintf(out_file,"%+7.4f ",joint_sensor_data_after_calib[1]);  
			fprintf(out_file,"%+7.4f ",joint_sensor_data_after_calib[2]);
			fprintf(out_file,"%+7.4f ",joint_sensor_data_after_calib[3]); 
		fprintf(out_file,"     ");
			fprintf(out_file,"%+7.4f ",joint_jacobian[0]); //19
			fprintf(out_file,"%+7.4f ",joint_jacobian[1]);  
			fprintf(out_file,"%+7.4f ",joint_jacobian[2]);
			fprintf(out_file,"%+7.4f ",joint_jacobian[3]); 
		fprintf(out_file,"     ");

		fprintf(out_file,"\n");
	}

    SnifferThread(int r=SNIFFER_THREAD_RATE): RateThread(r)
    {
		messages = localBufferSize;

		cal = 10;
		for(int i=0; i<6; i++)
		{
			unsigned_gaugeData[i] = 0;
			signed_gaugeData[i] = 0;
			dataBuffer[i] = 0;
			joint_sensor_data[i]=0;
			joint_estimated_sensor_data[i]=0;
			FT_sensor_data[i]=0;
			FT_virtual_sensor_data[i]=0;
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
		FT_calibrated_port.open(("/armV2Sniffer/calibrated_FT:i"));
		FT_ext_estim_port.open(("/armV2Sniffer/FT_ext_estim:i"));
		joint_estimated_sensor_port.open(("/armV2Sniffer/jointestimatedsensor:i"));
		joint_model_data_port.open(("/armV2Sniffer/jointmodeldata:i"));
		jacobian_port.open(("/armV2Sniffer/jacobian:i"));
		port_out_jnt_calib.open("/armV2Sniffer/jointCalibratedTorques:o");
		proj_wrench.open("/armV2Sniffer/projectedWrench:i");
		port_end_eff.open("/armV2Sniffer/end_eff:i");

		if (!yarp::os::Network::connect("/tqObs/right_arm/limbTorques:o","/armV2Sniffer/jointestimatedsensor:i"))
			fprintf(stderr,"ERROR: connect1 failed...\n");
		if (!yarp::os::Network::connect("/tqObs/right_arm/modelTorques:o","/armV2Sniffer/jointmodeldata:i"))
			fprintf(stderr,"ERROR: connect2 failed...\n");
		if (!yarp::os::Network::connect("/tqObs/right_arm/jacobianTorques:o","/armV2Sniffer/jacobian:i"))
			fprintf(stderr,"ERROR: connect jacob failed...\n");
		if (!yarp::os::Network::connect("/icubV2/right_arm/analog:o","/armV2Sniffer/calibrated_FT:i"))
			fprintf(stderr,"ERROR: connect jacob failed...\n");
		if (!yarp::os::Network::connect("/icubV2/wrench_right_arm/analog:o","/armV2Sniffer/end_eff:i"))
			fprintf(stderr,"ERROR: connect end_eff failed...\n");


		if (!yarp::os::Network::connect("/tqObs/right_arm/projectedWrench:o","/armV2Sniffer/projectedWrench:i"))
			fprintf(stderr,"ERROR: connect /projectedWrench failed...\n");
		if (!yarp::os::Network::connect("/tqObs/right_arm/extEstimWrench:o","/armV2Sniffer/FT_ext_estim:i"))
			fprintf(stderr,"ERROR: connect /FT_ext_estim failed...\n");

		Bottle *joint_est=joint_estimated_sensor_port.read(true); 
		j3offset = joint_est->get(3).asDouble(); //3 because there is 1 at hte beginning
		Bottle *ext_FT_offset=joint_estimated_sensor_port.read(true); 

		Vector *end_eff=port_end_eff.read(true);
		externalSensorOffset=*end_eff;

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

		//send the start message for the joint sensors
		messageBuffer[0].setId(0x20C);
		messageBuffer[0].setLen(2);
		messageBuffer[0].getData()[0]=7;
		messageBuffer[0].getData()[1]=3;
		iCanBus->canWrite(messageBuffer,1,&readMessages);

		cnt = 0;
		char filename [30];
		int filecount=0;
		do
		{
			sprintf (filename, "output%d.dat",filecount);
			fp = fopen(filename,"r");
			filecount++;
		}
		while (fp!=0);
		
		fp = fopen(filename,"w");
		printf ("opened file: %s\n", filename);
	
		//set the speeds
		ipos->setRefSpeed(0,3);
		ipos->setRefSpeed(1,3);
		ipos->setRefSpeed(2,3);
		ipos->setRefSpeed(3,3);
		yarp::os::Time::delay(0.1);

		printf ("moving to start position...\n");
#ifndef ONLY_SCREEN
		jnt_goto(0,  0,  15, 0); 
#endif
		printf ("...press something to continue...\n");
		char tmp [255];
		cin >> tmp;
		printf ("...going...\n");

		return true;
    }

    void run()
    {
		int i=0;

		Bottle *FT_calibrated_bottle=FT_calibrated_port.read(true);
		Bottle *FT_ext_estim_bottle=FT_ext_estim_port.read(true);
		Bottle *joint_estim_sensor_bottle=joint_estimated_sensor_port.read(true);
		Bottle *joint_model_bottle=joint_model_data_port.read(true);	
		Bottle *jacobian_bottle=jacobian_port.read(true);
		Vector *end_eff_v=port_end_eff.read(true);
		end_effector_data[0] =  end_eff_v->data()[0] - externalSensorOffset.data()[0];
		end_effector_data[1] =  end_eff_v->data()[1] - externalSensorOffset.data()[1];
		end_effector_data[2] =  end_eff_v->data()[2] - externalSensorOffset.data()[2];
		end_effector_data[3] =  end_eff_v->data()[3] - externalSensorOffset.data()[3];
		end_effector_data[4] =  end_eff_v->data()[4] - externalSensorOffset.data()[4];
		end_effector_data[5] =  end_eff_v->data()[5] - externalSensorOffset.data()[5];

		Bottle *proj_wrench_bottle=proj_wrench.read(false);

		//double dummy1 =                joint_estim_sensor_bottle->get(0).asDouble();

		if (proj_wrench_bottle)
		{
			proj_wrench_data[0]=proj_wrench_bottle->get(0).asDouble();
			proj_wrench_data[1]=proj_wrench_bottle->get(1).asDouble();
			proj_wrench_data[2]=proj_wrench_bottle->get(2).asDouble();
			proj_wrench_data[3]=proj_wrench_bottle->get(3).asDouble();
			proj_wrench_data[4]=proj_wrench_bottle->get(4).asDouble();
			proj_wrench_data[5]=proj_wrench_bottle->get(5).asDouble();
		}

		joint_estimated_sensor_data[0]=joint_estim_sensor_bottle->get(1).asDouble();
		joint_estimated_sensor_data[1]=joint_estim_sensor_bottle->get(2).asDouble();
		joint_estimated_sensor_data[2]=joint_estim_sensor_bottle->get(3).asDouble();
		joint_estimated_sensor_data[3]=joint_estim_sensor_bottle->get(4).asDouble();

		joint_model_data[0]=joint_model_bottle->get(0).asDouble();
		joint_model_data[1]=joint_model_bottle->get(1).asDouble();
		joint_model_data[2]=joint_model_bottle->get(2).asDouble();
		joint_model_data[3]=joint_model_bottle->get(3).asDouble();

		joint_jacobian[0]=jacobian_bottle->get(0).asDouble();
		joint_jacobian[1]=jacobian_bottle->get(1).asDouble();
		joint_jacobian[2]=jacobian_bottle->get(2).asDouble();
		joint_jacobian[3]=jacobian_bottle->get(3).asDouble();

		FT_sensor_data[0]=FT_calibrated_bottle->get(0).asDouble();
		FT_sensor_data[1]=FT_calibrated_bottle->get(1).asDouble();
		FT_sensor_data[2]=FT_calibrated_bottle->get(2).asDouble();
		FT_sensor_data[3]=FT_calibrated_bottle->get(3).asDouble();
		FT_sensor_data[4]=FT_calibrated_bottle->get(4).asDouble();
		FT_sensor_data[5]=FT_calibrated_bottle->get(5).asDouble();

		
		FT_virtual_sensor_data[0]=FT_ext_estim_bottle->get(0).asDouble();
		FT_virtual_sensor_data[1]=FT_ext_estim_bottle->get(1).asDouble();
		FT_virtual_sensor_data[2]=FT_ext_estim_bottle->get(2).asDouble();
		FT_virtual_sensor_data[3]=FT_ext_estim_bottle->get(3).asDouble();
		FT_virtual_sensor_data[4]=FT_ext_estim_bottle->get(4).asDouble();
		FT_virtual_sensor_data[5]=FT_ext_estim_bottle->get(5).asDouble();

		ienc->getEncoder(0,&joint_angle[0]);	
		ienc->getEncoder(1,&joint_angle[1]);	
		ienc->getEncoder(2,&joint_angle[2]);	
		ienc->getEncoder(3,&joint_angle[3]);	
       
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
		
#ifdef ONLY_SCREEN
		fprintf(stdout,"     ");
	/*		fprintf(stdout,"%+7.0f ",joint_sensor_data[0]); //5
			fprintf(stdout,"%+7.0f ",joint_sensor_data[1]); 
			fprintf(stdout,"%+7.0f ",joint_sensor_data[2]);
			fprintf(stdout,"%+7.0f ",joint_sensor_data[3]);
			fprintf(stdout,"%+7.0f ",joint_sensor_data[4]);
			fprintf(stdout,"%+7.0f ",joint_sensor_data[5]); //10
*/
			/*fprintf(stdout,"%+7.4f ",joint_estimated_sensor_data[0]); //11
			fprintf(stdout,"%+7.4f ",joint_estimated_sensor_data[1]);
			fprintf(stdout,"%+7.4f ",joint_estimated_sensor_data[2]);
			fprintf(stdout,"%+7.4f ",joint_estimated_sensor_data[3]);
			fprintf(stdout,"      &     ");
			fprintf(stdout,"%+7.3f ",joint_sensor_data_after_calib[0]); 
			fprintf(stdout,"%+7.3f ",joint_sensor_data_after_calib[1]); 
			fprintf(stdout,"%+7.3f ",joint_sensor_data_after_calib[2]);
			fprintf(stdout,"%+7.3f ",joint_sensor_data_after_calib[3]);
			fprintf(stdout,"      &     ");
			fprintf(stdout,"%+7.4f ",joint_estimated_sensor_data[0]-joint_sensor_data_after_calib[0]); //11
			fprintf(stdout,"%+7.4f ",joint_estimated_sensor_data[1]-joint_sensor_data_after_calib[1]);
			fprintf(stdout,"%+7.4f ",joint_estimated_sensor_data[2]-joint_sensor_data_after_calib[2]);
			fprintf(stdout,"%+7.4f ",joint_estimated_sensor_data[3]-joint_sensor_data_after_calib[3]);*/

		    fprintf(stdout,"      \n    ");
			fprintf(stdout,"%+7.2f ",FT_virtual_sensor_data[0]); //11
			fprintf(stdout,"%+7.2f ",FT_virtual_sensor_data[1]);
			fprintf(stdout,"%+7.2f ",FT_virtual_sensor_data[2]);
			fprintf(stdout,"%+7.3f ",FT_virtual_sensor_data[3]);
			fprintf(stdout,"%+7.3f ",FT_virtual_sensor_data[4]);
			fprintf(stdout,"%+7.3f ",FT_virtual_sensor_data[5]);
			fprintf(stdout,"      \n    ");
			fprintf(stdout,"%+7.2f ",end_effector_data[0]); //11
			fprintf(stdout,"%+7.2f ",end_effector_data[1]);
			fprintf(stdout,"%+7.2f ",end_effector_data[2]);
			fprintf(stdout,"%+7.3f ",end_effector_data[3]);
			fprintf(stdout,"%+7.3f ",end_effector_data[4]);
			fprintf(stdout,"%+7.3f ",end_effector_data[5]);
			fprintf(stdout,"      \n    ");
			fprintf(stdout,"%+7.2f ",proj_wrench_data[0]); //11
			fprintf(stdout,"%+7.2f ",proj_wrench_data[1]);
			fprintf(stdout,"%+7.2f ",proj_wrench_data[2]);
			fprintf(stdout,"%+7.3f ",proj_wrench_data[3]);
			fprintf(stdout,"%+7.3f ",proj_wrench_data[4]);
			fprintf(stdout,"%+7.3f ",proj_wrench_data[5]);
			fprintf(stdout,"      \n    ");
			fprintf(stdout,"      \n    ");
			fprintf(stdout,"      \n    ");
			/*fprintf(stdout,"      *     ");
			fprintf(stdout,"%+7.3f ",joint_jacobian[0]); 
			fprintf(stdout,"%+7.3f ",joint_jacobian[1]); 
			fprintf(stdout,"%+7.3f ",joint_jacobian[2]);
			fprintf(stdout,"%+7.3f ",joint_jacobian[3]);
			fprintf(stdout,"      *     ");
			fprintf(stdout,"%+7.3f ",joint_sensor_data_after_calib[0]+joint_jacobian[0]); 
			fprintf(stdout,"%+7.3f ",joint_sensor_data_after_calib[1]+joint_jacobian[1]); 
			fprintf(stdout,"%+7.3f ",joint_sensor_data_after_calib[2]+joint_jacobian[2]);
			fprintf(stdout,"%+7.3f ",joint_sensor_data_after_calib[3]+joint_jacobian[3]);*/


			//fprintf(stdout,"   sum:%+7.0f ",joint_sensor_data[4]+joint_sensor_data[5]);
			//fprintf(stdout,"   dif:%+7.0f ",joint_sensor_data[4]-joint_sensor_data[5]); //10
		fprintf(stdout,"\n");
#endif

/*
			joint_sensor_data_after_calib[0]= double(-signed_gaugeData[4]+ signed_gaugeData[5] - (- 984 + 1438)) * -1.370427775582143e-004 - (-0.122200000000000);
			joint_sensor_data_after_calib[1]= double(-signed_gaugeData[2]+ signed_gaugeData[3] - (+ 1318 - 244)) *  2.258853815799500e-004 - (-0.002500000000000);
			joint_sensor_data_after_calib[2]= 0;//(signed_gaugeData[0]+0.0)*1.0 + (signed_gaugeData[1]+0.0)*1.0;
			joint_sensor_data_after_calib[3]= double(signed_gaugeData[0] - -(173))*8.0386e-004 - (-0.229000000000000); //elbow
*/
/*
			joint_sensor_data_after_calib[0]= double(-signed_gaugeData[4]+ signed_gaugeData[5] - ( -(-1204) + 1612)) * -1.410071720430783e-004 + (-0.071100000000000);
			joint_sensor_data_after_calib[1]= double(-signed_gaugeData[2]+ signed_gaugeData[3] - ( -(1471) + 311)) *   2.325962999328555e-004 + (-0.008500000000000);
			joint_sensor_data_after_calib[2]= 0;//(signed_gaugeData[0]+0.0)*1.0 + (signed_gaugeData[1]+0.0)*1.0;
			joint_sensor_data_after_calib[3]= double(signed_gaugeData[0] - (-190)) * 7.948558573199704e-004 + (-0.114700000000000); //elbow
*/	
			joint_sensor_data_after_calib[0]= double(-signed_gaugeData[4]+ signed_gaugeData[5] - ( -(-1204) + 1612)) *  -0.1370e-3 +
											  double(-signed_gaugeData[2]+ signed_gaugeData[3] - ( -(1471)  + 311))   * -0.2473e-3 ;//(-0.071100000000000);
			
			joint_sensor_data_after_calib[1]= double(-signed_gaugeData[2]+ signed_gaugeData[3] - ( -(1471) + 311)) *   2.325962999328555e-004 + (-0.008500000000000);
			if (cal>0)
			{
				sj3offset = 0.015* FT_sensor_data[1] + FT_sensor_data[5];
				cal --;
			}
			joint_sensor_data_after_calib[2]= 0.015* FT_sensor_data[1] + FT_sensor_data[5] + j3offset - sj3offset;
			joint_sensor_data_after_calib[3]= double(signed_gaugeData[0] - (-190)) * 7.948558573199704e-004 + (-0.114700000000000); //elbow

			Vector &out_jnt_calib=port_out_jnt_calib.prepare();
			Vector data;
			data.resize(4,0.0);
			data[0]= joint_sensor_data_after_calib[0];
			data[1]= joint_sensor_data_after_calib[1];
			data[2]= joint_sensor_data_after_calib[2];
			data[3]= joint_sensor_data_after_calib[3];
			out_jnt_calib=data;
			/*out_jnt_calib.addDouble(joint_sensor_data_after_calib[0]);
			out_jnt_calib.addDouble(joint_sensor_data_after_calib[1]);
			out_jnt_calib.addDouble(joint_sensor_data_after_calib[2]);
			out_jnt_calib.addDouble(joint_sensor_data_after_calib[3]);
			*/
			port_out_jnt_calib.write();

		if(stage >=0)
		{
			#ifndef ONLY_SCREEN
			if (log_start && stage_counter>=10 && stage_counter<20) print_out(fp); //^^^
			#endif

			static double time_old_wd=Time::now();
			double time_wd=Time::now();
			fprintf(stdout,"s:%2d, count: %2d, t:%+3.3f |", stage, stage_counter, time_wd-time_old_wd);
			#ifndef ONLY_SCREEN
			print_out(stdout); //^^^
			#endif
			time_old_wd=time_wd;
		}
		
		if (log_start && stage_counter==0)
		{
			//jnt_goto(-stage,  0,  15, 0); //use this fo joint 0 only
			//jnt_goto(0,  stage,  15, 0); //use this fo joint 1 only
			//jnt_goto(0,  0,  15, stage); //use this fo joint 3 only (elbow)
			//jnt_goto(0,  90,  15, stage);  //use this fo joint 3 only (elbow) in plane

			if (stage>91)
			{
				fprintf(stdout,"Invalid stage: %d\n",stage);
				fprintf(stdout,"Finished\n");
				#ifndef ONLY_SCREEN
				jnt_goto(  0,  0,  15, 0); //^^^
				#endif
				log_start =false;
				return;
			}
			/*
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
			}*/
		}

		if (stage_counter>=20) //2 seconds
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