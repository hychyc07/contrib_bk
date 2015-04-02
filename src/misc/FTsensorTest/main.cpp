// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
*
\author Marco Randazzo

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/

#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <candriver.h>
#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <iostream>
#include <iomanip>
#include <string>

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

using namespace std;

#define HEX_VALC 0x8000
int THREAD_RATE=10; //ms
const int NUM_SENSORS=1;
const int NUM_CHANNELS=6;
string name="";
bool no_screen=false;
int full_scale_const[NUM_SENSORS][NUM_CHANNELS];
int adc[NUM_SENSORS][NUM_CHANNELS];

class FtThread: public RateThread
{
	public:
	yarp::os::BufferedPort <Vector> port;
	yarp::dev::CanBuffer tx_buffer;
	yarp::dev::CanBuffer rx_buffer;		
	cDriver *bus;
	int can_address;

    FtThread(int _can_address=0, int r=THREAD_RATE): RateThread(r), can_address(_can_address)
	{
		fprintf(stdout,"constructor \n");
		bus = 0;
	}

    bool threadInit()
    {
		bus = new cDriver;
		Property config;
		config.put("device", "ecan");
	    config.put("CanDeviceNum", 0);
	    config.put("CanTxTimeout", 100);
	    config.put("CanRxTimeout", 100);
	    config.put("CanTxQueueSize", 64);
        config.put("CanRxQueueSize", 64);

		if (can_address!=0)
		{
			printf ("connecting to sensor id %d\n", can_address);
			bus->init(config);
			ft_sensor_init();
		}
		else
		{
			printf ("using only simulated FT sensors\n");
		}

		string temp="/fakeFT";
		if (!name.empty()) temp=temp+"/"+name;
		if (name.c_str()[0]=='/') temp=name;
		port.open (temp.c_str());
		fprintf(stdout,"init \n");
		yarp::os::Time::delay(0.002);
		return true;
    }

    void run()
    {
		if (can_address != 0)
		{
			int len=bus->receive_message(rx_buffer,10,0.1);
			for (int i=0;i<len;i++)
			{
				if (rx_buffer[i].getId()== (0x30A | (can_address<<4)))
				{
					adc[0][0] = (rx_buffer[i].getData()[1]<<8)|rx_buffer[i].getData()[0];
					adc[0][1] = (rx_buffer[i].getData()[3]<<8)|rx_buffer[i].getData()[2];
					adc[0][2] = (rx_buffer[i].getData()[5]<<8)|rx_buffer[i].getData()[4];
				}
				if (rx_buffer[i].getId()== (0x30B | (can_address<<4)))
				{
					adc[0][3] = (rx_buffer[i].getData()[1]<<8)|rx_buffer[i].getData()[0];
					adc[0][4] = (rx_buffer[i].getData()[3]<<8)|rx_buffer[i].getData()[2];
					adc[0][5] = (rx_buffer[i].getData()[5]<<8)|rx_buffer[i].getData()[4];
				}
			}
		}


		Vector& output = port.prepare();
        output.clear();
        output.resize(6,0.0);
		output.data()[0] = (int(adc[0][0])-HEX_VALC)/float(HEX_VALC)*full_scale_const[0][0];
		output.data()[1] = (int(adc[0][1])-HEX_VALC)/float(HEX_VALC)*full_scale_const[0][1];
		output.data()[2] = (int(adc[0][2])-HEX_VALC)/float(HEX_VALC)*full_scale_const[0][2];
		output.data()[3] = (int(adc[0][3])-HEX_VALC)/float(HEX_VALC)*full_scale_const[0][3];
		output.data()[4] = (int(adc[0][4])-HEX_VALC)/float(HEX_VALC)*full_scale_const[0][4];
		output.data()[5] = (int(adc[0][5])-HEX_VALC)/float(HEX_VALC)*full_scale_const[0][5];
		if (!no_screen)
		{
			static double time_old_wd=Time::now();
			double time_wd=Time::now();
			fprintf(stdout,"time%+3.3f       ", time_wd-time_old_wd);
			cout << "       " << output.toString().c_str() << endl;
			time_old_wd=time_wd;
		}

        port.write();
	}

	void ft_sensor_init()
	{	
		int i=0;
		int ns=0;
		int nc=0;
		int net =0;
		int len = 0;
		int mode =0;
		int ret =0;
		int txbufsize=64;
		int rxbufsize=64;
		int txtout = 16;
		int rxtout = 16;

		// ******************************************************************************************************
		// creating can buffers
		tx_buffer=bus->createBuffer(1);
		rx_buffer=bus->createBuffer(MAX_READ_MSG);

		// ******************************************************************************************************
		// sending stop commands
		for (i=0; i<10; i++)
		{
			tx_buffer[0].setId(0x200 | can_address);
			tx_buffer[0].setLen(2);
			tx_buffer[0].getData()[0]=0x07;
			tx_buffer[0].getData()[1]=0x01;
			int r= bus->send_message(tx_buffer,1);
			if (r==1) break;
			else
			{
				printf ("trying to connect to sensor (%d)... " ,i);
			}
			yarp::os::Time::delay (0.1);
		}

		printf ("connected.\n");
		printf ("trying to read fullscales...\n");
		yarp::os::Time::delay (0.1);
		// ******************************************************************************************************
		// read sensor full scales
		ns=0;
		int timeout=0;
		int cs_addr=0;
		int cr_addr=0;
		//for (ns=0; ns<1; ns++) //@@@
		for (ns=0; ns<NUM_SENSORS; ns++) 
		{
			if (ns==0) {cs_addr = (0x200 | can_address); cr_addr = (0x200 | can_address <<4);}

			for (nc=0; nc<NUM_CHANNELS; nc++)
			{
				timeout=0;
				tx_buffer[0].setId(cs_addr);
				tx_buffer[0].setLen(2);
				tx_buffer[0].getData()[0]=0x18;
				tx_buffer[0].getData()[1]=nc;
				int tx=bus->send_message(tx_buffer,1);
				if (tx==0) printf ("transmission error (%d)... ",nc);
				bool exit_do=false;				

				do
				{
					int rx=bus->receive_message(rx_buffer,64,0.001);
					{
						for (int rxc=0; rxc<rx; rxc++)
						{
							if (rx_buffer[rxc].getId()==cr_addr &&
								rx_buffer[rxc].getData()[0] == 0x18 &&
								rx_buffer[rxc].getData()[1] == nc)
								{
									full_scale_const[ns][nc] = (rx_buffer[rxc].getData()[2]<<8)|rx_buffer[rxc].getData()[3];
									printf ("received fullscale ch:%d val:%d\n",nc,full_scale_const[ns][nc]);
									exit_do=true;   //exit do
									break;          //exit for
								}
							else
								{
									//printf ("received unknown message from:%d len:%d\n",rx_buffer[0].getId(),rx_buffer[0].getLen());
								}
						}
					}
					yarp::os::Time::delay (0.001);
					timeout++;
				} 
				while (timeout <10000 && exit_do==false);

				if (timeout>= 10000)
				{
					printf ("Error reading sensor fullscale, sensor %d, channel %d\n", ns,nc);
					return;
				}
			}
		}

		for (ns=0; ns<NUM_SENSORS; ns++)
			{
				printf ("fullscale sensor %d: ", ns);
				for (nc=0; nc<NUM_CHANNELS; nc++)
				{
					printf ("%d ", full_scale_const[ns][nc]);
				}
				printf ("\n");
			}

		yarp::os::Time::delay (1);
		// ******************************************************************************************************
		// sending start commands
		printf ("sending broadcast command...\n");
		tx_buffer[0].setId(0x200 | can_address);
		tx_buffer[0].setLen(2);
		tx_buffer[0].getData()[0]=0x07;
		tx_buffer[0].getData()[1]=0x00;
		bus->send_message(tx_buffer,1);
	}
};

int main(int argc, char *argv[]) 
{
	yarp::os::Network yarp;
	yarp::os::Time::turboBoost();

	YARP_REGISTER_DEVICES(icubmod)

	ResourceFinder rf;
    rf.setVerbose(true);
	rf.configure("ICUB_ROOT",argc,argv);	
	name = rf.find("name").asString();

	// init data	
	int nc=0;
	int ns=0;
	for (ns=0; ns<NUM_SENSORS; ns++)
		for (nc=0; nc<NUM_CHANNELS; nc++)
			{
				full_scale_const[ns][nc]=1;
				adc[ns][nc]=0;
			}

	if (rf.check("no_screen"))
	{
		no_screen=true;
	}

	if (rf.check("rate"))
	{
		THREAD_RATE = rf.find("rate").asInt();
	}

	int can_address=0;
	if (rf.check("can_address"))
	{
		can_address = rf.find("can_address").asInt();
	}
	
    if (!yarp.checkNetwork())
	{
		fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
	}

    FtThread thread(can_address);
    thread.start();

    while(1)
    {
		Time::delay(1);
    }

    thread.stop();
}