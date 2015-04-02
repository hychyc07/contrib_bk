// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
*
\author Marco Randazzo

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/canBusSniffer/main.cpp.
**/

#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Network.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/ctrl/math.h>

using namespace yarp::math;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

using namespace std;

const int SNIFFER_THREAD_RATE=50; //ms
const double arrow_len = 0.15;

class SnifferThread: public RateThread
{
	BufferedPort<Vector> port_in0;
	BufferedPort<Vector> port_in1;
	BufferedPort<Vector> port_in2;
	BufferedPort<Vector> port_in3;
	BufferedPort<Vector> port_in4;
	BufferedPort<Vector> port_in5;
	BufferedPort<Vector> port_in6;
	BufferedPort<Bottle> port_out;
	public:
    SnifferThread(int r=SNIFFER_THREAD_RATE): RateThread(r)
	{
		fprintf(stdout,"constructor \n");
	}

/*
	T = [ 0 -1  0 0; 
	      0  0  1 0.5976;
		 -1  0  0 -0.026;
		  0  0  0 1     ] 
*/

    bool threadInit()
    {
		fprintf(stdout,"init \n");
		port_in0.open(string("/simCOM0:i").c_str());
		port_in1.open(string("/simCOM1:i").c_str());
		port_in2.open(string("/simCOM2:i").c_str());
		port_in3.open(string("/simCOM3:i").c_str());
		port_in4.open(string("/simCOM4:i").c_str());
		port_in5.open(string("/simCOM5:i").c_str());
		port_in6.open(string("/simCOM6:i").c_str());
		port_out.open(string("/simCOM:o").c_str());
		yarp::os::Network::connect("/wholeBodyDynamics/com:o","/simCOM0:i");
		yarp::os::Network::connect("/wholeBodyDynamics/left_leg/com:o","/simCOM1:i");
		yarp::os::Network::connect("/wholeBodyDynamics/left_arm/com:o","/simCOM2:i");
		yarp::os::Network::connect("/wholeBodyDynamics/right_leg/com:o","/simCOM3:i");
		yarp::os::Network::connect("/wholeBodyDynamics/right_arm/com:o","/simCOM4:i");
		yarp::os::Network::connect("/wholeBodyDynamics/head/com:o","/simCOM5:i");
		yarp::os::Network::connect("/wholeBodyDynamics/torso/com:o","/simCOM6:i");
		yarp::os::Network::connect("/simCOM:o","/icubSim/world");

		Bottle a;
		a.clear();
		a.addString("world");
		a.addString("del");
		a.addString("all");
		port_out.prepare() = a;
		port_out.write();

		Vector *com_in0 = port_in0.read(true); double mass_0 = (*com_in0)[3];
		Vector *com_in1 = port_in1.read(true); double mass_1 = (*com_in1)[3];
		Vector *com_in2 = port_in2.read(true); double mass_2 = (*com_in2)[3];
		Vector *com_in3 = port_in3.read(true); double mass_3 = (*com_in3)[3];
		Vector *com_in4 = port_in4.read(true); double mass_4 = (*com_in4)[3];
		Vector *com_in5 = port_in5.read(true); double mass_5 = (*com_in5)[3];
		Vector *com_in6 = port_in6.read(true); double mass_6 = (*com_in6)[3];
		double ball_size = 0.06/5;
		bool   ball_size_fixed=false;
		yarp::os::Time::delay(0.2);

		//0
		a.clear();
		a.addString("world");
		a.addString("mk");
		a.addString("ssph");
		com_in0[3];
		if (ball_size_fixed) a.addDouble(0.06); else a.addDouble(ball_size/3*mass_0);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addInt(255);
		a.addInt(0);
		a.addInt(0);
		port_out.prepare() = a;
		port_out.write();
		yarp::os::Time::delay(0.2);

		//1
		a.clear();
		a.addString("world");
		a.addString("mk");
		a.addString("ssph");
		if (ball_size_fixed) a.addDouble(0.06); else a.addDouble(ball_size*mass_1);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addInt(0);
		a.addInt(255);
		a.addInt(0);
		port_out.prepare() = a;
		port_out.write();
		yarp::os::Time::delay(0.2);

		//2
		a.clear();
		a.addString("world");
		a.addString("mk");
		a.addString("ssph");
		if (ball_size_fixed) a.addDouble(0.06); else a.addDouble(ball_size*mass_2);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addInt(0);
		a.addInt(255);
		a.addInt(0);
		port_out.prepare() = a;
		port_out.write();
		yarp::os::Time::delay(0.2);

		//3
		a.clear();
		a.addString("world");
		a.addString("mk");
		a.addString("ssph");
		if (ball_size_fixed) a.addDouble(0.06); else a.addDouble(ball_size*mass_3);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addInt(0);
		a.addInt(255);
		a.addInt(0);
		port_out.prepare() = a;
		port_out.write();
		yarp::os::Time::delay(0.2);

		//4
		a.clear();
		a.addString("world");
		a.addString("mk");
		a.addString("ssph");
		if (ball_size_fixed) a.addDouble(0.06); else a.addDouble(ball_size*mass_4);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addInt(0);
		a.addInt(255);
		a.addInt(0);
		port_out.prepare() = a;
		port_out.write();
		yarp::os::Time::delay(0.2);

		//5
		a.clear();
		a.addString("world");
		a.addString("mk");
		a.addString("ssph");
		if (ball_size_fixed) a.addDouble(0.06); else a.addDouble(ball_size*mass_5);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addInt(0);
		a.addInt(255);
		a.addInt(0);
		port_out.prepare() = a;
		port_out.write();
		yarp::os::Time::delay(0.2);

		//6
		a.clear();
		a.addString("world");
		a.addString("mk");
		a.addString("ssph");
		if (ball_size_fixed) a.addDouble(0.06); else a.addDouble(ball_size*mass_6);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addInt(0);
		a.addInt(255);
		a.addInt(0);
		port_out.prepare() = a;
		port_out.write();
		yarp::os::Time::delay(0.2);

		//7
		a.clear();
		a.addString("world");
		a.addString("mk");
		a.addString("sbox");
		a.addDouble(0.02); 
		a.addDouble(0.02); 
		a.addDouble(arrow_len); 
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addDouble(0.0);
		a.addInt(255);
		
		a.addInt(0);
		a.addInt(0);
		port_out.prepare() = a;
		port_out.write();
		yarp::os::Time::delay(0.2);
		return true;
    }

    void run()
    {
		Vector *com_in0 = port_in0.read(true);
		Vector *com_in1 = port_in1.read(true);
		Vector *com_in2 = port_in2.read(true);
		Vector *com_in3 = port_in3.read(true);
		Vector *com_in4 = port_in4.read(true);
		Vector *com_in5 = port_in5.read(true);
		Vector *com_in6 = port_in6.read(true);

		Vector com_out0(3);
		Vector com_out1(3);
		Vector com_out2(3);
		Vector com_out3(3);
		Vector com_out4(3);
		Vector com_out5(3);
		Vector com_out6(3);
		Vector t0(3);
		t0[0]=0;
		t0[1]=0.5976;
		t0[2]=-0.0260;
		com_out0.zero();
		com_out1.zero();
		com_out2.zero();
		com_out3.zero();
		com_out4.zero();
		com_out5.zero();
		com_out6.zero();

		com_out0[0] = -(*com_in0)[1]+0;
		com_out0[1] =  (*com_in0)[2]+0.5976;
		com_out0[2] = -(*com_in0)[0]-0.0260;
		com_out1[0] = -(*com_in1)[1]+0;
		com_out1[1] =  (*com_in1)[2]+0.5976;
		com_out1[2] = -(*com_in1)[0]-0.0260;
		com_out2[0] = -(*com_in2)[1]+0;
		com_out2[1] =  (*com_in2)[2]+0.5976;
		com_out2[2] = -(*com_in2)[0]-0.0260;
		com_out3[0] = -(*com_in3)[1]+0;
		com_out3[1] =  (*com_in3)[2]+0.5976;
		com_out3[2] = -(*com_in3)[0]-0.0260;
		com_out4[0] = -(*com_in4)[1]+0;
		com_out4[1] =  (*com_in4)[2]+0.5976;
		com_out4[2] = -(*com_in4)[0]-0.0260;
		com_out5[0] = -(*com_in5)[1]+0;
		com_out5[1] =  (*com_in5)[2]+0.5976;
		com_out5[2] = -(*com_in5)[0]-0.0260;
		com_out6[0] = -(*com_in6)[1]+0;
		com_out6[1] =  (*com_in6)[2]+0.5976;
		com_out6[2] = -(*com_in6)[0]-0.0260;
		/*
		//DEBUG ONLY
		printf ("0 CG: %s\n",com_out0.toString().c_str());
		printf ("1 LL: %s\n",com_out1.toString().c_str());
		printf ("2 LA: %s\n",com_out2.toString().c_str());
		printf ("3 RL: %s\n",com_out3.toString().c_str());
		printf ("4 RA: %s\n",com_out4.toString().c_str());
		printf ("\n");
		*/

		static int cycle=0;
//		cycle = (cycle++)%8; // <- does not behave in linux :(
		cycle++;
		cycle = cycle%8;
        //cycle=3;//force LA
		Bottle a;
		static double time_old_wd=Time::now();
		double time_wd=Time::now();
                fprintf(stdout, "debug cycle %d\n",cycle);

        switch (cycle)
		{
		case 1:
		//world set sph 1 2 2 2 
		a.clear();
		a.addString("world");
		a.addString("set");
		a.addString("ssph");
		a.addInt(1);
		a.addDouble(com_out0(0));
		a.addDouble(com_out0(1));
		a.addDouble(com_out0(2));
		port_out.prepare() = a;
		port_out.write();
		//yarp::os::Time::delay(0.04);

		time_wd=Time::now();
		fprintf(stdout,"time%+3.3f \n", time_wd-time_old_wd);
		time_old_wd=time_wd;
		break;

		case 2:

        a.clear();
        		
        a.addString("world");
		a.addString("set");
		a.addString("ssph");
		a.addInt(2);
		a.addDouble(com_out1(0));
		a.addDouble(com_out1(1));
		a.addDouble(com_out1(2));
      		
        port_out.prepare() = a;
		port_out.write();
		//yarp::os::Time::delay(0.04);
		break;

		case 3:
        a.clear();
		a.addString("world");
		a.addString("set");
		a.addString("ssph");
		a.addInt(3);
		a.addDouble(com_out2(0));
		a.addDouble(com_out2(1));
		a.addDouble(com_out2(2));
		port_out.prepare() = a;
		port_out.write();
	//	yarp::os::Time::delay(0.04);
		break;

		case 4:
		a.clear();
		a.addString("world");
		a.addString("set");
		a.addString("ssph");
		a.addInt(4);
		a.addDouble(com_out3(0));
		a.addDouble(com_out3(1));
		a.addDouble(com_out3(2));
		port_out.prepare() = a;
		port_out.write();
		//yarp::os::Time::delay(0.04);
		break;

		case 5:
		a.clear();
		a.addString("world");
		a.addString("set");
		a.addString("ssph");
		a.addInt(5);
		a.addDouble(com_out4(0));
		a.addDouble(com_out4(1));
		a.addDouble(com_out4(2));
		port_out.prepare() = a;
		port_out.write();
		//yarp::os::Time::delay(0.04);
		break;

		case 6:
		a.clear();
		a.addString("world");
		a.addString("set");
		a.addString("ssph");
		a.addInt(6);
		a.addDouble(com_out5(0));
		a.addDouble(com_out5(1));
		a.addDouble(com_out5(2));
		port_out.prepare() = a;
		port_out.write();
		//yarp::os::Time::delay(0.04);
		break;

		case 7:
		a.clear();
		a.addString("world");
		a.addString("set");
		a.addString("ssph");
		a.addInt(7);
		a.addDouble(com_out6(0));
		a.addDouble(com_out6(1));
		a.addDouble(com_out6(2));
		port_out.prepare() = a;
		port_out.write();
		//yarp::os::Time::delay(0.04);
		break;

		case 8:
		a.clear();
		a.addString("world");
		a.addString("rot");
		a.addString("sbox");
		a.addInt(1);
		double vx=(*com_in0)[4];
		double vy=(*com_in0)[5];
		double vz=(*com_in0)[6];
		double ln=sqrt(vx*vx+vy*vy+vz*vz);
		const double max_vel = 100000;
		double ln_coeff= (ln>0.1?0.1:ln)/0.1;
		double vxn=vx/ln;
		double vyn=vy/ln;
		double vzn=vz/ln;
		double rx= atan2(vz,vy)*180/3.14;
		double ry= atan2(vz,vx)*180/3.14;
		double rz= atan2(vy,vx)*180/3.14;
		// 1y 2z 0x transform
		a.addDouble(ry);
		a.addDouble(rz);
		a.addDouble(rx);
		port_out.prepare() = a;
		port_out.write();
		//yarp::os::Time::delay(0.04);

		a.clear();
		a.addString("world");
		a.addString("set");
		a.addString("sbox");
		a.addInt(1);
		a.addDouble(com_out0(0)+arrow_len*vyn*ln_coeff);
		a.addDouble(com_out0(1)+arrow_len*vzn*ln_coeff);
		a.addDouble(com_out0(2)+arrow_len*vxn*ln_coeff);
		//port_out.prepare() = a;
		//port_out.write();
		//yarp::os::Time::delay(0.04);
		break;
		}

		

		
	}
};

int main(int argc, char *argv[]) 
{
	yarp::os::Network yarp;
	yarp::os::Time::turboBoost();

    if (!yarp.checkNetwork())
	{
		fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
	}

    SnifferThread thread;
    thread.start();

    while(1)
    {
		Time::delay(1);
    }

    thread.stop();
}
