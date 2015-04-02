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
#include <math.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/ctrl/math.h>

using namespace yarp::math;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

using namespace std;

const int SNIFFER_THREAD_RATE=10; //ms


class SnifferThread: public RateThread
{
    BufferedPort<Vector> port_in0;
    BufferedPort<Vector> port_in1;
    BufferedPort<Vector> port_in2;
    BufferedPort<Vector> port_in3;
    BufferedPort<Vector> port_in4;
    BufferedPort<Vector> port_in5;
    BufferedPort<Vector> port_in6;
    BufferedPort<Vector> port_in7;
    BufferedPort<Vector> port_in8;
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

    void threadRelease()
    {
        fprintf(stdout,"cleaning up \n");
        port_in0.interrupt();
        port_in0.close();
        port_in1.interrupt();
        port_in1.close();
        port_in2.interrupt();
        port_in2.close();
        port_in3.interrupt();
        port_in3.close();
        port_in4.interrupt();
        port_in4.close();
        port_in5.interrupt();
        port_in5.close();
        port_in6.interrupt();
        port_in6.close();
        port_in7.interrupt();
        port_in7.close();
        port_in8.interrupt();
        port_in8.close();

        remove_cog("COG");
        remove_cog("LL_COM");
        remove_cog("LA_COM");
        remove_cog("RL_COM");
        remove_cog("RA_COM");
        remove_cog("HD_COM");
        remove_cog("UPP_COM");
        remove_cog("LOW_COM");
        
        port_out.interrupt();
        port_out.close();
    }

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
        port_in7.open(string("/simCOM7:i").c_str());
        port_in8.open(string("/simCOM8:i").c_str());
        port_out.open(string("/simCOM:o").c_str());
        yarp::os::Network::connect("/wholeBodyDynamics/com:o","/simCOM0:i");
        yarp::os::Network::connect("/wholeBodyDynamics/left_leg/com:o","/simCOM1:i");
        yarp::os::Network::connect("/wholeBodyDynamics/left_arm/com:o","/simCOM2:i");
        yarp::os::Network::connect("/wholeBodyDynamics/right_leg/com:o","/simCOM3:i");
        yarp::os::Network::connect("/wholeBodyDynamics/right_arm/com:o","/simCOM4:i");
        yarp::os::Network::connect("/wholeBodyDynamics/head/com:o","/simCOM5:i");
        yarp::os::Network::connect("/wholeBodyDynamics/torso/com:o","/simCOM6:i");
        yarp::os::Network::connect("/wholeBodyDynamics/upper_body/com:o","/simCOM7:i");
        yarp::os::Network::connect("/wholeBodyDynamics/lower_body/com:o","/simCOM8:i");
        yarp::os::Network::connect("/simCOM:o","/iCubGui/objects");

        Bottle a;
        a.clear();
        a.addString("reset");
        port_out.prepare() = a;
        port_out.write();

        return true;
    }

    void remove_cog(string text)
    {
        Bottle obj;
        obj.clear();
        obj.addString("delete"); // command to add/update an object
        obj.addString(text.c_str());
        port_out.prepare() = obj;
        port_out.write();
        Time::delay(0.1);
    }

    void display_cog(string text,yarp::sig::Vector v, int r, int g, int b)
    {
        Bottle obj;
        obj.clear();
        obj.addString("object"); // command to add/update an object
        obj.addString(text.c_str());

        // object dimensions in millimiters 
        // (it will be displayed as an ellipsoid with the tag "my_object_name")
        bool fixed_size = false;
        if (fixed_size)
        {
            obj.addDouble(20);
            obj.addDouble(20);
            obj.addDouble(20);
        }
        else
        {
            obj.addDouble(pow(v[3],1.0/3.0)*20.0);
            obj.addDouble(pow(v[3],1.0/3.0)*20.0);
            obj.addDouble(pow(v[3],1.0/3.0)*20.0);
        }

        // object position in millimiters
        // reference frame: X=fwd, Y=left, Z=up
        obj.addDouble(v[0]*1000);
        obj.addDouble(v[1]*1000);
        obj.addDouble(v[2]*1000);

        // object orientation (roll, pitch, yaw) in degrees
        obj.addDouble(0);
        obj.addDouble(0);
        obj.addDouble(0);

        // object color (0-255)
        obj.addInt(r);
        obj.addInt(g);
        obj.addInt(b);
        // transparency (0.0=invisible 1.0=solid)
        obj.addDouble(0.9);

        port_out.prepare() = obj;
        port_out.write();
        Time::delay(0.1);
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
        Vector *com_in7 = port_in7.read(true);
        Vector *com_in8 = port_in8.read(true);

        /*
        //DEBUG ONLY
        printf ("0 CG: %s\n",com_out0.toString().c_str());
        printf ("1 LL: %s\n",com_out1.toString().c_str());
        printf ("2 LA: %s\n",com_out2.toString().c_str());
        printf ("3 RL: %s\n",com_out3.toString().c_str());
        printf ("4 RA: %s\n",com_out4.toString().c_str());
        printf ("\n");
        */

        display_cog("COG",   *com_in0,255,0,0);
        display_cog("LL_COM",*com_in1,0,255,0);
        display_cog("LA_COM",*com_in2,0,255,0);
        display_cog("RL_COM",*com_in3,0,255,0);
        display_cog("RA_COM",*com_in4,0,255,0);
        display_cog("HD_COM",*com_in5,0,255,0);
        display_cog("TO_COM",*com_in6,0,255,0);
        display_cog("UPP_COM",*com_in7,0,155,0);
        display_cog("LOW_COM",*com_in8,0,155,0);

        /*
        static double time_old_wd=Time::now();
        double time_wd=Time::now();
        fprintf(stdout,"time%+3.3f \n", time_wd-time_old_wd);
        time_old_wd=time_wd;
        */
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

