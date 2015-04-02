// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
*
\author Marco Randazzo

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/

#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <iostream>
#include <iomanip>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>

#include <libusb.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

using namespace std;

int THREAD_RATE=1; //ms
yarp::os::Semaphore semaphore;

class FtThread: public RateThread
{
    public:
    libusb_device_handle* usb_handle;
    unsigned char         data [64];
    unsigned char         readData [800];
    int                   currentValues[16][20];
    int                   offset[16];
    Vector                vectorData;
    bool                  flag_remove_offset;

    FtThread(int r=THREAD_RATE): RateThread(r)
    {
        memset(data,0,sizeof(data));
        memset(readData,0,sizeof(readData));
        memset(currentValues,0,sizeof(currentValues));
        memset(offset,0,sizeof(offset));
        usb_handle = NULL;
        data[0] = 0x81;
        vectorData.clear();
        flag_remove_offset=true;
    }

    bool threadInit()
    {
        libusb_device **devs;
        int r;
        ssize_t cnt;
        libusb_context * ctx = NULL;

        r = libusb_init(&ctx);
        if (r < 0)
        {
            fprintf(stderr, "failed to initialise libusb\n");
            return false;
        }
        libusb_set_debug(ctx,3);

        //cnt = libusb_get_device_list(NULL, &devs);
        
        usb_handle = libusb_open_device_with_vid_pid (ctx ,(short) 0x04d8, (short) 0x0204);
        if (usb_handle == NULL)
        {
            fprintf(stderr, "failed to open usb device\n");
            return false;
        }
        
        r = libusb_set_configuration(usb_handle, 1);
        if (r!= 0)
        {
            fprintf(stderr, "failed to set configuration\n");
            return false;
        }
/*
        r = libusb_set_interface_alt_setting(usb_handle, 0, -1);
        if (r!= 0)
        {
            fprintf(stderr, "failed to set alternate setting\n");
            return false;
        }
*/
        /*
         * Disable any kernel driver on this device+interface
         */
        if( 0 > (r = libusb_kernel_driver_active(usb_handle, 0)) )
        {
	        printf("Failed to determine if a kernel driver is active on interface\n");
	        libusb_close(usb_handle);
	        return false;
        }
        
        if( r == 1 )
        {
	        if( (r = libusb_detach_kernel_driver(usb_handle, 0)) )
	        {
	            printf( "Failed to detach kernel driver from interface\n");
	            libusb_close(usb_handle);
	            return false;
	        }
        }
        r = libusb_claim_interface(usb_handle, 0);
        if (r!= 0)
        {
            fprintf(stderr, "failed to claim interface\n");
            return false;
        }

        printf ("Input thread starting!\n");
        return true;
    }

    void swap_values(int &a, int &b)
    {
        int c = a;
        a = b;
        b = c;
    }
    
    void run()
    {
        memset(data,0,sizeof(data));
        data[0] = 0x81;
        memset(readData,0,sizeof(readData));
        int transfered_bytes=0;
        int r1 = libusb_bulk_transfer(usb_handle, short(0x01), data, sizeof(data), &transfered_bytes, 50);
        int r2 = libusb_bulk_transfer(usb_handle, short(0x81), readData, sizeof(readData), &transfered_bytes, 2);
        int pckt_cnt = (int)readData[3]<<8|readData[2];
        
        int i=0;
        int j=0;
        static unsigned int critical_counter=0;

        semaphore.wait();
        for (j=0; j<20; j++)
        {
            for (i=0; i<16; i++)
                {
                    currentValues[i][j] = readData[i*2+5+j*40]<<8 | readData[i*2+4+j*40];           
                    if (flag_remove_offset) {currentValues[i][j]-=offset[i];}
                    vectorData.push_back(currentValues[i][j]);
                }
            vectorData.push_back(critical_counter++);         
        }
        semaphore.post();

        //block to remove offset
        static int timer = 0;
        timer++;
        if (timer==1500) 
        for (i=0; i<16; i++)
        {
            offset[i]=currentValues[i][0]-512;
        }

        #ifdef DEBUG_RAW_DATA
        printf ("%d %d %d --- %d\n", r1, r2, transfered_bytes, pckt_cnt);
        for ( j=0; j<20; j++)        
        {   
            printf ("%d ", i+j*40);
            for ( i=0; i<40; i++)
                printf ("%3d ", readData[i+j*40]);
            printf ("\n");
        }
        printf("\n");
        #endif

        #define DEBUG_INT_DATA
        #ifdef DEBUG_INT_DATA
        //printf ("%d %d %d --- %d\n", r1, r2, transfered_bytes, pckt_cnt);
        for (j=0; j<16; j++) 
        {
            for (i=0; i<16; i++)
            {
                printf("%3d ",currentValues[i][j]);
            }
            printf (" %d",timer);
            printf("\n");
        }
        //printf("\n");
        #endif
     
    }

    ~FtThread()
    {

    }
};

class palmSensModule: public RFModule
{
    public:
    FtThread* thread;
    yarp::os::BufferedPort <Vector> out_port;

    bool configure(ResourceFinder &rf)
    {
        out_port.open("/palm_sensor:o");
        thread = new FtThread;
        if (thread->start())
            return true;
        else
            return false;
    }

    double getPeriod()
    {
        return 0.05; //50ms period
    }

    bool updateModule()
    {
        if (out_port.getOutputCount()>0)
        {
            semaphore.wait();
            out_port.prepare()  = thread->vectorData ;
            out_port.write();
            thread->vectorData.clear();
            semaphore.post();
        }
        return true;
    }

    bool close()
    {
        printf("closing module\n");
        out_port.interrupt();
        out_port.close();
        thread->stop();
        delete thread;
        return true;
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

    if (rf.check("rate"))
    {
        THREAD_RATE = rf.find("rate").asInt();
    }

    if (!yarp.checkNetwork())
    {
        fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    palmSensModule mainModule;
    return mainModule.runModule(rf);

}
