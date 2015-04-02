// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Marco Maggiali, Marco Randazzo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */
#include "candriver.h"
#include <stdio.h>

#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>

using namespace yarp::os;
using namespace yarp::dev;

///*****************************************************************/
cDriver::cDriver ()
{

}


//*****************************************************************/
int cDriver::init (Searchable &config)
{
    bool ret;

    ret=dd.open(config);
    if (!ret)
        return -1;
    
    dd.view(iCanBus);
    dd.view(iFactory);
    
    if (iCanBus==0)
        return -1;
    if (iFactory==0)
        return -1;

    int i;
	for (i = 0x300; i < 0x3FF; i++) iCanBus->canIdAdd (i);
	for (i = 0x200; i < 0x2FF; i++) iCanBus->canIdAdd (i); //for strain board (polling messages used for calibration)

    iCanBus->canSetBaudRate(0); //0=1Mbit/s

    return 0;
}

int cDriver::uninit ()
{
    dd.close();
    return true;
}

//*****************************************************************/
int cDriver::receive_message(CanBuffer &messages, int howMany, double TIMEOUT)
{
    bool  ret;

    if (howMany>MAX_READ_MSG)
        return 0;

    unsigned int how_many_messages=0;
    int read=0;
    int count=0;

    double start=Time::now();
    double now=start;
    bool done=false;

    CanBuffer tmpBuff=createBuffer(MAX_READ_MSG);
    while(!done)
        {
            
            ret=iCanBus->canRead(tmpBuff, MAX_READ_MSG, &how_many_messages, false);

            if (read+how_many_messages>MAX_READ_MSG)
                {
                    how_many_messages=(MAX_READ_MSG-read);
                }

            for(unsigned int k=0;k<how_many_messages;k++)
                {
                    messages[read]=tmpBuff[k];
                    read++;
                }
            
            now=Time::now();
            
            if (read>=howMany)
                {
                    done=true;
                    read=howMany;
                }
                
            if ( (now-start)>TIMEOUT)
                done=true;

            
            //  Time::delay(0.0);
        }

    destroyBuffer(tmpBuff);
    if(!ret) 
        return 0;

    return read;
}

//*****************************************************************/
int cDriver::send_message(CanBuffer &message, int messages)
{
    unsigned int sent=0;

	bool ret = iCanBus->canWrite(message, messages, &sent);
	
	if(!ret)
        return 0;
	else
		return sent;
}

yarp::dev::CanBuffer cDriver::createBuffer(int m)
{
    return iFactory->createBuffer(m);
}


void cDriver::destroyBuffer(yarp::dev::CanBuffer &buff)
{
    iFactory->destroyBuffer(buff);
}
