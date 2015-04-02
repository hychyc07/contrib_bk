#ifndef C_DEVICE2YARP
#define C_DEVICE2YARP

//yarp include
#include <yarp/os/all.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <unistd.h>

#include "sending_buffer.h"

class C_device2yarp:public RateThread
{
public:
    C_device2yarp(string, bool, string);
    ~C_device2yarp();
    virtual void run();
private:
    //Declaration of the method

    //Declaration of the variables
    BufferedPort<C_sendingBuffer> port;
    FILE* raw;

    int file_desc,len,sz;
    unsigned char buffer_msg[64];
    short enabled;
	char buffer[SIZE_OF_DATA];

    int err;//détection des erreursm
    unsigned int timestamp;
    short cartX, cartY, polarity;

    unsigned int xmask;
    unsigned int ymask;
    int yshift;
    int xshift;
    int polshift;
    int polmask;
    int retinalSize;

    bool save;

    stringstream str_buf;
};
#endif //C_DEVICE2YARP
