/*
 * This class use the USB retina driver wrote by
 * Martin Ebner, IGI / TU Graz (ebner at igi.tugraz.at)
 *
 *  The term of the contract of the used source is :
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 2.6.3 version of drivers/usb/usb-skeleton.c
 * (Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com))
 *
 */

#include "device2yarp.h"

using namespace std;
using namespace yarp::os;
C_device2yarp::C_device2yarp(string deviceNum, bool i_bool, string i_fileName):RateThread(10), save(i_bool)
{
//	xmask = 0x000000fE;
//	ymask = 0x00007f00;
//	yshift = 8;
//	xshift = 1;
//	polshift = 0;
//	polmask = 0x00000001;

//	retinalSize = 128;

    len=0;
    sz=0;
    memset(buffer, 0, SIZE_OF_DATA);
    if(save)
        raw = fopen(i_fileName.c_str(), "wb");
    str_buf << "/icub/retina" << deviceNum << ":o";
	port.open(str_buf.str().c_str());
    str_buf.str("");
	/* open device file /dev/retina0*/
	str_buf << "/dev/retina" << deviceNum;
	cout << str_buf.str() << endl;
	file_desc = open(str_buf.str().c_str(), O_RDWR);
	if (file_desc < 0)
		cout << "Can't open device file: %s\n" << str_buf.str() << endl;
    else
    {
#ifdef FAST
        unsigned char bias[] = {0xb8,               //request
                                0x00, 0x00,         //value
                                0x00, 0x00,          //index
                                0x00,0x07,0xc8,	    // cas
								0x10,0xe9,0x8C,		// injGnd
								0xFF,0xFF,0xFF,		// reqPd
								0x7c,0x7f,0xf5,		// puX
								0x00,0x00,0x84,		// diffOff          ++
								0x02,0x6d,0xab,		// req
								0x00,0x03,0xc9,		// refr             ++ refractory of the pixels
								0xFF,0xFF,0xFF,		// puY
								0x03,0x34,0x4c,		// diffOn           ++
								0x00,0x33,0x45,		// diff
								0x00,0x01,0x0f,		// foll
								0x00,0x01,0x0f}; 	// Pr               ++ Velocity of the "log circuit"
#else
#ifdef SLOW
        unsigned char bias[] = {0xb8,               //request
                                0x00, 0x00,         //value
                                0x00, 0x00,          //index
                                0x00,0x00,0x36,	    // cas
								0x10,0xe9,0x8C,		// injGnd
								0xFF,0xFF,0xFF,		// reqPd
								0x7c,0x7f,0xf5,		// puX
								0x00,0x00,0x84,		// diffOff
								0x02,0x6d,0xab,		// req
								0x00,0x00,0x06,		// refr
								0xFF,0xFF,0xFF,		// puY
								0x07,0x5c,0x8b,		// diffOn
								0x00,0x75,0xc9,		// diff
								0x00,0x00,0x33,		// foll
								0x00,0x00,0x03}; 	// Pr
#else
//        unsigned char bias[] = {0xb8,               //request
//                                0x00, 0x00,         //value
//                                0x00, 0x00,          //index
//                                0x00,0x07,0xc8,	    // cas
//								0x10,0xe9,0x8C,		// injGnd
//								0xFF,0xFF,0xFF,		// reqPd
//								0x7c,0x7f,0xf5,		// puX
//								0x00,0x01,0xbe,		// diffOff
//								0x04,0xb9,0x56,		// req
//								0x00,0x09,0x31,		// refr
//								0xFF,0xFF,0xFF,		// puY
//								0x01,0xa5,0xae,		// diffOn
//								0x00,0x1d,0x72,		// diff
//								0x00,0x01,0x10,		// foll
//								0x00,0x07,0x5d}; 	// Pr
        unsigned char bias[] = {0xb8,               //request
                                0x00, 0x00,         //value
                                0x00, 0x00,          //index
                                0x00,0x07,0xc8,	    // cas
								0x10,0xe9,0x8C,		// injGnd
								0xFF,0xFF,0xFF,		// reqPd
								0x7c,0x7f,0xf5,		// puX
								0x00,0x04,0xfe,		// diffOff
								0x04,0xb9,0x56,		// req
								0x00,0x09,0x31,		// refr
								0xFF,0xFF,0xFF,		// puY
								0x01,0xe5,0x20,		// diffOn
								0x00,0x26,0xd2,		// diff
								0x00,0x01,0x10,		// foll
								0x00,0x09,0x31}; 	// Pr
#endif //SLOW
#endif //FAST


        int err = write(file_desc,bias,41); //5+36
#ifdef _DEBUG
        cout << "Return of the bias writing : " << err << endl;
#endif
        unsigned char start[5];
        start[0] = 0xb3;
        start[1] = 0;
        start[2] = 0;
        start[3] = 0;
        start[4] = 0;
        err = write(file_desc,start,5);
#ifdef _DEBUG
        cout << "Return of the start writing : " << err << endl;
#endif
    }
}

C_device2yarp::~C_device2yarp()
{
    if(save)
        fclose(raw);

    port.close();

    unsigned char stop[5];
    stop[0] = 0xb4;
    stop[1] = 0;
    stop[2] = 0;
    stop[3] = 0;
    stop[4] = 0;
    err = write(file_desc,stop,5);
//--------------------------------------------------------------------------------------------------------//
#ifdef _DEBUG
    printf("%d address events read\n",len/4);
#endif
    close(file_desc);
}



void  C_device2yarp::run()
{
    /* read address events from device file */
    //sz = read(file_desc,buffer,SIZE_OF_DATA);
    sz = pread(file_desc,buffer,SIZE_OF_DATA,0);
#ifdef _DEBUG
    cout << "Size of the buffer : " << sz << endl;
    for(int i=0; i<sz; i+=4)
    {
            unsigned int part_1 = 0x00FF&buffer[i];
            unsigned int part_2 = 0x00FF&buffer[i+1];
            unsigned int part_3 = 0x00FF&buffer[i+2];
            unsigned int part_4 = 0x00FF&buffer[i+3];
            unsigned int blob = (part_1)|(part_2<<8);
            unsigned int timestamp = ((part_3)|(part_4<<8))/*&0x7fff*/;
            printf("%x : %x\n", blob, timestamp);
    }
#endif
    C_sendingBuffer data2send(buffer, sz);
    C_sendingBuffer& tmp = port.prepare();
    tmp = data2send;
    port.write();
    if(save)
    {
        fwrite(&sz, sizeof(int), 1, raw);
        fwrite(buffer, 1, sz, raw);
    }
    memset(buffer, 0, SIZE_OF_DATA);
}
