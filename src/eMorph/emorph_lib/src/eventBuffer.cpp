// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * Data buffer implementation for the event-based camera.
 *
 * Author: Giorgio Metta, Francesco Rea
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include <iCub/emorph/eventBuffer.h>

using namespace std;
using namespace yarp::os;
using namespace emorph::ebuffer;


namespace emorph
{

namespace ebuffer
{

eventBuffer::eventBuffer()
{
    packet = new char[SIZE_OF_DATA];
    memset(packet, 0, SIZE_OF_DATA);
    size_of_the_packet=0;
}

eventBuffer::eventBuffer(char* i_data, int i_size)
{
    packet = new char[SIZE_OF_DATA];
    memset(packet, 0, SIZE_OF_DATA);
    memcpy(packet, i_data, i_size);
    size_of_the_packet = i_size;
}

eventBuffer::eventBuffer(const eventBuffer& buffer) {
    packet = new char[SIZE_OF_DATA];
    memset(packet, 0, SIZE_OF_DATA);
    if (buffer.size_of_the_packet > 0)
        memcpy(packet, buffer.packet, sizeof(char) * buffer.size_of_the_packet);
    size_of_the_packet = buffer.size_of_the_packet;
}

eventBuffer::~eventBuffer()
{
    delete[] packet;
}

void eventBuffer::operator=(const eventBuffer& buffer) {
    memset(packet, 0, SIZE_OF_DATA);
    if (buffer.size_of_the_packet > 0)
        memcpy(packet, buffer.packet, sizeof(char) * buffer.size_of_the_packet);
    size_of_the_packet = buffer.size_of_the_packet;
}

void eventBuffer::set_data(char* i_data, int i_size)
{
    memset(packet, 0, SIZE_OF_DATA);
    memcpy(packet, i_data, i_size);
    size_of_the_packet = i_size;
}

bool eventBuffer::write(yarp::os::ConnectionWriter& connection)
{
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_BLOB + BOTTLE_TAG_INT);
    connection.appendInt(2);        // four elements
    connection.appendInt(size_of_the_packet);
    int ceilSizeOfPacket = (size_of_the_packet+7)/8*8;   // the nearest multiple of 8 greater or equal to size_of_the_packet
    connection.appendBlock(packet,ceilSizeOfPacket);
    connection.convertTextMode();   // if connection is text-mode, convert!
    return true;

}

bool eventBuffer::read(yarp::os::ConnectionReader& connection)
{
    connection.convertTextMode();   // if connection is text-mode, convert!
    int tag = connection.expectInt();
    if (tag != BOTTLE_TAG_LIST+BOTTLE_TAG_BLOB+BOTTLE_TAG_INT)
        return false;
    int ct = connection.expectInt();
    if (ct!=2)
        return false;
    size_of_the_packet = connection.expectInt();
    int ceilSizeOfPacket = (size_of_the_packet+7)/8*8; // the nearest multiple of 8 greater or equal to size_of_the_packet
    memset(packet, 0, SIZE_OF_DATA);
    connection.expectBlock(packet, ceilSizeOfPacket);
    return true;
}

}

}
