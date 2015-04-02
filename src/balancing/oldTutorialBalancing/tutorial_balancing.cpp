// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

bool openRemoteControlBoard(PolyDriver *&d, IPositionControl *pos, IEncoders *encs, std::string boardToBeOpened, std::string robotName)
{

    std::string remotePorts="/";
    remotePorts+=robotName;
    remotePorts+="/";
    remotePorts+=boardToBeOpened;

    std::string localPorts="/test/client/" + boardToBeOpened;

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());   //local port names
    options.put("remote", remotePorts.c_str());         //where we connect to

    // create a device
    d = new PolyDriver(options);

    if (!d->isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    return 1;
}

int main(int argc, char *argv[]) 
{
    Network yarp;

    Property params;
    params.fromCommand(argc, argv);

    if (!params.check("robot"))
        {
            fprintf(stderr, "Please specify the name of the robot\n");
            fprintf(stderr, "--robot name (e.g. icub)\n");
            return -1;
        }
    std::string robotName=params.find("robot").asString().c_str();

    IPositionControl *rpos;
    IEncoders *rencs;
    PolyDriver *rightDevice;
    if (!openRemoteControlBoard(rightDevice, rpos, rencs, "right_leg", robotName))
        return 0;

    bool ok;
    ok = rightDevice->view(rpos);
    ok = ok && rightDevice->view(rencs);
    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }
    else
        printf("Interface were ok!\n");

    int nj=0;
    rpos->getAxes(&nj);
    fprintf(stderr, "Number of axes is: %d", nj);
    Vector encoders;
    Vector command;
    Vector tmp;
    encoders.resize(nj);
    tmp.resize(nj);
    command.resize(nj);

    IPositionControl *lpos;
    IEncoders *lencs;
    PolyDriver *leftDevice;
    if (!openRemoteControlBoard(leftDevice, lpos, lencs, "left_leg", robotName))
        return 0;

    ok = leftDevice->view(lpos);
    ok = ok && leftDevice->view(lencs);
    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }
    else
        printf("Interface were ok!\n");
    
    int i;
    for (i = 0; i < nj; i++) {
         tmp[i] = 50.0;
    }
    rpos->setRefAccelerations(tmp.data());
    lpos->setRefAccelerations(tmp.data());

    for (i = 0; i < nj; i++) {
        tmp[i] = 10.0;
        rpos->setRefSpeed(i, tmp[i]);
        lpos->setRefSpeed(i, tmp[i]);
    }

    //rpos->setRefSpeeds(tmp.data()))
    
    //fisrst zero all joints
    //
    command=0;
    //now set the shoulder to some value
    command[0]=0;
    command[1]=0;
    command[2]=0;
    command[3]=0;
    command[4]=0;
    command[5]=0;
    // rpos->positionMove(command.data());
    
    bool done=false;

    while(!done)
    {
        rpos->checkMotionDone(&done);
        lpos->setRefSpeed(i, tmp[i]);
        Time::delay(0.1);
    }

    int times=0;
    while(true)
    {
        times++;
        if (times%2)
        {
             command[1]= 3.0;
             command[0]= 0.0;
             command[2]= 0.0;
             command[3]= 0.0;
             command[4]= 0.0;
             command[5]=-3.0;
             rpos->positionMove(command.data());
             command[1]= 9.0;
             command[0]= 0.0;
             command[2]= 0.0;
             command[3]= 0.0;
             command[4]= 0.0;
             command[5]=-9.0;
             lpos->positionMove(command.data());
        }
        else
        {
             command[1]= 9.0;
             command[0]= 0.0;
             command[2]= 0.0;
             command[3]= 0.0;
             command[4]= 0.0;
             command[5]=-9.0;
             rpos->positionMove(command.data());
             command[1]= 3.0;
             command[0]= 0.0;
             command[2]= 0.0;
             command[3]= 0.0;
             command[4]= 0.0;
             command[5]=-3.0;
             lpos->positionMove(command.data());
        }

        fprintf(stderr, "Sending the position command! %s\n", command.toString().c_str());



        int count=50;
        while(count--)
            {
                Time::delay(0.1);
                rencs->getEncoders(encoders.data());
                printf("Right: %.1lf %.1lf %.1lf %.1lf %.1lf %.1lf \n", encoders[0], encoders[1], encoders[2], encoders[3], encoders[4], encoders[5]);
                lencs->getEncoders(encoders.data());
                printf("Left : %.1lf %.1lf %.1lf %.1lf %.1lf %.1lf \n", encoders[0], encoders[1], encoders[2], encoders[3], encoders[4], encoders[5]);
            }
    }

    leftDevice->close();
    rightDevice->close();
    delete leftDevice;
    delete rightDevice;

    
    return 0;
}
