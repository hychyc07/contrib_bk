// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <string>
#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <termios.h>
#include <unistd.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

int mygetch() {
  struct termios oldt,
                 newt;
  int            ch;
  tcgetattr( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~( ICANON | ECHO );
  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
  return ch;
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
    std::string remotePorts="/";
    remotePorts+=robotName;
    remotePorts+="/left_arm";

    std::string localPorts="/test/client";

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());   //local port names
    options.put("remote", remotePorts.c_str());         //where we connect to

    // create a device
    PolyDriver robotDevice(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    IPositionControl *pos;
    IEncoders *encs;

    bool ok;
    ok = robotDevice.view(pos);
    ok = ok && robotDevice.view(encs);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }

    int nj=0;
    pos->getAxes(&nj);
    Vector encoders;
    Vector command;
    Vector tmp;
    encoders.resize(nj);
    tmp.resize(nj);
    command.resize(nj);
    
    int i;
    for (i = 0; i < nj; i++) {
         tmp[i] = 50.0;
    }
    pos->setRefAccelerations(tmp.data());

    for (i = 0; i < nj; i++) {
        tmp[i] = 4.0;
        pos->setRefSpeed(i, tmp[i]);
    }

    //pos->setRefSpeeds(tmp.data()))
    
    //fisrst zero all joints
    //
    command=0;
    //now set the shoulder to some value
    command[0]=-26.0;
    command[1]=20.0;
    command[2]=0.0;
    command[3]=49.0;
    command[4]=0.0;

    pos->positionMove(command.data());
    
    bool done=false;
    while(!done) {
        Time::delay(0.5);
        pos->checkMotionDone(&done);
    }

    printf("\niCub @ HOME. Press any key...");
    mygetch();

    int times=0;
    while(true)
    {
        times++;
        if (times%2)
        {
             printf("\n\nSet pos1: ");
             //command[0]=-50;
             command[1]=64.0;
             //command[2]=-10;
             //command[3]=50;
             //command[4]=0;
        }
        else
        {
             printf("\n\nSet pos2: ");
             //command[0]=-20;
             command[1]=20.0;
             //command[2]=-10;
             //command[3]=30;
             //command[4]=0;
        }

        pos->positionMove(command.data());

        printf("waiting");
        bool done3=false;
        while(!done3) {
            Time::delay(1.0);
            pos->checkMotionDone(&done3);
            printf(".");
        }
            printf("ok!\n");
    mygetch();

    }


    robotDevice.close();
    
    return 0;
}
