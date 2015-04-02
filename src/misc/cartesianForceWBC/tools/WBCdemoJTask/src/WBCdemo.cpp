
#include "iCub/WBCdemo/WBCdemo.h"

/************************************************************************/
WBCdemo::WBCdemo() { }

/************************************************************************/
bool WBCdemo::configure(ResourceFinder &rf) {

    qPort.setSharedArea(&mem);
    mPort.setWBCthread(&wThread);
    wThread.setSharedArea(&mem);

    qPort.open("/WBCdemo/q:i");
    mPort.open("/WBCdemo/mode:i");

    //-----------------CHECK ROBOT AND PART NAME------------//
    ConstString robot_name;
    if (rf.check("robot"))
        robot_name = rf.find("robot").asString();
    else robot_name = DEFAULT_ROBOT;
    ConstString part_name;
    if (rf.check("part"))
        part_name = rf.find("part").asString();
    else part_name = DEFAULT_PART;

    ConstString remotePorts="/";
    remotePorts+=robot_name+"/";
    remotePorts+=part_name;
    ConstString localPorts="/WBCdemo/";
    localPorts+=part_name;

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts);   //local port names
    options.put("remote", remotePorts);    //where we connect to
    robotDevice.open(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

/*    Property coption;
    coption.put("device","cartesiancontrollerclient");
    ConstString cremotePorts="/";
    cremotePorts+=robot_name+"/cartesianController/left_arm";
    coption.put("remote",cremotePorts);
    coption.put("local","/cartesian_client/left_arm");
    cartesianDevice.open(coption);
    if (!cartesianDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }*/

    bool ok;
    ok = robotDevice.view(pos);
    ok = ok && robotDevice.view(ictrl);
    ok = ok && robotDevice.view(itrq);
    //ok = ok && cartesianDevice.view(icart);
    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return false;
    }

    int axes;
    pos->getAxes(&axes);
    // do something with this?

    //-----------------CHECK IF AUTOCONNECT IS ON, CONNECT IF TRUE-----------//
    bool autoconnect;
    if (rf.check("autoconnect")) {
        autoconnect = true;
        fprintf(stderr,"Autoconnect option found. \n");
        ConstString bS("/");
        if(Network::connect(bS+robot_name+bS+part_name+"/state:o","/WBCdemo/q:i")) {
            mem.setQ(*qPort.read(true));
            printf("[success] connected to %s %s q.\n",robot_name.c_str(),part_name.c_str());
        } else {
            printf("[error] could not connect to %s %s q, aborting...\n",robot_name.c_str(),part_name.c_str());
            return false;
        }
    } else {
        autoconnect = false;
        fprintf(stderr,"Autoconnect option not found, disabled. \n");
    }

    //-----------------ACTIVATE THE CALLBACKS ALWAYS-----------//
    qPort.useCallback();
    mPort.useCallback();

    //-----------------GIVE THE WBC THREAD INTERFACE POINTERS AND INIT IT-----------//
    wThread.setIPositionControl(pos);
    wThread.setIControlMode(ictrl);
    wThread.setITorqueControl(itrq);
    wThread.init(rf);

    return true;
}

/*****************************************************************/
double WBCdemo::getPeriod() {  // Frec at which updateModule below gets called
        return 5.0;  // Hard-code slow thread to 5 s.
}

/************************************************************************/
bool WBCdemo::updateModule() {
//    printf("Alive\n");
    return true;
}

/************************************************************************/
bool WBCdemo::interruptModule() {
    qPort.disableCallback();
    mPort.disableCallback();
    qPort.interrupt();
    mPort.interrupt();
    wThread.askToStop();
    Time::delay(1.0); // so the wThread can set motors to position mode
    // cartesianDevice.close();
    robotDevice.close();
    qPort.close();
    mPort.close();
    return true;
}

