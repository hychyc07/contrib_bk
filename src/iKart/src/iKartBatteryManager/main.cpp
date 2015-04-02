/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/** 
\defgroup iKartBattery iKartBattery
 
@ingroup icub_module  
 
iKart battery manager.
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Marco Randazzo

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
@@@TODO
 
\section portsa_sec Ports Accessed
 
@@@TODO
 
\section portsc_sec Ports Created 
 
@@@TODO

\section in_files_sec Input Data Files

@@@TODO

\section out_data_sec Output Data Files 

@@@TODO
 
\section conf_file_sec Configuration Files

@@@TODO

\section tested_os_sec Tested OS
Windows, Linux

\author Marco Randazzo 
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/SerialInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <cstring>


YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

struct struct_battery_data
{
    int count;
    int raw_voltage;
    int raw_current;
    int raw_charge;
    int raw_checksum;
    
    double voltage;
    double current;
    double charge;
    char* timestamp;
};

class CtrlThread: public RateThread
{

private:         
    bool                logEnable;
    bool                verboseEnable;
    bool                screenEnable;
    bool                debugEnable;
    bool                shutdownEnable;
    char                log_buffer[255];
    FILE                *logFile;
    bool                yarp_found;
    bool                first_reading;
    Network             yarp;

protected:
    ResourceFinder      &rf;
    PolyDriver          driver;
    ISerialDevice       *pSerial;
    char                serial_buff[255];
    Port                port_battery_output;
    Port                port_shutdown;

    string remoteName;
    string localName;

    struct_battery_data battery_data;

public:
    CtrlThread(unsigned int _period, ResourceFinder &_rf,
               string _remoteName, string _localName) :
               RateThread(_period),     rf(_rf),
               remoteName(_remoteName), localName(_localName) 
    {
        //yarp.setVerbosity(-1);
        logEnable=false;
        shutdownEnable=true;
        first_reading=false;
        for (int i=0; i<255; i++) serial_buff[i]=0;

        time_t rawtime;
        struct tm * timeinfo;
        time ( &rawtime );
        timeinfo = localtime ( &rawtime );
        battery_data.timestamp=asctime (timeinfo);
    }

    void check_battery_status()
    {
        static bool notify_15=true;
        static bool notify_12=true;
        static bool notify_10=true;
        static bool notify_0 =true;

        if (battery_data.charge > 20)
        {
            notify_15 = true;
            notify_12 = true;
            notify_10 = true;
            notify_0  = true;
        }

        if (battery_data.charge < 15)
        {
            if (notify_15) {notify_message ("WARNING: battery charge below 15%"); notify_15=false;}
        }
        if (battery_data.charge < 12)
        {
            if (notify_12) {notify_message ("WARNING: battery charge below 12%"); notify_12=false;}
        }
        if (battery_data.charge < 10)
        {
            if (notify_10) {notify_message ("WARNING: battery charge below 10%"); notify_10=false;}
        }
        if (battery_data.charge < 5)
        {
            if (notify_0)
            {
                if (shutdownEnable)
                {
                    emergency_shutdown ("CRITICAL WARNING: battery charge below critical level 5%. The robot will be stopped and the system will shutdown in 2mins.");
                    stop_robot("/icub/quit");
                    stop_robot("/ikart/quit");
                    notify_0=false;
                }
                else
                {
                    notify_message ("CRITICAL WARNING: battery charge reached critical level 5%, but the emergency shutodown is currently disabled!");
                    notify_0=false;
                }
            }
        }
    }

    virtual bool threadInit()
    {
        //user options
        logEnable=rf.check("logToFile");
        verboseEnable=rf.check("verbose");
        screenEnable=rf.check("screen");
        debugEnable=rf.check("debug");
        shutdownEnable=(!rf.check("noShutdown"));

        //serial port configuration parameters
        rf.setDefaultContext("iKart/conf");
        rf.setDefaultConfigFile("batterySerialPort.ini");
        ConstString configFile=rf.findFile("from");
        Property prop;
        prop.fromConfigFile(configFile.c_str());
        prop.put("device","serialport");
        printf("\nSerial port configuration:\n%s \n\n",prop.toString().c_str());

        if (logEnable)
        {
            fprintf(stderr, "writing to log file batteryLog.txt\n");
            logFile = fopen("batteryLog.txt","w");
        }

        //open serial port driver
        driver.open(prop);
        if (!driver.isValid())
        {
            fprintf(stderr, "Error opening PolyDriver check parameters\n");
            return false;
        }
        driver.view(pSerial);
            
        if (!pSerial)
        {
            fprintf(stderr, "Error opening serial driver. Device not available\n");
            return false;
        }
        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            fprintf(stderr, "Thread started successfully\n");
        else
            fprintf(stderr, "Thread did not start\n");
    }

    void notify_message(string msg)
    {
    #ifdef WIN32
        fprintf(stderr,"%s", msg.c_str());
    #else
        fprintf(stderr,"%s", msg.c_str());
        string cmd = "echo "+msg+" | wall";
        system(cmd.c_str());
    #endif
    }

    void emergency_shutdown(string msg)
    {
    #ifdef WIN32
        string cmd;
        cmd = "shutdown /s /t 120 /c "+msg;
        fprintf(stderr,"%s", msg.c_str());
        system(cmd.c_str());
    #else
        string cmd;
        fprintf(stderr,"%s", msg.c_str());
        cmd = "echo "+msg+" | wall";
        system(cmd.c_str());

        cmd = "sudo shutdown -h 2 "+msg;
        system(cmd.c_str());

        cmd = "ssh icub@pc104 sudo shutdown -h 2";
        system(cmd.c_str());
    #endif
    }

    void stop_robot(string quit_port)
    {
        //typical quit_port:
        // "/icub/quit"
        // "/ikart/quit"
        if (yarp_found)
        {
            port_shutdown.open((localName+"/shutdown").c_str());
            yarp.connect((localName+"/shutdown").c_str(),quit_port.c_str());
            Bottle bot;
            bot.addString("quit");
            port_shutdown.write(bot);
            port_shutdown.interrupt();
            port_shutdown.close();
        }
    }

    void print_battery_status()
    {
        fprintf(stdout,"%s", log_buffer);
    }

    bool verify_checksum(struct_battery_data& b)
    {
        if (b.raw_checksum==b.raw_voltage+b.raw_current+b.raw_charge)
            return true;
        return false;
    }

    virtual void run()
    {
        //network checks
        //is yarp server available?
        yarp_found = yarp.checkNetwork();
        if (yarp_found)
        {
            //is output port already open? if not, open it
            if (!yarp.exists((localName+"/battery:o").c_str()))
                port_battery_output.open((localName+"/battery:o").c_str());

        }
        //read battery data.
        //if nothing is received, rec=0, the while exits immediately. The string will be not valid, so the parser will skip it and it will leave unchanged the battery status (voltage/current/charge)
        //if a text line is received, then try to receive more text to empty the buffer. If nothing else is received, serial_buff will be left unchanged from the previous value. The loop will exit and the sting will be parsed.
        serial_buff[0]=0;
        int rec = 0;
        do
        {
            rec = pSerial->receiveLine(serial_buff,250);
            if (verboseEnable) fprintf(stderr,"%d ", rec);
            if (debugEnable) fprintf(stderr,"<%s> ", serial_buff);
        }
        while
            (rec>0);
        if (verboseEnable) fprintf(stderr,"\n");

        int len = strlen(serial_buff);
        if (len>0)
        {
            if (verboseEnable)
                fprintf(stderr,"%s", serial_buff);   
        
            int pars = 0;
            pars = sscanf (serial_buff, "%*s %d %*s %d %*s %d %*s %d", &battery_data.raw_current, &battery_data.raw_voltage,&battery_data.raw_charge,&battery_data.raw_checksum);

            if (pars == 4)
            {
                if (verify_checksum(battery_data))
                {
                    time_t rawtime;
                    struct tm * timeinfo;
                    time ( &rawtime );
                    timeinfo = localtime ( &rawtime );
                    battery_data.timestamp=asctime (timeinfo);
                    battery_data.voltage = double(battery_data.raw_voltage)/1024 * 66;
                    battery_data.current = (double(battery_data.raw_current)-512)/128 *20; //+- 60 is the maximum current that the sensor can read. 128+512 is the value of the AD 
                                                                                           //when the current is 20A.
                    battery_data.charge =  double(battery_data.raw_charge)/100; // the value coming from the BCS board goes from 0 to 100%
                    sprintf(log_buffer,"battery status: %+6.1fA   % 6.1fV   charge:% 6.1f%%    time: %s", battery_data.current,battery_data.voltage,battery_data.charge, battery_data.timestamp);
                    first_reading = true;
                }
                else
                {
                    fprintf(stderr,"checksum error while reading battery data\n");
                }
            }
            else
            {
                fprintf(stderr,"error reading battery data: %d\n", pars);
            }
        }
        //send data to yarp output port (if available)
        if (yarp_found)
        {
            Bottle bot; 
            bot.addString("count");
            bot.addInt(battery_data.count);
            bot.addString("voltage");
            bot.addDouble(battery_data.voltage);
            bot.addString("current");
            bot.addDouble(battery_data.current);
            bot.addString("charge");
            bot.addDouble(battery_data.charge);
            bot.addString("time");
            bot.addString(battery_data.timestamp);
            port_battery_output.write(bot);
        }

        // if the battery is not charging, checks its status of charge
        if (first_reading && battery_data.current>0.4) check_battery_status();

        // print data to screen
        if (screenEnable)
        {
            fprintf(stderr,"%s", log_buffer);
        }
        // save data to file
        if (logEnable)
        {
            fprintf(logFile,"%s", log_buffer);
        }

    }

    virtual void threadRelease()
    {    
        fprintf(stdout,"closing ports...\n");
        port_battery_output.interrupt();
        port_battery_output.close();

        //close log file
        if (logEnable)
        {
            fprintf(stdout,"closing logfile...\n");
            fclose(logFile);
        }
    }

};



class CtrlModule: public RFModule
{
protected:
    CtrlThread *thr;
    //Port        rpcPort;

public:
    CtrlModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        string slash="/";
        string ctrlName;
        string robotName;
        string partName;
        string remoteName;
        string localName;

        Time::turboBoost();

        // get params from the RF
        ctrlName=rf.check("ctrlName",Value("ikart")).asString();
        robotName=rf.check("robot",Value("ikart")).asString();

        remoteName=slash+robotName+"/wheels";
        localName=slash+ctrlName;//+"/local/";

        thr=new CtrlThread(10000,rf,remoteName,localName);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        //rpcPort.open((localName+"/rpc").c_str());
        //attach(rpcPort);

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;
        return true;
    }

    virtual double getPeriod()    { if (thr) thr->print_battery_status(); return 60.0;  }
    virtual bool   updateModule() { return true; }
};



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "--verbose:     show the received raw data from the battery managment board"<< endl;
        cout << "--debug:       show advanced debug information"<< endl; 
        cout << "--screen:      show measurements on screen"<< endl;
        cout << "--logToFile:   save the mesurments to file: batteryLog.txt"<< endl;
        cout << "--noShutdown:  do not not shutdown even if the battery reaches the critical level"<< endl;
        return 0;
    }

    YARP_REGISTER_DEVICES(icubmod)

    CtrlModule mod;

    return mod.runModule(rf);
}



