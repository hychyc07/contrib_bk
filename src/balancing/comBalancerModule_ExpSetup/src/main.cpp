#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <string.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "comBalancing.h"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace std;

class Balancer: public RFModule
{
private:
    //Module parameters and class variables.
    Property OptionsTorso;
    Property OptionsLeftLeg;
    Property OptionsRightLeg;
    PolyDriver *dd_torso;
    PolyDriver *dd_rightLeg;
    PolyDriver *dd_leftLeg;

    comBalancingThread *balThread;
    Port rpcPort;

    int nj;
    double *angle;
    Vector command;
    Vector command_RL;
    Vector command_LL;
public:
    Balancer()
    {
        dd_torso = 0;
        dd_rightLeg = 0;
        dd_leftLeg = 0;
    }

    bool configure(ResourceFinder &rf)
    { 
        //----------------------------------- LOCAL NAME --------------------------------------------------
        string local_name = "balanceModule";
        if (rf.check("local"))
        {
            local_name = rf.find("local").asString();
        }

        string wbs_name = "/wholeBodyDynamics";
        if (rf.check("local_wbs"))
        {
            wbs_name = rf.find("local_wbs").asString();
        }
        
        //Another way to get local module name parameter:
        // local_name = rf.check("local", Value("balanceModule"), "module name (string)").asString;
        // setName(local_name.c_str());             //setting the read local_name as module name

        //--------------------------------- SOME RPC PORT --------------------------------------------------
        string rpcPortName = "/"+local_name+"/rpc:i";
        rpcPort.open(rpcPortName.c_str());
        attach(rpcPort);

        //--------------------------------GETTING ROBOT NAME------------------------------------------------
        string robot_name;
        if (rf.check("robot"))
            robot_name = rf.find("robot").asString();
        else
            robot_name = "icub";

        //Another way to do this:
        //string robot_name = rf.check("robot", Value ("icub"), "Robot name (string)").asString();
        //string robotPortName = "/" +robot_name+ "/head";        //robotPortName can be used later on to create ports

        //----------------------------------GETTING HEAD TYPE ---------------------------------------------
        string head_type;
        if (rf.check("headV2"))
        {
            fprintf(stderr, "Using head Version 2\n");
            head_type = "V2";
        }

        //------------------------Displaying ZMP / Controlling Real Robot----------------------------------
        bool display = false;
        if (rf.check("display_zmp"))
        {
            fprintf(stderr, "Not Controlling real robot. Displaying ZMP on iCub_SIM\n");
            display=true;
        }        
        //----------------------------------- RATE --------------------------------------------------------
        int rate=10; //ms
        if (rf.check("rate"))
        {
            rate = rf.find("rate").asInt();
            fprintf(stderr,"rateThread working at %d ms\n",rate);            
        }
        else
        {
            fprintf(stderr,"Rate not specified. Using 10ms as default\n");            
            rate = 10;
        }

        //---------------------------------Ankles Sensors--------------------------------------------------
        bool ankles_sens = false;
        if (rf.check("ankles_sens"))
        {
            ankles_sens = true;
            fprintf(stderr, "Using ankles F/T sensors to compute COP\n");
        }
        else
        {
            fprintf(stderr, "Using Legs F/T sensors to compute COP\n");
        }


        //------------------------------- CREATING DEVICES ------------------------------------------------
        
        //Torso Device
        OptionsTorso.put("device","remote_controlboard");
        OptionsTorso.put("local",string("/"+local_name+"/torso/client").c_str());
        OptionsTorso.put("remote",string("/"+robot_name+"/torso").c_str());
        
        if(!CreateDevice(dd_torso, OptionsTorso))
        {
            fprintf(stderr,"ERROR: unable to create torso device\n ");
            return false;
        }
        else
            fprintf(stderr, "torso device driver created\n" );

        //Right leg device
        OptionsRightLeg.put("device","remote_controlboard");
        OptionsRightLeg.put("local",string("/"+local_name+"/right_leg/client").c_str());
        OptionsRightLeg.put("remote",string("/"+robot_name+"/right_leg").c_str());

        if(!CreateDevice(dd_rightLeg, OptionsRightLeg))
        {
            fprintf(stderr, "ERROR: unable to create right leg device\n");
            return false;
        }
        else
            fprintf(stderr, "right leg device driver created\n");

        //Left leg device
        OptionsLeftLeg.put("device","remote_controlboard");
        OptionsLeftLeg.put("local",string("/"+local_name+"/left_leg/client").c_str());
        OptionsLeftLeg.put("remote",string("/"+robot_name+"/left_leg").c_str());

        if(!CreateDevice(dd_leftLeg, OptionsLeftLeg))
        {
            fprintf(stderr, "ERROR: unable to create left leg device\n");
            return false;
        }
            fprintf(stderr, "left leg device driver created\n");

        //-------------------------- THREAD ---------------------------------------------------------------
        balThread  = new comBalancingThread(rate,dd_torso,dd_rightLeg,dd_leftLeg,robot_name,local_name,wbs_name,head_type,display,ankles_sens);
        fprintf(stderr, "Thread INSTANTIATED!\n");
        Time::delay(5.0);

        balThread->start();
        fprintf(stderr, "Thread STARTED!\n");
        return true;

    }

    virtual bool CreateDevice(PolyDriver *&_dd, Property options)
    {
        int trials=0;
        double start_time = yarp::os::Time::now();

        do
        {
            double current_time = yarp::os::Time::now();

            //remove previously existing drivers
            if (_dd)
            {
                delete _dd;
                _dd=0;
            }

            //creates the new device driver
            _dd = new PolyDriver(options);
            bool connected =_dd->isValid();

            //check if the driver is connected
            if (connected) break;
        
            //check if the timeout (60s) is expired
            if (current_time-start_time > 60.0)
            {
                fprintf(stderr,"It is not possible to instantiate the device driver. I tried %d times!\n", trials);
                return false;
            }

            yarp::os::Time::delay(5);
            trials++;
            fprintf(stderr,"\nUnable to connect the device driver, trying again...\n");
        }
        while (true);

        return true;
    }

    bool close()
    { //Close and shut down the module.

        //------ STOPPING THE THREAD ----------------------
        fprintf(stderr, "Closing balancerModule...\n");
        balThread->stop();
        fprintf(stderr, "comBalancingThread stopped\n");
        delete balThread;
        balThread=0;

        //------- PURGING DRIVERS ---------------------------
        if (dd_torso)
        {
            fprintf(stderr, "Closing Torso driver\n");
            dd_torso->close();
            dd_torso = 0;
        }

        if (dd_rightLeg)
        {
            fprintf(stderr, "Closing Right leg driver\n");
            dd_rightLeg->close();
            dd_rightLeg = 0;
        }

        if (dd_leftLeg)
        {
            fprintf(stderr, "Closing left leg driver\n");
            dd_leftLeg->close();
            dd_leftLeg = 0;
        }

        fprintf(stderr, "balancerModule was closed succesfully!\n");
        return true;
    }
    double getPeriod()
    {
        return 1;
    }

    bool updateModule()
    {
        fprintf(stderr, "I'm running!!\n");
        double avgTime, stdDev, period;
        period = balThread->getRate();
        balThread->getEstPeriod(avgTime,stdDev);
        if(avgTime > 1.3*period){
            printf("(real period: %3.3f +/- %3.3f. Expected %3.3f )\n",avgTime, stdDev, period);
        }

        double avgTime2, stdDev2;
        balThread->getEstUsed(avgTime2,stdDev2);
        fprintf(stderr,"Prob this is real run() time: %3.3f +/- %3.3f \n",avgTime2,stdDev2);
        return true;
    }



};

int main (int argc, char * argv[])
{
    // we need this to initialize the drivers list for cartesian controller
    // YARP_REGISTER_DEVICES(icubmod)

    //Creating and preparing the Resource Finder
    ResourceFinder rf;
    rf.setVerbose(true);
    // rf.setDefaultConfigFile(balancerModule.ini);
    // rf.setDefaultContext("BalancerModule/conf");
    rf.configure("ICUB_ROOT",argc,argv);
    // rf.setName("balancerModule");

    if (rf.check("help"))
    {
        cout<< "Possible parameters" << endl << endl;
        cout<< "\t--context        :Where to find a user defined environment (.ini file) within $ICUB_ROOT/app"    <<endl;
        cout<< "\t--from           :Name of the file.ini to be used for calibration."                                <<endl;
        cout<< "\t--rate           :Period used by the module. Default set to 10ms."                                <<endl;
        cout<< "\t--robot          :Robot name (icubSim or icub). Set to icub by default."                            <<endl;
        cout<< "\t--local          :Prefix of the ports opened by the module. Set to the module name by default, i.e. balancerModule " <<endl;
        cout<< "\t--headV2         :Specifying head model to be used. Set to V2 for black icub and following versions."<<endl;
        cout<< "\t--display_zmp    :Flag to decide whether to control the real robot or to only display computed zmp on the simulator. "<<endl;
        return 0;
    }
    
    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr,"Sorry YARP network is not available\n");
        return -1;
    }

    //Creating the module
    Balancer balancerModule;    

    //Running the module: runModule() calls configure first and, if succesful, it then runs. 
    return balancerModule.runModule(rf);
}
