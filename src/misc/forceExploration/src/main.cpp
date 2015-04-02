/******************** connessioni di porte *************************
yarp connect /icub/left_arm/analog:o /forceExploration/left_arm/FT:i
yarp connect /icub/right_arm/analog:o /forceExploration/right_arm/FT:i
**********************************************************************/

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynTransform.h>

#include <iostream>
#include <iomanip>
#include <string.h>
#include <fstream>

#include <gsl/gsl_math.h>

#include "armExploration.h"
#include "wrenchComputation.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;

using namespace std;

YARP_DECLARE_DEVICES(icubmod)

//******************************************************************************
//******************************************************************************
class forceExploration: public RFModule
{
private:
    Property OptionsLimb;
    Property OptionsTorso;
    Property OptionsExpl;

    wrenchComputation *iwCom;
    armExploration *iaExp;
    
    Vector FT;

    PolyDriver *dd;
    PolyDriver *tt;
    PolyDriver *cc;

public:
    forceExploration()
    {
        iwCom=0;
        //dd=0;
        //tt=0;
    }

    virtual bool createDriver(PolyDriver *_dd)
    {
        if (!_dd || !(_dd->isValid()))
        {
            fprintf(stderr,"It is not possible to instantiate the device driver\nreturning...");
            return 0;
        }

        IEncoders *encs;

        bool ok = true;
        ok = ok & _dd->view(encs);
        if (!ok)
        {
            fprintf(stderr,"ERROR: one or more devices has not been viewed\nreturning...");
            return false;
        }

        return true;
    }

    bool configure(ResourceFinder &rf)
    {
        string local;
        string part="right_arm";
        string robot="icub";
        string fwdSlash = "/";
        
        bool v=1;

        int ia_rate = 50;
        int iw_rate = 100;

        string name;
        if (rf.check("name"))
            name = rf.find("name").asString();
        else name = "forceExploration";

        //---------------------ROBOT-----------------------------//
        ConstString robotName=rf.find("robot").asString();
        if (rf.check("robot"))
        {
            robot = rf.find("robot").asString().c_str();
        }
        else cout<<"Could not find robotname in the config file; using "<<robot<<" as default\n";
        
        //---------------------VERBOSE-----------------------------//
        ConstString verbose=rf.find("verbose").asString();
        if (rf.check("verbose"))
        {
            v = rf.find("verbose").asInt();
        }
        else cout<<"Could not find verbose option in the config file; using "<<v<<" as default\n";
        
        //---------------------PART-----------------------------//
        ConstString partName=rf.find("part").asString();
        if (rf.check("part"))
        {
            part = rf.find("part").asString().c_str();
        }
        else cout<<"Could not find part in the config file; using "<<part<<" as default\n";
        
        //---------------------ia_RATE-----------------------------//
        if (rf.check("ia_rate"))
        {
            ia_rate = rf.find("ia_rate").asInt();
            fprintf(stderr,"ArmExploration rateThread working at %d ms\n", ia_rate);
        }
        else cout<<"Could not find ia_rate in the config file; using "<<ia_rate<<"ms as default\n";
        
        //---------------------iw_RATE-----------------------------//
        if (rf.check("iw_rate"))
        {
            iw_rate = rf.find("iw_rate").asInt();
            fprintf(stderr,"WrenchComputation rateThread working at %d ms\n", iw_rate);
        }
        else cout<<"Could not find iw_rate in the config file; using "<<iw_rate<<"ms as default\n";
        
        FT.resize(6,0.0);


        //---------------------DEVICES--------------------------//
        if( iw_rate != 0 ) {
            OptionsTorso.put("robot",robot.c_str());
            OptionsTorso.put("part","torso");
            OptionsTorso.put("device","remote_controlboard");
            OptionsTorso.put("local",(fwdSlash+name+fwdSlash+part+"/torso/client").c_str());
            OptionsTorso.put("remote","/icub/torso");

            tt = new PolyDriver(OptionsTorso);

            if (!createDriver(tt)) {
                fprintf(stderr,"ERROR: unable to create torso device driver...quitting\n");
                return false;
            }
            else    fprintf(stderr,"torso device driver created\n");

            OptionsLimb.put("robot",robot.c_str());
            OptionsLimb.put("part",part.c_str());
            OptionsLimb.put("device","remote_controlboard");
            OptionsLimb.put("local",(fwdSlash+name+fwdSlash+part+"/client").c_str());
            OptionsLimb.put("remote",(fwdSlash+robot+fwdSlash+part).c_str());
            dd = new PolyDriver(OptionsLimb);
            if (!createDriver(dd))
            {
                fprintf(stderr,"ERROR: unable to create limb device driver...quitting\n");
                return false;
            }
            else    fprintf(stderr,"limb device driver created\n");
        }
        
        if( ia_rate != 0 ) {
            OptionsExpl.put("robot",robot.c_str());
            OptionsExpl.put("part",part.c_str());
            OptionsExpl.put("device","cartesiancontrollerclient");
            OptionsExpl.put("local",(fwdSlash+name+fwdSlash+part+"/cartesianclient").c_str());
            OptionsExpl.put("remote",(fwdSlash+robot+fwdSlash+"cartesianController"+fwdSlash+part).c_str());
            cc = new PolyDriver(OptionsExpl);
            /*if (!createDriver(cc))
            {
                fprintf(stderr,"ERROR: unable to create cartesian device driver...quitting\n");
                return false;
            }
            else*/    fprintf(stderr,"cartesian device driver created\n");
        }

        //--------------------------THREADS--------------------------
        if( iw_rate != 0 ) {
            if (part=="left_arm" || part=="right_arm") {
                iwCom = new wrenchComputation(iw_rate, dd, tt, part.c_str(), name.c_str(), FT);
                fprintf(stderr,"iwCom thread istantiated...\n");
                iwCom->start();
            }
            else {    
                fprintf(stderr,"unable to istantiate iwCom thread!!!");
                return false;
            }
        }
        else {
            iwCom = 0;
            dd = 0;
            tt = 0;
        }
        
        if( ia_rate != 0 ) {    
            if (part=="left_arm" || part=="right_arm") {
                iaExp = new armExploration(ia_rate, cc, part.c_str(), name.c_str(), FT, v);
                fprintf(stderr,"iaExp thread istantiated...\n");
                iaExp->start();
            }
            else {    
                fprintf(stderr,"unable to istantiate iaExp thread!!!");
                return false;
            }
        }
        else {
            iaExp = 0;
            cc = 0;
        }

        return true;
    }

    bool close()
    {
		if (iwCom)
        {
            delete iwCom;
            iwCom=0;
        }
        
		if (iaExp)
        {
            delete iaExp;
            iaExp=0;
        }

        if (dd)
        {
            delete dd;
            dd=0;
        }
        
        if (tt)
        {
            delete tt;
            tt=0;
        }
        
        if (cc)
        {
            delete cc;
            cc=0;
        }

        return true;
    }

    double getPeriod()  { return 1.0;  }
    bool updateModule() { 
        //cout<<"MODULE: FT= "<<FT.toString()<<endl;
        return true; }
};

//******************************************************************************
//******************************************************************************
int main(int argc, char * argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {        
        cout << "Options:" << endl << endl;
        //cout << "   --context\tcontext: where to find the called resource (referred to $ICUB_ROOT/app: default wrechObserver/conf)" << endl;
        cout << "   --from\tfrom: the name of the file.ini to be used for calibration" << endl;
        cout << "   --robot\trobot: the name of the robot. Default icub" << endl;
        cout << "   --part\tpart: the name of the arm (left_arm or right_arm). Default left_arm." << endl;
        cout << "   --ia_rate\trate: the period used by the armExploration thread. Default 50ms." << endl;
        cout << "   --iw_rate\trate: the period used by the wrenchComputation thread. Default 100ms." << endl;
        return 0;
    }

    YARP_REGISTER_DEVICES(icubmod)

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    forceExploration obs;

    return obs.runModule(rf);
}

