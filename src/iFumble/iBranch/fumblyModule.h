
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/Drivers.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <iCub/ctrl/math.h>
#include <iCub/action/actionPrimitives.h>
#include <iostream>

#define USE_LEFT    0
#define USE_RIGHT   1
#define HOMING_PERIOD 2

#define CMD_CALIB                   VOCAB4('c','a','l','i')
#define AFFACTIONPRIMITIVESLAYER    ActionPrimitivesLayer2

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::action;

//////////////////////////////////////////////////////
// this class handles the offset to be added to the
// current table heigth according to the feedback
// coming from contact detection
class tableManager
{
protected:
    string fileName;
    double heightOffset;

    string id;
    Vector position;

    string sender;
    string receiver;

public:
    tableManager()
    {
        fileName="";
        heightOffset=0.0;

        id="";
        position.resize(3,0.0);

        sender="";
        receiver="";
    }

    void initTxParams(const string &_sender, const string &_receiver)
    {
        sender=_sender;
        receiver=_receiver;
    }

    void load(const string &_fileName)
    {        
        fileName=_fileName;

        Property config;
        config.fromConfigFile(fileName.c_str());
        if (config.isNull())
            return;

        // parsing id option
        id=config.check("id",Value("")).asString().c_str();

        // parsing POSITION part
        Bottle &bPosition=config.findGroup("POSITION");
        if (bPosition.isNull())
            return;

        position[0]=bPosition.check("x",Value(0.0)).asDouble();
        position[1]=bPosition.check("y",Value(0.0)).asDouble();
        position[2]=bPosition.check("z",Value(0.0)).asDouble();

        heightOffset=0.0;
    }

    void save()
    {
        if (heightOffset)
        {
            ofstream fout;
            fout.open(fileName.c_str());

            // id option
            fout<<endl;
            fout<<"id\t"<<id<<endl;
            fout<<endl;

            // position part
            fout<<"[POSITION]"<<endl;
            fout<<"x\t"<<position[0]<<endl;
            fout<<"y\t"<<position[1]<<endl;
            fout<<"z\t"<<(position[2]+heightOffset)<<endl;

            fout.close();
        }
    }

    void setTableHeight(const double height)
    {
        heightOffset=height-position[2];

        if (heightOffset)
        {
            Port port;
            port.open(sender.c_str());

            if (Network::connect(sender.c_str(),receiver.c_str()))
            {
                Bottle cmd, reply;
                cmd.addString("set");
                cmd.addString("heightOffset");
                cmd.addDouble(heightOffset);
                   
                port.write(cmd,reply);
            }
            else
                cout<<"Warning: unable to connect to "<<receiver<<endl;

            port.close();
        }
    }

    ~tableManager()
    {
        save();
    }
};

//////////////////////////////////////////////////////


class fumblyModule: public RFModule
{
protected:
    string partUsed;

	AFFACTIONPRIMITIVESLAYER *actionL;
    AFFACTIONPRIMITIVESLAYER *actionR;
    AFFACTIONPRIMITIVESLAYER *action;
	BufferedPort<Bottle>      inPort;
    Port                      rpcPort;

    Vector graspOrienL, graspOrienR;
    Vector graspDispL,  graspDispR;
    Vector dOffsL,      dOffsR;
    Vector dLiftL,      dLiftR;
    Vector home_xL,     home_xR;

    Vector *graspOrien;
    Vector *graspDisp;
    Vector *dOffs;
    Vector *dLift;
    Vector *home_x;

    bool openPorts;
    bool firstRun;
    map<string,Matrix>            palmOrientations;
    string armToBeUsed;
    tableManager                  tableMan;

public:
    fumblyModule();
    void getArmDependentOptions(Bottle &b, Vector &_gOrien, Vector &_gDisp,
                                Vector &_dOffs, Vector &_dLift, Vector &_home_x);
    virtual bool configure(ResourceFinder &rf);
    virtual bool close();
    virtual double getPeriod();
    void useArm(const int arm);
    void init();
    virtual bool updateModule();
    virtual bool respond(const Bottle &command, Bottle &reply);
    bool interruptModule();
    void fiddle(const Vector &xd, double theta, const double &rStart,  double ftheta, const double &rStop, const double &execTime);
    void goHome();
    void computePalmOrientations();
    

};


