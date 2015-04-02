
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>

#include <gsl/gsl_math.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/neuralNetworks.h>

#include <stdio.h>
#include <string>
#include <list>

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;


class managerThread : public RateThread
{
protected:
    ResourceFinder &rf;

    string name;
    string robot;
    string arm;
    string eye;

    PolyDriver   *drvHead;
    PolyDriver   *drvGazeCtrl;

    IEncoders    *head;
    IGazeControl *gazeCtrl;

    BufferedPort<Bottle> inportIMDTargetLeft;
    BufferedPort<Bottle> inportIMDTargetRight;
    Port                 cmdPort;

    struct item
    {
        double t;
        Vector p;
    };

    ff2LayNN_tansig_purelin armNet;
    //list<item> listL, listR;
    deque<item> listL, listR;

    void cmdFromStereo(const Vector &pl, const Vector &pr)
    {
        Vector h(6);
        head->getEncoders(h.data());

        Vector in(7);
        in[0]=pl[0];
        in[1]=pl[1];
        in[2]=pr[0];
        in[3]=pr[1];
        in[4]=h[3];
        in[5]=h[4];
        in[6]=h[5];

        Vector arm_out=armNet.predict(in);
        Vector arm_out_e(4);
        arm_out_e[0]=arm_out[0];
        arm_out_e[1]=arm_out[1];
        arm_out_e[2]=arm_out[2];
        arm_out_e[3]=1.0;

        Vector xeye(3),oeye(4);
        gazeCtrl->getHeadPose(xeye,oeye);

        Matrix T=axis2dcm(oeye);
        T(0,3)=xeye[0];
        T(1,3)=xeye[1];
        T(2,3)=xeye[2];

        Vector arm_out_r=T*arm_out_e;

        // safe thresholding
        if (arm_out_r[0]>-0.15)
            arm_out_r[0]=-0.15;

        Bottle cmdBot;
        cmdBot.addVocab(VOCAB4('t','a','k','e'));
        cmdBot.addDouble(arm_out_r[0]);
        cmdBot.addDouble(arm_out_r[1]);
        cmdBot.addDouble(arm_out_r[2]);

        cmdPort.write(cmdBot);

        
        
        fprintf(stdout,"Thread is now suspended.\n");
        this->suspend();
    }

    void close()
    {
        if (drvHead)
            delete drvHead;

        if (drvGazeCtrl)
        {
            delete drvGazeCtrl;
        }

        inportIMDTargetLeft.interrupt();
        inportIMDTargetLeft.close();

        inportIMDTargetRight.interrupt();
        inportIMDTargetRight.close();

        cmdPort.interrupt();
        cmdPort.close();
    }

public:
    managerThread(ResourceFinder &_rf) : RateThread(20), rf(_rf)
    {   
        drvHead=NULL;     
        drvGazeCtrl=NULL;        
    }

    virtual bool threadInit()
    {
        // general part
        name=rf.check("name",Value("poeticon/reacher")).asString().c_str();
        robot=rf.check("robot",Value("icub")).asString().c_str();
        arm=rf.check("arm",Value("right_arm")).asString().c_str();
        eye=rf.check("eye",Value("right")).asString().c_str();
        setRate(rf.check("period",Value(20)).asInt());

        // init network
        Property armNetOptions,gazeNetOptions;
        armNetOptions.fromConfigFile(rf.findFile("arm_network").c_str()); 

        if (!armNet.configure(armNetOptions))
            return false;

        fprintf(stdout,"\nArm Network:\n");
        armNet.printStructure();

        // open ports
        inportIMDTargetLeft.open(("/"+name+"/left/blobs:i").c_str());
        inportIMDTargetRight.open(("/"+name+"/right/blobs:i").c_str());
        cmdPort.open(("/"+name+"/cmd:o").c_str());

        // open cartesiancontrollerclient and gazecontrollerclient drivers
        Property optHead("(device remote_controlboard)");
        Property optGazeCtrl("(device gazecontrollerclient)");

        optHead.put("remote",("/"+robot+"/head").c_str());
        optHead.put("local",("/"+name+"/head").c_str());

        optGazeCtrl.put("remote","/iKinGazeCtrl");
        optGazeCtrl.put("local",("/"+name+"/gaze").c_str());

        drvHead=new PolyDriver;
        if (!drvHead->open(optHead))
        {
            close();
            return false;
        }            

        drvGazeCtrl=new PolyDriver;
        if (!drvGazeCtrl->open(optGazeCtrl))
        {
            close();
            return false;
        }

        // open views
        drvHead->view(head);
        drvGazeCtrl->view(gazeCtrl);

        return true;
    }

    virtual void run()
    {
        double t=Time::now();
        Bottle *bL=inportIMDTargetLeft.read(false);
        Bottle *bR=inportIMDTargetRight.read(false);

        if (bL!=NULL)
        {
            Vector p(2);
            p[0]=bL->get(0).asList()->get(0).asInt();
            p[1]=bL->get(0).asList()->get(1).asInt();
            item i;
            i.t=t;
            i.p=p;
            listL.push_back(i);
        }

        if (bR!=NULL)
        {
            Vector p(2);
            p[0]=bR->get(0).asList()->get(0).asInt();
            p[1]=bR->get(0).asList()->get(1).asInt();
            item i;
            i.t=t;
            i.p=p;
            listR.push_back(i);
        }

        if ((listL.size()>5) && (listR.size()>5))
        {
            if ((listL.size()>10) && (listR.size()>10))
            {                
                Vector pl(2), pr(2),stdl(2),stdr(2);
                pl=pr=stdl=stdr=0.0;

                for (unsigned int i=0; i<listL.size(); i++)
                {
                    pl=pl+listL[i].p;
                    stdl[0]+=listL[i].p[0]*listL[i].p[0];
                    stdl[1]+=listL[i].p[1]*listL[i].p[1];
                }
                
                for (unsigned int i=0; i<listR.size(); i++)
                {
                    pr=pr+listR[i].p;
                    stdr[0]+=listR[i].p[0]*listR[i].p[0];
                    stdr[1]+=listR[i].p[1]*listR[i].p[1];
                }

                pl=(1.0/listL.size())*pl;
                pr=(1.0/listR.size())*pr;

                stdl[0]=sqrt((1.0/listL.size())*stdl[0] - pl[0]*pl[0]);
                stdl[1]=sqrt((1.0/listL.size())*stdl[1] - pl[1]*pl[1]);
                stdr[0]=sqrt((1.0/listR.size())*stdr[0] - pr[0]*pr[0]);
                stdr[1]=sqrt((1.0/listR.size())*stdr[1] - pr[1]*pr[1]);


                
                if (norm(stdl)<7.0 && norm(stdr)<7.0)
                {
                    cmdFromStereo(pl,pr);
                }

                listL.clear();
                listR.clear();
            }
        }
    }

    virtual void threadRelease()
    {
        close();
    }
};


class managerModule: public RFModule
{
protected:
    managerThread *thr;    
    Port           rpcPort;

public:
    managerModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        string name=rf.check("name",Value("poeticon/reacher")).asString().c_str();
        setName(("/"+name).c_str());

        thr=new managerThread(rf);
        if (!thr->start())
        {
            delete thr;    
            return false;
        }

        rpcPort.open(getName("/rpc"));
        attach(rpcPort);

        fprintf(stdout,"Thread is now suspended.\n");
        thr->suspend();
        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        if (command.size())
        {
            switch (command.get(0).asVocab())
            {
                case VOCAB4('s','u','s','p'):
                {                    
                    thr->suspend();
                    return true;
                }
        
                case VOCAB3('r','u','n'):
                {
                    thr->resume();
                    Time::delay(30.0);
                    reply.addString("Deployment achieved!");
                    return true;
                }
        
                default:
                    return RFModule::respond(command,reply);
            }
        }
        else
            return false;
    }

    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        thr->stop();
        delete thr;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)
    
    ResourceFinder rf;
    rf.setDefault("arm_network","arm_network.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    managerModule mod;
    

    return mod.runModule(rf);
}



