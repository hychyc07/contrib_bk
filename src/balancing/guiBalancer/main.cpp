
#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Network.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/ctrl/math.h>

using namespace yarp::math;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

using namespace std;
using namespace yarp::os;

const int THREAD_RATE=20; //ms

class PlotGuiClass: public RateThread
{
private:
    BufferedPort<Vector> *port_in_zmp;      //Port for ZMP Coordinates
    BufferedPort<Vector> *port_in_com;      //Port for COM Coordinates      
    BufferedPort<Bottle> *port_to_iCubGui;  //Port to send coordinates to iCubGui
 
public:
    PlotGuiClass(int r=THREAD_RATE): RateThread(r)
    {
        fprintf(stderr, "constructor\n");   
    }

    bool threadInit()
    {   
        port_to_iCubGui = new BufferedPort<Bottle>;
        port_in_zmp = new BufferedPort<Vector>;
        port_in_com = new BufferedPort<Vector>;

        fprintf(stderr, "Initializing Thread ...\n");
        port_in_zmp->open(string("/guiBalancer/ZMP:i").c_str());
        port_in_com->open(string("/guiBalancer/COM:i").c_str());
        port_to_iCubGui->open(string("/guiBalancer/objects:o").c_str());

        Bottle bot;
        bot.clear();
        bot.addString("reset");
        port_to_iCubGui->prepare()=bot;
        port_to_iCubGui->write();

        return true;
    }

    void run()
    {   
        Vector *zmp_in = port_in_zmp->read(true);
        Vector *com_in = port_in_com->read(true);
        plot("DSP_ZMP",*zmp_in,255,0,0);
        plot("COM",*com_in,0,255,0);
        Vector root (3,0.0);
        plot("ROOT",root,0,0,255);

    }

    void plot(string text, yarp::sig::Vector v, int r, int g, int b)
    {
        Bottle bot;
        // bot.clear();
        // bot.addString("object");
        // bot.addString(text.c_str());

        //-------------------
        bot.clear();
        bot.addString("object");
        bot.addString(text.c_str());

        bot.clear();
        bot.addString("object_with_label");
        char buff[255];
        sprintf (buff,"%+3.3f %+3.3f",v[0],v[1]);
        bot.addString(text.c_str());
        bot.addString(buff);

        //setting size
        bot.addDouble(10);
        bot.addDouble(10);
        bot.addDouble(10);

        //setting position
        // reference frame: HAS BEEN CHANGED AND NOW IT'S JUST LIKE THE ROOT REFERENCE FRAME ON THE SIMULATOR
        bot.addDouble(-v[0]*1000); //@@@
        bot.addDouble(v[1]*1000);
        bot.addDouble(-597.6);

        // bot.addDouble(50);
        // bot.addDouble(20);
        // bot.addDouble(-597.6);

        //rotation
        bot.addDouble(0);
        bot.addDouble(0);
        bot.addDouble(0);

        //color
        bot.addInt(r);
        bot.addInt(g);
        bot.addInt(b);

        //Transparency
        bot.addDouble(1.0);
        port_to_iCubGui->prepare()=bot;
        port_to_iCubGui->write(true); 
    }

    void threadRelease()
    {
        fprintf(stderr, "Closing /guiBalancer/objects:o...\n");
        closePort(port_to_iCubGui);

        fprintf(stderr, "Closing /guiBalancer/ZMP:i...\n");
        closePort(port_in_zmp);

        fprintf(stderr, "Closing /guiBalancer/COM:i...\n");
        closePort(port_in_com);

    }


    void closePort(Contactable *_port)
    {
        if(_port)
        {
            _port->interrupt();
            _port->close();

            delete _port;
            _port = 0;
        }
    }

};



int main(int argc, char *argv[])
{
    Network yarp;
    Time::turboBoost();

    if (!yarp.checkNetwork())
    {
        fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    PlotGuiClass plottingThread;
    plottingThread.start();

    while(1)
    {
        Time::delay(1);
    }

    plottingThread.stop();
}
