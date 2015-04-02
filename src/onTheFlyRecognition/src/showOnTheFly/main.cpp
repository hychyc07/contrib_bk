/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Carlo Ciliberto
 * email:  carlo.ciliberto@iit.it
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

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/PortReport.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <highgui.h>
#include <cv.h>

#include <stdio.h>
#include <string>
#include <deque>
#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;







class ShowPort: public BufferedPort<Image>
{
private:
   ResourceFinder                      &rf;

   Semaphore                           mutex;

   BufferedPort<Bottle>                port_blobs_in;
   BufferedPort<Bottle>                port_label_in;

   bool                                verbose;

   Port                                outPort;

   int                                 window_radius;

   string                              name_result;




   virtual void onRead(Image &img)
   {
        mutex.wait();

        //check if some blob has arrived
        Bottle *blob=port_blobs_in.read(false);
        if(blob!=NULL)
        {
           Bottle *window=blob->get(0).asList();
           if(window->get(2).asInt()>10)
           {
               Bottle *tmp_label=port_label_in.read(false);
               if(tmp_label!=NULL)
                   name_result=tmp_label->get(0).asString().c_str();

               //print the rectangle around the object and the name of the object.
               int x_min=window->get(0).asInt()-window_radius;
               int y_min=window->get(1).asInt()-window_radius;
               int x_max=window->get(0).asInt()+window_radius;
               int y_max=window->get(1).asInt()+window_radius;

               if(x_min<0)x_min=0;
               if(y_min<0)y_min=0;
               if(x_max>img.width()-1)x_max=img.width()-1;
               if(y_max>img.height()-1)y_max=img.height()-1;

               CvFont font;
               cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.8,0.8,0,3);

               int y_text=y_min-10;
               if(y_text<5) y_text=y_max+2;

               cvRectangle(img.getIplImage(),cvPoint(x_min,y_min),cvPoint(x_max,y_max),cvScalar(0,255,0),2);
               cvPutText(img.getIplImage(),name_result.c_str(),cvPoint(x_min,y_text),&font,cvScalar(0,0,255));
           }
        }


       if(outPort.getOutputCount())
       {
            outPort.write(img);
       }

       mutex.post();

    }



public:
   ShowPort(ResourceFinder &_rf)
       :BufferedPort<Image>(),rf(_rf)
   {
       verbose=rf.check("verbose");

       //features parameters initialization
       Bottle &bGeneral=rf.findGroup("general");

       name_result="?";
       window_radius=40;

       string name=rf.find("name").asString().c_str();

       outPort.open(("/"+name+"/img:o").c_str());
       port_blobs_in.open(("/"+name+"/blobs:i").c_str());
       port_label_in.open(("/"+name+"/label:i").c_str());

       BufferedPort<Image>::useCallback();


   }

   virtual void close()
   {
       outPort.close();
       port_blobs_in.close();
       port_label_in.close();
       BufferedPort<Image>::close();
   }


   bool execReq(const Bottle &command, Bottle &reply)
   {
       switch(command.get(0).asVocab())
       {

           default:
               return false;
       }
   }



};


class ShowModule: public RFModule
{
protected:
    ShowPort         *showPort;
    Port                rpcPort;

public:
    ShowModule()
    {}

    virtual bool configure(ResourceFinder &rf)
    {
        string name=rf.find("name").asString().c_str();

        Time::turboBoost();

        showPort=new ShowPort(rf);
        showPort->open(("/"+name+"/img:i").c_str());
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);

        return true;
    }

    virtual bool close()
    {
        showPort->interrupt();
        showPort->close();
        delete showPort;

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        if(showPort->execReq(command,reply))
            return true;
        else
            return RFModule::respond(command,reply);
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule()
    {

        return true;
    }

};




int main(int argc, char *argv[])
{
   Network yarp;

   if (!yarp.checkNetwork())
       return -1;

   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultContext("showOnTheFly/conf");
   rf.setDefaultConfigFile("config.ini");
   rf.configure("ICUB_ROOT",argc,argv);
   rf.setDefault("name","showOnTheFly");
   ShowModule mod;

   return mod.runModule(rf);
}

