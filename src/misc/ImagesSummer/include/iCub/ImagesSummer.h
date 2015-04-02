#ifndef __ICUB_IMAGESSUMMER_H__
#define __ICUB_IMAGESSUMMER_H__

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>

#include "iCub/MyThread.h"
 
using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;


class ImagesSummer:public RFModule
{
   /* module parameters */

   string moduleName;

   // names of the ports
   string image1PortName; 
   string image2PortName;  
   string imageSumPortName;
   string handlerPortName;

   double img1weight;	
   double delay;

   /* class variables */

   BufferedPort<ImageOf<PixelRgb> > image1Port;			
   BufferedPort<ImageOf<PixelRgb> > image2Port;		
   BufferedPort<ImageOf<PixelRgb>> imageSumPort;
   Port handlerPort;									// a port to handle messages

   /* pointer to a new thread to be created and started in configure() and stopped in close() */
   MyThread *myThread;


public:
   
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const Bottle& command, Bottle& reply);
   double getPeriod(); 
   bool updateModule();
};


#endif // __ICUB_IMAGESSUMMER_H__
//empty line to make gcc happy

