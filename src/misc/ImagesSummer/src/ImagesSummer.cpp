#include "iCub/ImagesSummer.h"


bool ImagesSummer::configure(yarp::os::ResourceFinder &rf)
{    
   /* Process all parameters from both command-line and .ini file */

   /* get the module name which will form the stem of all module port names */
   moduleName            = rf.check("name", Value("vipSim/sum1"), 
                           "module name (string)").asString();
   /* before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name*/
   setName(moduleName.c_str());

   /* flowPort : input port for the optical flow (default: /optFlow:i).
	* meanCoordPort: output port for 2d coordinates (default: /flowMean/2Dcoord:o)
	* meanImgPort: output port for images representing the point of the flow’s mean (default: /flowMean/img:o)
	*/
   /* get the name of the output ports, automatically prefixing the module name by using getName() */
   image1PortName				= "/";
   image2PortName				= "/";
   imageSumPortName				= "/";
   image1PortName				+= getName( rf.check("image1Port", Value("/img1:i"), 
	   "First image input port (string)").asString());
   image2PortName				+= getName( rf.check("image2Port", Value("/img2:i"), 
	   "Second image input port (string)").asString());
   imageSumPortName				+= getName( rf.check("imageSumPort", Value("/imgSum:o"), 
	   "Images sum output port (string)").asString());

   /* get some other values from the configuration file */
   img1weight					= rf.check("img1weight", Value(0.5), 
	   "Weight of the first image in the sum (double in [0,1])").asDouble();
   delay						= rf.check("delay", Value(0.1), 
	   "Delay between two consecutive sum in sec (double)").asDouble();
     
   /* open ports  */        
   if (!image1Port.open(image1PortName.c_str())) {
      cout << getName() << ": unable to open port " << image1PortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }
   if (!image2Port.open(image2PortName.c_str())) {
      cout << getName() << ": unable to open port " << image2PortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }
   if (!imageSumPort.open(imageSumPortName.c_str())) {
      cout << getName() << ": unable to open port " << imageSumPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }   

   /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
   handlerPortName = "/";
   handlerPortName += getName(rf.check("handlerPort", Value("/ImagesSummer")).asString());
   if (!handlerPort.open(handlerPortName.c_str())) {
      cout << getName() << ": Unable to open port " << handlerPortName << endl;  
      return false;
   }
   attach(handlerPort);                  // attach to port
   //attachTerminal();                     // attach to terminal


   /* create the thread and pass pointers to the module parameters */
   myThread = new MyThread(&image1Port, &image2Port, &imageSumPort, &img1weight, &delay);
   /* now start the thread to do the work */
   myThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true ;      // let the RFModule know everything went well
                      // so that it will then run the module
}


bool ImagesSummer::interruptModule()
{
   image1Port.interrupt();
   image2Port.interrupt();
   imageSumPort.interrupt();
   handlerPort.interrupt();

   return true;
}


bool ImagesSummer::close()
{
   image1Port.close();
   image2Port.close();
   imageSumPort.close();
   handlerPort.close();

   /* stop the thread */
   myThread->stop();

   return true;
}


bool ImagesSummer::respond(const Bottle& command, Bottle& reply) 
{
  string helpMessage =  string(getName().c_str()) + " commands are: \n" +  "help \n" + "quit";
  reply.clear(); 

  if (command.get(0).asString()=="quit") {
       reply.addString("quitting");
       return false;
   }
   else if (command.get(0).asString()=="help") {
      cout << helpMessage;
      reply.addString("ok");
   }
   //else if (command.get(0).asString()=="set") {
   //   if (command.get(1).asString()=="thr") {
   //      thresholdValue = command.get(2).asInt(); // set parameter value
   //      reply.addString("ok");
   //   }
   //}
   return true;
}

bool ImagesSummer::updateModule(){ return true;}
double ImagesSummer::getPeriod(){ return 0.1;}

