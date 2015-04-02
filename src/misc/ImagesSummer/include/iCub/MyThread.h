#include <iostream>
#include <string>

#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>

#include "cv.h"				// openCV

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;

class MyThread : public Thread
{
private:
	/* class constants */

   /* class variables */
	double *img1weight;		
	double *delay;

   /* class variables */

   BufferedPort<ImageOf<PixelRgb>> *image1Port;			
   BufferedPort<ImageOf<PixelRgb>> *image2Port;		
   BufferedPort<ImageOf<PixelRgb>> *imageSumPort;

   /* class private methods */
   void log(string s, bool endLine=true);
   
public:

   /* class methods */

   MyThread(BufferedPort<ImageOf<PixelRgb> > *image1Port, BufferedPort<ImageOf<PixelRgb> > *image2Port, 
	   BufferedPort<ImageOf<PixelRgb>> *imageSumPort, double *img1weight, double *delay);
   bool threadInit();     
   void threadRelease();
   void run(); 
};