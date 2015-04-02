#include <string.h>
#include<time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/all.h>
#include "pmp.h"
#include "vision.h"

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>


 //
 using namespace yarp::dev;
 using namespace yarp::os;
 using namespace yarp::sig;
 using namespace yarp::sig::draw;
 using namespace yarp::sig::file;
 using namespace yarp;

 int main(int argc, char **argv)

/* Passes the appropriate coordinates and other arguements to the MotCon function 
for the pushing task*/
{    
        Network yarp;
    
	    Network::init();
		
        PassiveMotionParadigm *P1 = new PassiveMotionParadigm(0);
		P1->MotCon(-50,-300,100,50,-300,100);

	   	Network::fini();
		return 0;
}