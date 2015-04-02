
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>


#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

#include "ExtractorUtils.h"

using namespace std;
using namespace yarp::os;

string generate_time_tag()
{
    //generate the time tag
    time_t rawtime;
    time( &rawtime );
    struct tm *timeinfo;
    timeinfo=localtime ( &rawtime );
    char idchar[30];
    strftime(idchar,30,"%Y%m%d-%H%M%S",timeinfo);
    //---------------------

    return static_cast<string>(idchar);
}



void initResourceFinder(ResourceFinder *rf, const string &context_path, const string &name)
{
    int argc=0;
    char **argv=NULL;
    rf->setVerbose(false);
    rf->setDefaultContext(context_path.c_str());
    rf->setDefaultConfigFile((name+".ini").c_str());

    rf->configure("ICUB_ROOT",argc,argv);
}

