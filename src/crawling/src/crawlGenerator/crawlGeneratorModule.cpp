#include "crawlGeneratorThread.h"
#include "crawlGeneratorModule.h"

#include <yarp/os/Os.h>
#include <stdio.h>
#include <string>

#define DEBUG 0

////////////CRAWL GENERATOR MODULE/////////////////

crawlGeneratorModule::crawlGeneratorModule()
{
}

crawlGeneratorModule::~crawlGeneratorModule()
{
}

double crawlGeneratorModule::getPeriod()

{

	//    printf("crawl Generator Module is running\n");

	return 1.0;

}



bool crawlGeneratorModule::updateModule()
{
	if (theThread->isRunning()==true)
	{
		return true;
	}
	else
	{
		printf("The crawl generator thread has been stopped\n");
		printf("Closing the module...\n");
		return false;
	}


}


bool crawlGeneratorModule::close()
{    
	theThread->stop();
	delete theThread;
	fprintf(stderr, "%s module closed\n", partName.c_str());
	return true;
}


bool crawlGeneratorModule::configure(yarp::os::ResourceFinder &rf)
{
	Property options(rf.toString());
	int period = 25; // in ms

	if(options.check("period"))
	{
		period = options.find("period").asInt();
	}

	if(options.check("part"))
	{
		partName=options.find("part").asString().c_str();
		printf("module taking care of part %s\n",partName.c_str());
	}


	theThread = new GeneratorThread(period);
	if(!theThread->init(rf))
	{
		printf("Failed to initialize the thread\n");
		fflush(stdout);
		return false;
	}

	theThread->start();

	return true;

}

