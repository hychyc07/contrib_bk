#ifndef __CHRIS_YARP_PORT_RECORDER__
#define __CHRIS_YARP_PORT_RECORDER__


//STL
#include <vector>
#include <sstream>

//YARP
#include <yarp/os/Contact.h>
#include <yarp/os/Module.h>
#include <yarp/os/Property.h>

//CHRIS
#include "portRecorder.h"

namespace chris 
{
        namespace YarpPortRecorder
		{
            class YarpPortRecorder;
        }
}


class YarpPortRecorder: public yarp::os::Module
{
private:
	string					mmodulename;
	int                     mcaptureperiod;
	int                     mdelayb4capture;
	int                     mfilesizelimit;
	vector<PortRecorder*>	mrecorders;
	Semaphore*				mmutex;
	ofstream*				moutputFile;
	bool*                   misRecording;
	BufferedPort<Bottle>	mrpcPort;

protected:
    virtual bool	        respond(const Bottle &command, Bottle &reply);

public:

	YarpPortRecorder()  {mmutex = new Semaphore(); moutputFile=NULL; misRecording = new bool(false);}
	virtual bool        open(yarp::os::Searchable &s);
	virtual bool        close();
	virtual bool        updateModule();
	virtual double      getPeriod();

};
#endif
