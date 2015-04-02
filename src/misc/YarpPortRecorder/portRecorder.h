#ifndef __CHRIS_PORT_RECORDER__
#define __CHRIS_PORT_RECORDER__

//FILES
#include <fstream>

//STL
#include <iostream>
#include <string>

//YARP
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>



using namespace std;
using namespace yarp::os;

namespace chris 
{
        namespace YarpPortRecorder
		{
            class PortRecorder;
        }
}


class PortRecorder:public BufferedPort<Bottle>
{
private:
	ofstream*    file;
	Semaphore*   mutex;
	int          ID;
	string       sourcePort;
	int          mnrecords;
	int          mreccounter;
	double       mstarttime;
	bool         mhasfinished;

	

public:
	PortRecorder(string _sourcePort, int _ID, int _nrecords, ofstream* _file, Semaphore* _mutex);
	bool        Connect();
	void        recordOnce();
	bool        isFinished();
};

#endif
