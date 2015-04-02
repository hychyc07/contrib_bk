#ifndef __CHRIS_MODULE_Player__
#define __CHRIS_MODULE_Player__

//DEFAULT PARAMETERS
#define DEFAULT_INPUT_FILE_NAME "input.txt"

//YARP
#include <yarp/os/all.h>

//STL
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;
using namespace yarp::os;

namespace chris 
{
        namespace PortPlayer
		{
            class PortPlayer;
        }
}

struct Record
{
	int						playerIndex;
	int						recordNumber;
	double					playTime;
	Bottle					content;
};

class PortPlayer: public yarp::os::Module
{
private:
	string								modulename;
	vector< BufferedPort<Bottle> * >	Players;
	ifstream*							inputFile;
	vector<Record>						inMemoryFile;

	int									mPeriod;
	bool								isLooping;
	bool								isPlaying;
	bool								isVerbose;
	double								startTime;
	vector<Record>::iterator			nextItem;

public:

	PortPlayer() { isPlaying = true;}
	virtual bool   open(yarp::os::Searchable &s);
	virtual bool   close();
	virtual bool   updateModule();
	virtual double getPeriod();

};
#endif
