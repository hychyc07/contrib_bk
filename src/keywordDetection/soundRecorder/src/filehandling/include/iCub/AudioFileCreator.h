/*
 * AudioFileCreator.h
 *
 *  Created on: Aug 16, 2012
 *      Author: cdondrup
 */

#ifndef AUDIOFILECREATOR_H_
#define AUDIOFILECREATOR_H_

#include <string>
#include <vector>
#include <iostream>

#include <sndfile.h>

#include <yarp/sig/Sound.h>
#include <yarp/os/all.h>

//#define AMPLIFY 1

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;

class AudioFileCreator {
private:
	string workdir;
	Port *out;

	bool open(SNDFILE** sndfile, SF_INFO* sndinfo, string filename);
	bool close(SNDFILE* sndfile);
	short* convert(vector<Sound> input, int* size);
	string createFilename(string workdir);
	bool sendInfo(string filename);
public:
	AudioFileCreator(Port *out, string workdir) :
		workdir(workdir), out(out) {}
	bool write(vector<Sound> input);
};




#endif /* AUDIOFILECREATOR_H_ */
