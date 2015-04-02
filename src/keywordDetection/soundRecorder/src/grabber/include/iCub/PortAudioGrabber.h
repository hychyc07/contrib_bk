/*
 * PortAudioGrabber.h
 *
 *  Created on: Aug 17, 2012
 *      Author: cdondrup
 */

#ifndef PORTAUDIOGRABBER_H_
#define PORTAUDIOGRABBER_H_

#include <cstdlib>
#include <cstdio>

#include <yarp/os/Thread.h>
#include <yarp/os/ManagedBytes.h>
#include <yarp/sig/Sound.h>

#include <portaudio.h>

/* Select sample format. */
#define PA_SAMPLE_TYPE  paInt16
typedef short SAMPLE;
#define SAMPLE_SILENCE  (0)
#define SAMPLE_RATE  (44100)
#define NUM_CHANNELS    (2)
#define DITHER_FLAG     (0)
#define	NUM_SAMPLES	((int)(512))

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class PortAudioGrabber {
private:
	void *system_resource;
	yarp::os::ManagedBytes buffer;
	int amplify;

	bool closePortAudio();
	bool init();
	bool release();
public:
	PortAudioGrabber(int amplify);
	~PortAudioGrabber();
	bool getSound(Sound &sound);

	int getAmplify() const {
		return amplify;
	}

	void setAmplify(int amplify) {
		this->amplify = amplify;
	}
};

#endif /* PORTAUDIOGRABBER_H_ */
