/*
 * SoundPortReader.h
 *
 *
 *      Author: Christian Dondrup
 */

#ifndef SOUNDPORTREADER_H_
#define SOUNDPORTREADER_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <stdio.h>
#include <iostream>
#include <vector>

#include "iCub/Converter.h"
#include "iCub/Correlator.h"
#include "iCub/Volume.h"
#include "iCub/SignalProcessor.h"
#include "iCub/SaliancyMapCreator.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

#define WINDOW_SIZE 1024
#define BUFFER_SIZE 5

/**
 * This class listens on the Object port and creates a new ObjectData object when there is new data.
 */
class SoundPortReader: public Thread {
private:

	/* class variables */
	bool stoploop;
	int windowstate;
	string filename;
	Sound output;
	Volume* volume;
	vector<Sound> buffer;
	SignalProcessor* signaleProcessor;
	int threshold, highPassFreq;
	SaliancyMapCreator saliancyMapCreator;


	/* thread parameters: */
	BufferedPort<Sound> *in;
	BufferedPort<ImageOf<PixelMono> >* out;
	Sound *sound;

public:
	SoundPortReader(BufferedPort<Sound> *in, BufferedPort<ImageOf<PixelMono> > *out, string filename, int threshold, int highPassFreq) :
		in(in), out(out), filename(filename), threshold(threshold), highPassFreq(highPassFreq) {}
	/* class methods */

	bool threadInit();
	void threadRelease();

	/**
	 * Reads from the Port instance given on creation.
	 */
	void run();

	/**
	 * Sets a bool to end the loop in the run function.
	 */
	void stopLoop() {
		stoploop = true;
	}
};

#endif /* SOUNDPORTREADER_H_ */
