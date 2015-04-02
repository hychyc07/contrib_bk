/*
 * SoundGrabber.cpp
 *
 *  Created on: Aug 16, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/SoundGrabber.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

bool SoundGrabber::threadInit() {
	stoploop = false;
	recording = false;
	buffer.reserve(BUFFER_SIZE);
	audioFileCreator = new AudioFileCreator(results, workdir);
	portAudioGrabber = new PortAudioGrabber(amplify);
	return true;
}

void SoundGrabber::threadRelease() {
	buffer.clear();
	delete portAudioGrabber;
	delete audioFileCreator;
}

void SoundGrabber::run() {
	while (!stoploop) {
		Sound* sound = &out->prepare(); // = in->read();
		portAudioGrabber->setAmplify(amplify);
		portAudioGrabber->getSound(*sound);
		if (true /*sound*/) {
			if (isRecording()) {
//				cout<<snippet.size()<<endl;
				if (snippet.size() == 0)
					for (vector<Sound>::iterator it = buffer.begin();
							it < buffer.end(); ++it) {
						snippet.push_back(*it);
					}
				snippet.push_back(*sound);
			} else {
				if(snippet.size() > 0) {
					audioFileCreator->write(snippet);
					snippet.clear();
				}
			}
//			cout<<sound->getSamples()<<endl;
			if (buffer.size() >= BUFFER_SIZE) {
				buffer.erase(buffer.begin());
			}
			buffer.push_back(*sound);
			out->write();
		}
	}
}

