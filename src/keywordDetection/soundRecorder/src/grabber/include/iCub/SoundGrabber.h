/*
 * SoundGrabber.h
 *
 *  Created on: Aug 16, 2012
 *      Author: Christian Dondrup
 */

#ifndef SOUNDGRABBER_H_
#define SOUNDGRABBER_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <vector>
#include <iostream>
#include <string>

#include "iCub/AudioFileCreator.h"
#include "iCub/PortAudioGrabber.h"

#define BUFFER_SIZE 5

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class SoundGrabber: public Thread {
private:
	bool stoploop, recording;
	BufferedPort<Sound> *out;
	Port *results;
	vector<Sound> buffer, snippet;
	Semaphore mutex;
	AudioFileCreator* audioFileCreator;
	string workdir;
	PortAudioGrabber* portAudioGrabber;
	int amplify;

public:
	SoundGrabber(BufferedPort<Sound> *out, Port *results, string workdir,
			int amplify) :
			out(out), mutex(1), workdir(workdir), amplify(amplify), results(
					results) {
	}
	bool threadInit();
	void threadRelease();

	/**
	 * @brief Starts the main loop of the Thread.
	 */
	void run();

	int getAmplify() const {
		return amplify;
	}

	void setAmplify(int amplify) {
		this->amplify = amplify;
	}

	/**
	 * @brief Sets a bool to end the loop in the run function.
	 */
	void stopLoop() {
		stoploop = true;
	}

	void startRecording() {
		mutex.wait();
		if (!isRecording())
			cout << "Start recording" << endl;

		recording = true;
		mutex.post();
	}
	void stopRecording() {
		mutex.wait();
		if (isRecording())
			cout << "Stop recording" << endl;

		recording = false;
		mutex.post();
	}
	bool isRecording() const {
		return recording;
	}

	void toggleRecording() {
		recording = isRecording() ? false : true;
	}
};

class ControleThread: public Thread {
private:
	bool stoploop;
	Port *in;
	SoundGrabber* soundGrabber;
public:
	ControleThread(Port *in, SoundGrabber *soundGrabber) :
			in(in), soundGrabber(soundGrabber) {
	}
	bool threadInit() {
		stoploop = false;
		return true;
	}
	void threadRelease() {
		soundGrabber->stopRecording();
	}

	/**
	 * @brief Starts the main loop of the Thread.
	 */
	void run() {
		while (!stoploop) {
			Bottle bot;
			in->read(bot);
			if (bot != NULL) {
				if (bot.size() > 0) {
					if (bot.get(0) == "start") {
						soundGrabber->startRecording();
					} else if (bot.get(0) == "stop") {
						soundGrabber->stopRecording();
					} else if (bot.get(0) == "amp") {
						if (bot.size() > 1) {
							soundGrabber->setAmplify(bot.get(1).asInt());
						}
					}
				}
			}
		}
	}

	/**
	 * @brief Sets a bool to end the loop in the run function.
	 */
	void stopLoop() {
		stoploop = true;
		in->interrupt();
	}
};

#endif /* SOUNDGRABBER_H_ */
