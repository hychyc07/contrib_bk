/*
 * SegmentationThread.h
 *
 *  Created on: Jul 18, 2012
 *      Author: cdondrup
 */

#ifndef SEGMENTATIONTHREAD_H_
#define SEGMENTATIONTHREAD_H_

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <yarp/os/all.h>

#include "iCub/FileHandler.h"
#include "iCub/Segmenter.h"
#include "iCub/SignalProcessor.h"

using namespace std;
using namespace yarp::os;

/**
 * @brief The main thread for the segmentation task.
 */
class SegmentationThread: public Thread {
private:
	bool stoploop, verbose, fromFiltered;
	string audiofile, workdir;
	int lowPassCutOffFreq, highPassCutOffFreq;
	int lambda, delta;
	int choose;
	int prefac;
	Port *out;
	float mu;

	FileHandler* fileHandler;
	SignalProcessor* signalProcessor;
	Segmenter* segmenter;

public:
	/**
	 * @brief Creates a new SegmentationThread Object.
	 *
	 * @param audiofile The audio file (.wav) to segment or a test file with a list of files
	 * @param workdir The directory where the result is stored
	 * @param lowPassCutOffFreq The frequency for the low pass filter
	 * @param highPassCutOffFreq The frequency for the high pass filter
	 * @param lambda The minimal speech segment size
	 * @param delta The maximal inter segment distance to be counted as one segment
	 * @param choose Either -1,0,1 chooses the result with less segments if -1 and the result
	 * with more segment if 1. If 0 throws error if not both stereo channel return the
	 * same number of segments.
	 * @param fromFiltered Iff true creates the segment files from the filtered original
	 * @param verbose Triggers verbose mode
	 */
	SegmentationThread(Port *out, string audiofile, string workdir,
			int lowPassCutOffFreq, int highPassCutOffFreq, int prefac,
			int lambda, int delta, float mu, int choose, bool fromFiltered, bool verbose) :
			audiofile(audiofile), workdir(workdir), lowPassCutOffFreq(
					lowPassCutOffFreq), highPassCutOffFreq(highPassCutOffFreq), verbose(
					verbose), lambda(lambda), delta(delta), choose(choose), fromFiltered(
					fromFiltered), out(out), prefac(prefac), mu(mu) {
	}

	bool threadInit() {
		fileHandler = new FileHandler(audiofile, workdir, verbose);
		signalProcessor = new SignalProcessor(lowPassCutOffFreq,
				highPassCutOffFreq, prefac, verbose);
		segmenter = new Segmenter(fileHandler, signalProcessor, lambda, delta, mu,
				choose, fromFiltered, verbose);
		return true;
	}

	void threadRelease() {
		cout << "  Segmentation stopped" << endl;
		delete fileHandler;
		delete signalProcessor;
		delete segmenter;
		if (audiofile != "")
			cout << "======" << endl << "press CTRL-C to exit..." << endl;
	}

	/**
	 * @brief Starts the main loop of the Thread.
	 */
	void run() {
		if (segmenter->segmentSignal()) {
			Bottle ori;
			ori.addVocab(VOCAB4('O','R','I','G'));
			ori.addString(audiofile.c_str());
			out->write(ori);
			for (int i = 0; i < segmenter->getSegmentNames().size(); i++) {
				Bottle seg;
				seg.addVocab(VOCAB3('S','E','G'));
				seg.addString(segmenter->getSegmentNames()[i].c_str());
				out->write(seg);
			}
		}
	}

	/**
	 * @brief Sets a bool to end the loop in the run function.
	 */
	void stopLoop() {
		stoploop = true;
		out->interrupt();
	}
};

class ControlThread: public Thread {
private:
	bool stoploop;
	SegmentationThread* segmentationThread;
	bool verbose, fromFiltered;
	string audiofile, workdir;
	int lowPassCutOffFreq, highPassCutOffFreq;
	int lambda, delta;
	int choose;
	int prefac;
	float mu;
	Port *control, *out;

public:
	/**
	 * @brief Creates a new ControlThread Object.
	 *
	 * @param audiofile The audio file (.wav) to segment or a test file with a list of files
	 * @param workdir The directory where the result is stored
	 * @param lowPassCutOffFreq The frequency for the low pass filter
	 * @param highPassCutOffFreq The frequency for the high pass filter
	 * @param lambda The minimal speech segment size
	 * @param delta The maximal inter segment distance to be counted as one segment
	 * @param choose Either -1,0,1 chooses the result with less segments if -1 and the result
	 * with more segment if 1. If 0 throws error if not both stereo channel return the
	 * same number of segments.
	 * @param fromFiltered Iff true creates the segment files from the filtered original
	 * @param verbose Triggers verbose mode
	 */
	ControlThread(Port *control, Port *out, string audiofile, string workdir,
			int lowPassCutOffFreq, int highPassCutOffFreq, int prefac,
			int lambda, int delta, float mu, int choose, bool fromFiltered, bool verbose) :
			workdir(workdir), lowPassCutOffFreq(lowPassCutOffFreq), highPassCutOffFreq(
					highPassCutOffFreq), verbose(verbose), lambda(lambda), delta(
					delta), choose(choose), fromFiltered(fromFiltered), control(
					control), audiofile(audiofile), out(out), prefac(prefac), mu(mu) {
	}

	bool threadInit() {
		stoploop = false;
		segmentationThread = NULL;
		return true;
	}
	void threadRelease() {
		segmentationThread->stopLoop();
		segmentationThread->stop();
	}

	/**
	 * @brief Starts the main loop of the Thread.
	 */
	void run() {
		if (audiofile == "") {
			while (!stoploop) {
				Bottle bot;
				control->read(bot);
				if (bot.size() > 0) {
					if (segmentationThread != NULL) {
						segmentationThread->stopLoop();
						segmentationThread->stop();
						delete segmentationThread;
					}
					segmentationThread = new SegmentationThread(out,
							string(bot.get(0).asString().c_str()), workdir,
							lowPassCutOffFreq, highPassCutOffFreq, prefac,
							lambda, delta, mu, choose, fromFiltered, verbose);
					segmentationThread->start();
				}
			}
		} else if (audiofile.substr(audiofile.rfind(".") + 1) == "wav") {
			segmentationThread = new SegmentationThread(out, audiofile, workdir,
					lowPassCutOffFreq, highPassCutOffFreq, prefac, lambda,
					delta, mu, choose, fromFiltered, verbose);
			segmentationThread->start();
		} else {
			cout << "Reading wave file list: " << audiofile << endl;
			//Opening the file list
			ifstream infile;
			infile.open(audiofile.c_str(), ifstream::in);
			if (!infile) {
				cout << "ERROR: Unable to open file: " << audiofile << endl;
				return;
			}
			//reading file list line by line
			while (infile.good() && !stoploop) {
				char line[500];
				infile.getline(line, 500);
				if (infile.good()) {
					string tmp(line);
					cout << "Segmenting: " << tmp << endl;
					if(segmentationThread != NULL)
						delete segmentationThread;
					segmentationThread = new SegmentationThread(out,
							tmp, workdir, lowPassCutOffFreq,
							highPassCutOffFreq, prefac, lambda, delta, mu, choose,
							fromFiltered, verbose);
					segmentationThread->start();
					while(segmentationThread->isRunning());
				}
			}
			infile.close();
		}
	}

	/**
	 * @brief Sets a bool to end the loop in the run function.
	 */
	void stopLoop() {
		stoploop = true;
		control->interrupt();
	}
};

#endif /* SEGMENTATIONTHREAD_H_ */
