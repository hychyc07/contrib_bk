/*
 * WarpingThread.h
 *
 *  Created on: Oct 29, 2012
 *      Author: cdondrup
 */

#ifndef WARPINGTHREAD_H_
#define WARPINGTHREAD_H_

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <map>
#include <algorithm>

#include <yarp/os/all.h>

#include "iCub/SpeechSegment.h"
#include "iCub/simmx.h"
#include "iCub/PathFinder.h"
#include "iCub/ListCreator.h"
#include "iCub/StringConverter.h"

using namespace std;
using namespace yarp::os;

class WarpingThread: public Thread {
private:
	bool stoploop;
	Port *in, *out;
	string filelist, outfile;
	map<SpeechSegment*, vector<SpeechSegment*> > utterances;

	map<SpeechSegment*, vector<SpeechSegment*> > searchForUtterance(
			string soundfile);
	bool isNew(string soundfile);
	void printResults();
	bool writeResults(string outfilename);

public:
	WarpingThread(string filelist, Port* in, Port* out, string outfile) :
			filelist(filelist), in(in), out(out), outfile(outfile) {
	}
	bool threadInit() {
		return true;
	}
	void threadRelease() {
	}
	void run() {
		if (filelist != "") {
			cout << "Reading wave file list: " << filelist << endl;
			//Opening the file list
			ifstream infile;
			infile.open(filelist.c_str(), ifstream::in);
			if (!infile) {
				cout << "ERROR: Unable to open file: " << filelist << endl;
				return;
			}
			//reading file list line by line
			while (infile.good()) {
				char line[500];
				infile.getline(line, 500);
				if (infile.good()) {
					searchForUtterance(string(line));
				}
			}
			infile.close();
			printResults();
			writeResults(outfile);
		} else {
			while (!stoploop) {
				Bottle bot;
				in->read(bot);
				//Checking if the bottle is good
				while (bot.size() > 0) {
					searchForUtterance(string(bot.pop().asString().c_str()));
				}
				printResults();
				writeResults(outfile);
			}
		}
		cout << "Search finished." << endl;
	}
	void stopLoop() {
		stoploop = true;
	}
};

#endif /* WARPINGTHREAD_H_ */
