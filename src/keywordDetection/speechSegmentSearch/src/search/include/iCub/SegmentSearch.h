/*
 * SegmentSearch.h
 *
 *  Created on: Jun 28, 2012
 *      Author: cdondrup
 */

#ifndef SEGMENTSEARCH_H_
#define SEGMENTSEARCH_H_

#include <vector>
#include <dirent.h>
#include <string>
#include <fstream>
#include <iostream>
#include <limits.h>

#include <yarp/os/all.h>

#include <musicretr-1.0/sigproc.h>
#include <musicretr-1.0/keypoint.h>
#include <musicretr-1.0/csapp.h>
#include <musicretr-1.0/directhash.h>
#include <musicretr-1.0/util.h>
#include <musicretr-1.0/verify.h>

#include "iCub/Constants.h"
#include "iCub/MatchWriter.h"

using namespace std;
using namespace yarp::os;

class SegmentSearch: public Thread {
private:
	bool stoploop;
	string audiofile, filterfile, workdir, emfile;
	vector<Filter> filters;
	emparams params;
	KeypointDB *kdb;
	SearchStrategy *search;
	DirectHash *table;
	MatchWriter *matchWriter;
	Port *in;
	string outfile;
	int nbest;


	bool searchSegment(char* file);
	/** Choose the top 1 song based on confidence score, out of list of songs.
	 * Assumes the lower the score, the better the confidence.
	 */
	map<unsigned int, vsong> chooseBestMatch(map<unsigned int, vsong> & scores);
	/** Choose the top N songs based on confidence score, out of list of songs.
	 * Assumes the lower the score, the better the confidence.
	 */
	multimap<float, pair<unsigned int,vsong> > chooseBestNMatches(map<unsigned int, vsong> & scores,
			unsigned int n);
	/** Prints number of keys matched per song, sorted by number of keys.
	 * Could be slow, so don't call too often.
	 */
	void printcountstats(map<unsigned int, vsong> & matches, KeypointDB & kdb);

	void readDatabaseFiles();

public:
	/**
	 * @brief Creates a new SegmentSearch object.
	 * @param audiofile The path to a wave file or the path to a list of wave files (.txt)
	 * @param filters The path to the filter descriptor file
	 * @param workdir The path to a directory where everything should be stored
	 * @param emfile The path to the em desctipro file
	 * @param matchWriter An instance of MacthWrter to send the resulting data
	 * @param in The BufferedPort to read file names from if no audiofile is given
	 */
	SegmentSearch(string audiofile, string filters, string workdir,
			string emfile, MatchWriter* matchWriter, Port* in, int nbest, string outfile) :
			audiofile(audiofile), filterfile(filters), workdir(workdir), emfile(
					emfile), matchWriter(matchWriter), in(in), nbest(nbest), outfile(outfile) {
	}
	bool threadInit();
	void threadRelease();

	/**
	 * @brief Starts the main loop of the Thread.
	 *
	 */
	void run();

	/**
	 * @brief Sets a bool to end the loop in the run function.
	 */
	void stopLoop() {
		stoploop = true;
		in->interrupt();
	}
};

#endif /* SEGMENTSEARCH_H_ */
