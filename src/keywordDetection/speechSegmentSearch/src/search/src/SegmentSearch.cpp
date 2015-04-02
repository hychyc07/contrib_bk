/*
 * SegmentSearch.cpp
 *
 *  Created on: Jun 28, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/SegmentSearch.h"

using namespace std;
using namespace yarp::os;

bool SegmentSearch::threadInit() {
	cout << " SegmentSearch" << endl;

	/* Initializing all global variables */
	stoploop = false;
	//Reading the filters from the descriptor file
	filters = readFilters((char*) filterfile.c_str());
	if (filters.size() == 0) {
		cerr << "ERROR: could not read " << filterfile << endl;
		return false;
	}
	//Trying to open the working directory to check if it exists
	DIR* tmp = opendir((char*) workdir.c_str());
	if (tmp == NULL) {
		cerr << "ERROR: " << workdir << " does not exist!" << endl;
		return false;
	} else {
		closedir(tmp);
	}
	//Trying to open the em parameters file to check if it exists
	ifstream check(emfile.c_str());
	if (check.good()) {
		check.close();
		//Reading em paramaters from file
		params = readEMParams((char*) emfile.c_str());
	} else {
		cerr << "ERROR: " << emfile << " could not be opened!" << endl;
		return false;
	}
	//Trying to open the file database to check if it exists
	check.open(string(workdir + SEPERATOR + FILES_DB).c_str());
	if (check.good()) {
		check.close();
	} else {
		cerr << (audiofile != "" ? "ERROR: " : "WARNING: ")
				<< workdir + SEPERATOR + FILES_DB
				<< " could not be opened!" << endl;
		if (audiofile != "")
			return false;
		else
			cerr << "Running in online mode: "
					"Be sure to create this file via the SpeechFingerprinter "
					"before trying to search it." << endl;
	}
	//Trying to open the keys database to check if it exists
	check.open(string(workdir + SEPERATOR + KEYS_DB).c_str());
	if (check.good()) {
		check.close();
	} else {
		cerr << (audiofile != "" ? "ERROR: " : "WARNING: ")
				<< workdir + SEPERATOR + KEYS_DB
				<< " could not be opened!" << endl;
		if (audiofile != "")
			return false;
		else
			cerr << "Running in online mode: "
					"Be sure to create this file via the SpeechFingerprinter "
					"before trying to search it." << endl;
	}
	if (audiofile != "")
		readDatabaseFiles();
//	//Creating a new keypoint database with the files and keys database
//	kdb = new KeypointDB((char*) string(workdir + SEPERATOR + FILES_DB).c_str(),
//			(char*) string(workdir + SEPERATOR + KEYS_DB).c_str());
//	//Creating a hashtable of keys
//	table = new DirectHash(kdb);
//	search = table;
	return true;
}

void SegmentSearch::readDatabaseFiles() {
	//Creating a new keypoint database with the files and keys database
	kdb = new KeypointDB((char*) string(workdir + SEPERATOR + FILES_DB).c_str(),
			(char*) string(workdir + SEPERATOR + KEYS_DB).c_str());
	//Creating a hashtable of keys
	table = new DirectHash(kdb);
	search = table;
}

void SegmentSearch::threadRelease() {
	cout << "  SegmentSearch stopped" << endl;
	if (audiofile != "")
		cout << "======" << endl << "press CTRL-C to continue.." << endl;
}

void SegmentSearch::run() {
	if (audiofile.substr(audiofile.find_last_of(".") + 1) == "wav") {
		//Wave file was given on startup
		cout << "Searching matches for: " << audiofile << endl;
		//Search for audio wave file
		if (!searchSegment((char*) audiofile.c_str()))
			return;
		cout << "Search finished." << endl;
	} else if (audiofile.substr(audiofile.find_last_of(".") + 1) == "txt") {
		//File list was given on startup
		cout << "Reading wave file list: " << audiofile << endl;
		//Opening the file list
		ifstream infile;
		infile.open(audiofile.c_str(), ifstream::in);
		if (!infile) {
			cerr << "ERROR: Unable to open file: " << audiofile << endl;
			matchWriter->error("Unable to open file: " + audiofile, audiofile);
			return;
		}
		//reading file list line by line
		while (infile.good()) {
			char line[500];
			infile.getline(line, 500);
			if (infile.good()) {
				cout << "Searching matches for: " << line << endl;
				//Search for each file listed in the file list
				if (!searchSegment(line))
					return;
			}
		}
		cout << "Search finished." << endl;
		infile.close();
	} else if (audiofile == "") {
		while (!stoploop) {
			Bottle bot;
			in->read(bot);
			//Checking if the bottle is good
			while (bot.size() > 0) {
				readDatabaseFiles();
				searchSegment((char*) bot.pop().asString().c_str());
			}
		}
	} else {
		cerr << "ERROR: Unknown file type: " << audiofile << endl;
	}
}

bool SegmentSearch::searchSegment(char* file) {
	unsigned int nsamples = 0, freq = 0;
	//Reading wave file from disk
	float* audio = wavread(file, &nsamples, &freq);
	if (!audio) {
		cerr << "ERROR: " << file << " could not be opened!" << endl;
		matchWriter->error(string(file) + " could not be opened!",
				string(file));
		return false;
	}
	unsigned int nbits;
	//Creating bit representation of wave file
	unsigned int * bits = wav2bits(filters, audio, nsamples, freq, &nbits);
	free(audio);
	if (bits == NULL) {
		matchWriter->error("Segment too short to compute!", string(file));
		free(kdb);
		free(table);
		search = NULL;
		return false;
	}
	//Creating finger print
	vector<Keypoint *> qkeys = bitsToKeys(bits, nbits);
	free(bits);
	//Looking for matches in database
	readDatabaseFiles();
	vector<Keypoint *> rkeys = FindMatches(search, qkeys, 0);
	map<unsigned int, vector<Keypoint *> > fkeys = FilterMatches(rkeys,
			(int) (MMATCH2 * qkeys.size()));
	map<unsigned int, vsong> scoresem = verify4em(fkeys, qkeys, *kdb, params);
	printcountstats(scoresem, *kdb);
	//Choosing n best matches
	multimap<float, pair<unsigned int, vsong> > scores = chooseBestNMatches(
			scoresem, nbest);
	DeleteKeys(qkeys);
	//Printing results to console
	matchWriter->prepare();
	Bottle bot;
	for (multimap<float, pair<unsigned int, vsong> >::iterator it =
			scores.begin(); it != scores.end(); it++) {
		cout << basename(kdb->getFileName(it->second.first)).c_str() << endl;
		bot.add(kdb->getFileName(it->second.first).c_str());
		bot.add(it->first);
	}
	matchWriter->setData(bot, string(file));
	matchWriter->send();
	matchWriter->writeToFile(outfile, bot, string(file));
	free(kdb);
	free(table);
	search = NULL;
	return true;
}

map<unsigned int, vsong> SegmentSearch::chooseBestMatch(
		map<unsigned int, vsong> & scores) {
	map<unsigned int, vsong> ret;

	unsigned int minf = 0;
	float minscore = INT_MAX;

	//Finding the match with minimal score
	//The lower the score the better the match
	for (map<unsigned int, vsong>::iterator it = scores.begin();
			it != scores.end(); it++) {
		if (it->second.score < minscore) {
			minf = it->first;
			minscore = it->second.score;
		}
	}

	if (minscore != INT_MAX)
		ret[minf] = scores[minf];

	return ret;
}

multimap<float, pair<unsigned int, vsong> > SegmentSearch::chooseBestNMatches(
		map<unsigned int, vsong> & scores, unsigned int n) {
	multimap<float, pair<unsigned int, vsong> > ret;

	multimap<float, unsigned int> sortedscores;

	//Sorting matches by scores
	for (map<unsigned int, vsong>::iterator it = scores.begin();
			it != scores.end(); it++) {
		sortedscores.insert(
				pair<float, unsigned int>(it->second.score, it->first));
	}

	//Choosing the best n matches as return value
	unsigned int count = 0;
	for (map<float, unsigned int>::iterator it = sortedscores.begin();
			it != sortedscores.end(); it++) {
		if (count < n)
			ret.insert(
					pair<float, pair<unsigned int, vsong> >(it->first,
							pair<unsigned int, vsong>(it->second,
									scores[it->second])));
		count++;
	}

	return ret;
}

void SegmentSearch::printcountstats(map<unsigned int, vsong> & matches,
		KeypointDB & kdb) {
	// map match score -> file id
	multimap<float, unsigned int> counts;

	for (map<unsigned int, vsong>::iterator it = matches.begin();
			it != matches.end(); ++it) {
		counts.insert(pair<float, unsigned int>(it->second.score, it->first));
	}

	for (multimap<float, unsigned int>::iterator it = counts.begin();
			it != counts.end(); ++it)
		printf("File %d: %f\t%s\n", it->second, it->first,
				basename(kdb.getFileName(it->second)).c_str());

}

