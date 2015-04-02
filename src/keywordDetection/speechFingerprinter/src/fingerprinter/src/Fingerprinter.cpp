/*
 * Fingerprinter.cpp
 *
 *  Created on: Jun 22, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/Fingerprinter.h"

bool Fingerprinter::threadInit() {
	cout << " Fingerprinter" << endl;

	/* Initializing all global variables */
	stoploop = false;
	//Reading the filters from the descriptor file
	filters = readFilters((char*) filterfile.c_str());
	if (filters.size() == 0) {
		cout << "ERROR: could not read " << filterfile << endl;
		return false;
	}
	//Trying to open the working directory to check if it exists
	DIR* tmp = opendir((char*) workdir.c_str());
	if (tmp == NULL) {
		cout << "ERROR: " << workdir << " does not exist!" << endl;
		return false;
	} else {
		closedir(tmp);
	}
	return true;
}

void Fingerprinter::threadRelease() {
	cout << "  Fingerprinter stopped" << endl;
	if (audiofile != "")
		cout << "======" << endl << "press CTRL-C to continue.." << endl;
}

void Fingerprinter::run() {
	/* Checking if an audio file or a file list was given and calling the appropriate fingerprinting method */
	if (audiofile.substr(audiofile.find_last_of(".") + 1) == "wav") {
		//Wave file was given on startup
		cout << "Fingerprinting: " << audiofile << endl;
		//Fingerprinting singel audio wave file
		if (!fingerprint((char*) audiofile.c_str()))
			return;
		cout << "Fingerprinting finished." << endl;
	} else if (audiofile.substr(audiofile.find_last_of(".") + 1) == "txt") {
		//File list was given on startup
		cout << "Reading wave file list: " << audiofile << endl;
		//Opening the file list
		ifstream infile;
		infile.open(audiofile.c_str(), ifstream::in);
		if (!infile) {
			cout << "ERROR: Unable to open file: " << audiofile << endl;
			return;
		}
		//reading file list line by line
		while (infile.good()) {
			char line[500];
			infile.getline(line, 500);
			if (infile.good()) {
				cout << "Fingerprinting: " << line << endl;
				//fingerprinting each file listed in the file list
				if (!fingerprint(line))
					return;
			}
		}
		cout << "Fingerprinting finished." << endl;
		infile.close();
	} else if (audiofile == "") {
		while (!stoploop) {
			Bottle bot;
			control->read(bot);
			if (bot.size() > 0) {
				ConstString newfile = bot.get(0).asString();
				cout << "Fingerprinting: " << newfile << endl;
				if (!fingerprint((char*) newfile.c_str())) {
					string msg = "Unable to fingerprint: ";
					msg += newfile.c_str();
					cerr<<msg<<endl;
					bot.addString(msg.c_str());
				}
				cout<<"Updating database in "<<workdir<<endl;
				createDatabase(workdir);
				result->write(bot);
			}
		}
	} else {
		cout << "ERROR: Unknown file type: " << audiofile << endl;
	}

	/* Creating the database files after ending the fingerprinting to make sure everything is represented */
	if(!createDatabase(workdir))
		return;
}

bool Fingerprinter::createDatabase(string workdir) {
	DatabaseBuilder* db = new DatabaseBuilder(workdir);
	if (!db->create()) {
		cout << "ERROR: Could not create database files in " << workdir << endl;
		return false;
	}
	return true;
}

string Fingerprinter::createFilename(string workdir, string filename, int num) {
	string file = workdir + "/"
			+ filename.substr(filename.find_last_of("/") + 1,
					filename.find_last_of(".")
							- (filename.find_last_of("/") + 1));
	if(num > 0)
		return file + "-" + to_string<int>(num) + ".keys";
	ifstream infile;
	infile.open(string(file + ".keys").c_str(), ifstream::in);
	if (!infile || overwrite) {
		infile.close();
		return file + ".keys";
	}
	infile.close();
	return file + "_" + to_string<double>(Time::now()) + ".keys";
}

bool Fingerprinter::fingerprint(char* file) {
	unsigned int nsamples = 0, freq = 0;
	//Reading the wave file
	float* audio = wavread(file, &nsamples, &freq);
	if (!audio) {
		cout << "ERROR: " << file << " could not be opened!" << endl;
		return false;
	}
	//Creating the fingerprint
	unsigned int nbits;
	unsigned int * bits = wav2bits(filters, audio, nsamples, freq, &nbits);

	//The wave file isn't needed anymore
	free(audio);

	if(bits == NULL) {
		cout << "ERROR: " << file << " is too short to compute!" << endl;
		return false;
	}

	//Storing the fingerprint to the working directory
	char* out = (char*) createFilename(workdir, string(file)).c_str();
	printf("Writing %d keys to %s.\n", nbits, out);
	int i = 0;
	while(!writebits(bits, nbits, out)) {
		out = (char*) createFilename(workdir, string(file), ++i).c_str();
		cout<<"Changing file name too: "<<out<<endl;
		if(i == 5)
			break;
	}

	//Freeing the fingerprint
	free(bits);
	return true;
}

