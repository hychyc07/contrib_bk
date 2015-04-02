/*
 * MatchWriter.cpp
 *
 *  Created on: Jun 11, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/MatchWriter.h"

using namespace std;
using namespace yarp::os;

void MatchWriter::setData(Bottle data, string filename) {
	if (bot.size() == 0) {
		bot.addVocab(VOCAB3('R','E','S'));
		addAsList(&bot, &data);
		bot.addVocab(VOCAB4('O','R','I','G'));
		bot.addString(filename.c_str());
	}
}

void MatchWriter::prepare() {
	//bot = &this->out->prepare();
	if (bot.size() > 0)
		bot.clear();
}

void MatchWriter::send() {
	out->write(bot);
}

void MatchWriter::error(string msg, string filename) {
	prepare();
	bot.addVocab(VOCAB3('E','R','R'));
	bot.addString(msg.c_str());
	bot.addVocab(VOCAB4('O','R','I','G'));
	bot.addString(filename.c_str());
	send();
}

bool MatchWriter::writeToFile(string filename, Bottle data, string orig) {
	if (filename != "") {
		ofstream outfile;
		outfile.open(filename.c_str(), ofstream::app);
		if (!outfile) {
			cout << "ERROR: Could write to " << filename << endl;
			return false;
		}
		//Writing the file names to the filelist.txt
		string result = basename(orig);
		result = result.substr(0, result.rfind("."));
		for (int i = 1; i < data.size(); i += 2) {
			if (outfile.good()) {
				string tmp = basename(string(data.get(i - 1).asString().c_str()));
				result += "," + tmp.substr(0, tmp.rfind(".")) + ","
						+ to_string<double>(data.get(i).asDouble());
			} else {
				cout << "ERROR: Could write to " << filename << endl;
				return false;
			}
		}
		result += "\n";
		outfile.write(result.c_str(), result.size());
		//Closing the filelist.txt file
		outfile.close();
		return true;
	}
	return false;
}
