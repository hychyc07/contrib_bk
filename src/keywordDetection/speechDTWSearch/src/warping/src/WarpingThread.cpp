/*
 * WarpingThread.cpp
 *
 *  Created on: Oct 29, 2012
 *      Author: cdondrup
 */

#include "iCub/WarpingThread.h"

bool sortVector(SpeechSegment* x, SpeechSegment* y) {
	return x->getCost() < y->getCost();
}

map<SpeechSegment*, vector<SpeechSegment*> > WarpingThread::searchForUtterance(
		string soundfile) {

	if (!isNew(soundfile)) {
		cout << "Has already been compared: " << soundfile << endl;
		return utterances;
	}

	if (utterances.size() == 0) {
		cout << "First entry: " << soundfile << endl;
		vector<SpeechSegment*> tmp;
		utterances[new SpeechSegment(soundfile)] = tmp;
		return utterances;
	}

	vector<string> soundfiles;
	for (map<SpeechSegment*, vector<SpeechSegment*> >::iterator it =
			utterances.begin(); it != utterances.end(); ++it) {
		cout << "----------------------------------------------" << endl;
		soundfiles.push_back((*it).first->getPath());
		if ((*it).first->getPath() == soundfile)
			continue;
		cout << "Comparing: " << (*it).first->getPath() << endl;
		cout << "----------------------------------------------" << endl;
		cout << "to: " << soundfile << endl;
		SpeechSegment* comp = new SpeechSegment(soundfile);
		comp->setSimMat(
				simmx::calculateSimMatrix((*it).first->getSpectrum(),
						comp->getSpectrum()));
		comp->setSimMatM((*it).first->getSpectrum().size());
		comp->setSimMatN(comp->getSpectrum().size());
		comp->setCost(
				PathFinder::findPath(comp->getSimMat(), comp->getSimMatM(),
						comp->getSimMatN()));
		(*it).second.push_back(comp);

		cout << "----------------------------------------------" << endl;
		sort((*it).second.begin(), (*it).second.end(), sortVector);
	}

	SpeechSegment* key = new SpeechSegment(soundfile);
	vector<SpeechSegment*> tmp;
	for (vector<string>::iterator it = soundfiles.begin();
			it != soundfiles.end(); ++it) {
		cout << "----------------------------------------------" << endl;
		cout << "Comparing: " << key->getPath() << endl;
		cout << "----------------------------------------------" << endl;

		if (*it == key->getPath())
			continue;

		cout << "to: " << *it << endl;
		SpeechSegment* comp = new SpeechSegment(*it);
		comp->setSimMat(
				simmx::calculateSimMatrix(key->getSpectrum(),
						comp->getSpectrum()));
		comp->setSimMatM(key->getSpectrum().size());
		comp->setSimMatN(comp->getSpectrum().size());
		comp->setCost(
				PathFinder::findPath(comp->getSimMat(), comp->getSimMatM(),
						comp->getSimMatN()));
		tmp.push_back(comp);
	}
	utterances[key] = tmp;

	return utterances;
}

bool WarpingThread::isNew(string soundfile) {
	for (map<SpeechSegment*, vector<SpeechSegment*> >::iterator it =
			utterances.begin(); it != utterances.end(); ++it) {
		if ((*it).first->getPath() == soundfile)
			return false;
	}
	return true;
}

void WarpingThread::printResults() {
	Bottle result;
	cout << "Results:" << endl;
	for (map<SpeechSegment*, vector<SpeechSegment*> >::iterator it =
			utterances.begin(); it != utterances.end(); ++it) {
		cout << "++++++++++++++++++++++++++++++++++++++++++++++" << endl;
		cout << (*it).first->getPath() << ":" << endl;
		result.addString((*it).first->getPath().c_str());
		cout << "---------------------------------------------" << endl;
		Bottle tmp;
		for (vector<SpeechSegment*>::iterator vec_it = (*it).second.begin();
				vec_it != (*it).second.end(); ++vec_it) {
			cout << (*vec_it)->getPath() << ": " << (*vec_it)->getCost()
					<< endl;
			tmp.addString((*vec_it)->getPath().c_str());
			tmp.addDouble((*vec_it)->getCost());
		}
		addAsList(&result, &tmp);
	}
	cout << "++++++++++++++++++++++++++++++++++++++++++++++" << endl;
	out->write(result);
}

bool WarpingThread::writeResults(string outfilename) {
	if (outfile != "") {
		ofstream outfile;
		outfile.open(outfilename.c_str(), ofstream::out);
		if (!outfile) {
			cout << "ERROR: Could write to " << outfilename << endl;
			return false;
		}
		//Writing the file names to the filelist.txt
		for (map<SpeechSegment*, vector<SpeechSegment*> >::iterator it =
				utterances.begin(); it != utterances.end(); ++it) {
			if (outfile.good()) {
				string result = (*it).first->getName().substr(0,
						(*it).first->getName().rfind("-"));
				for (vector<SpeechSegment*>::iterator vec_it =
						(*it).second.begin(); vec_it != (*it).second.end();
						++vec_it) {
					result += ","
							+ (*vec_it)->getName().substr(0,
									(*vec_it)->getName().rfind("-")) + ","
							+ to_string<double>((*vec_it)->getCost());
				}
				result += "\n";
				outfile.write(result.c_str(), result.size());
			} else {
				cout << "ERROR: Could write to " << outfilename << endl;
				return false;
			}
		}
		//Closing the filelist.txt file
		outfile.close();
		return true;
	}
	return false;
}

