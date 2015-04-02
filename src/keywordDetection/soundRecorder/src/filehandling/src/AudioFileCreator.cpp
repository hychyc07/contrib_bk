/*
 * AudioFileCreator.cpp
 *
 *  Created on: Aug 16, 2012
 *      Author: cdondrup
 */

#include "iCub/AudioFileCreator.h"
#include "iCub/StringConverter.h"

using namespace std;

bool AudioFileCreator::open(SNDFILE** sndfile, SF_INFO* sndinfo,
		string filename) {
	cout << "Writing: " << filename << endl;
	if (!sf_format_check(sndinfo)) {
		cerr << "Specified format is invalid!" << endl;
		return false;
	}
	*sndfile = sf_open(filename.c_str(), SFM_WRITE, sndinfo);
	if (sf_error(*sndfile)) {
		cerr << sf_strerror(*sndfile) << endl;
		return false;
	}

	return true;
}

bool AudioFileCreator::close(SNDFILE* sndfile) {
	int err = sf_close(sndfile);
	if (err) {
		cerr << sf_error_number(err) << endl;
		return false;
	}
	return true;
}

bool AudioFileCreator::write(vector<Sound> input) {
	int size = 0;
	short* wave = convert(input, &size);
//	cout << "Array size: " << size << endl;
//	for (int i = 0; i < size; i++)
//		cout << wave[i] << ", ";
//	cout << endl;
	SF_INFO* sndinfo = new SF_INFO();
	sndinfo->channels = input.begin()->getChannels();
	sndinfo->samplerate = input.begin()->getFrequency();
	sndinfo->format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;
	SNDFILE* sndfile;
	string filename = createFilename(workdir);
	open(&sndfile, sndinfo, filename);
	sf_count_t check = sf_writef_short(sndfile, wave, size / input.begin()->getChannels());
	cout << "Successfully wrote: " << check << "/" << size / input.begin()->getChannels() << " frames"
			<< endl << endl;
	sf_write_sync (sndfile);
	close(sndfile);
	sendInfo(filename);
	return true;
}

short* AudioFileCreator::convert(vector<Sound> input, int *size) {
	*size = input.size() * input.begin()->getChannels()
			* input.begin()->getSamples();
	short* result = new short[*size];
	for (int j = 0; j < input.size(); j++) {
		Sound* sound = &input.at(j);
//		for (int i = 0; i < sound->getSamples(); i ++) {
//			cout<<sound->getSafe(i,0)<<", ";
//			if (sound->getChannels() == 2) {
//				cout<<sound->getSafe(i,1)<<", ";
//			}
//		}
		for (int i = 0; i < sound->getSamples() * sound->getChannels(); i +=
				sound->getChannels()) {
			result[j * sound->getSamples() * sound->getChannels() + i] =
					sound->getSafe(i / sound->getChannels(), 0); // * AMPLIFY;
			if (sound->getChannels() == 2) {
				result[j * sound->getSamples() * sound->getChannels() + i + 1] =
						sound->getSafe(i / sound->getChannels(), 1); // * AMPLIFY;
			}
		}
	}
//	cout<<endl<<"----------------------------------------"<<endl;
	return result;
}

string AudioFileCreator::createFilename(string workdir) {
	return workdir + "/" + to_string<double>(Time::now()) + ".wav";
}

bool AudioFileCreator::sendInfo(string filename) {
	Bottle bot;
	bot.addString(filename.c_str());
	out->write(bot);
	return true;
}

