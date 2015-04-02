/*
 * SpeechSegment.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: cdondrup
 */

#include "iCub/SpeechSegment.h"
#include "iCub/simmx.h"

bool SpeechSegment::load() {
	sfinfo = new SF_INFO();
	sfinfo->format = 0;
	wave = NULL;
	cout << "Loading: "<< path;
	sndfile = sf_open(path.c_str(), SFM_READ, sfinfo);
	if (sf_error(sndfile)) {
		cerr << sf_strerror(sndfile) << endl;
		loaded = false;
		return loaded;
	}
	cout << " ... Loaded";

	//Storing values that might be overridden in the sfinfo
	frames = sfinfo->frames;
	channels = sfinfo->channels;
	arraysize = frames * channels;
	samplerate = sfinfo->samplerate;

	if (wave)
		delete[] wave;
	//Readinf sudio data into a float array
	wave = new float[arraysize];
	sf_count_t check = sf_readf_float(sndfile, wave, frames);
	cout << ": " << check << "/" << frames << " frames" << endl;
	loaded = close(sndfile);

	wave = stereoToMono(wave, frames, arraysize);

	return loaded;
}

bool SpeechSegment::close(SNDFILE* sndfile) {
	int err = sf_close(sndfile);
	if (err) {
		cerr << sf_error_number(err) << endl;
		return false;
	}
	return true;
}

bool SpeechSegment::transform(float* wave, int frames) {
	spectrum = stft::windowedFFT(wave, frames);
	cout<<"Created spectrum of size: "<<spectrum.size()<<endl;
	transformed = true;
	return transformed;
}

float** SpeechSegment::decomposeStereo(float* wave, int frames, int arraysize) {
	float** result = new float*[2];
	result[0] = new float[frames];
	result[1] = new float[frames];
	long j = 0;
	//Stereo data is just one mixed array -> left,right,left,right...
	for (long i = 0; i < arraysize; i++) {
		if (i % 2 == 0) {
			result[0][j] = wave[i];
		} else {
			result[1][j] = wave[i];
			j++;
		}
	}
	return result;
}

float* SpeechSegment::stereoToMono(float* wave, int frames, int arraysize) {
	float* result = new float[frames];
	float** channels = decomposeStereo(wave, frames, arraysize);
	delete [] wave;
	for(int i = 0; i < frames; i++) {
		result[i] = (channels[0][i] + channels[1][i]) / 2;
	}
	delete [] channels[0];
	delete [] channels[1];
	delete [] channels;
	return result;
}

bool SpeechSegment::writeSegment(float* wave, int frames, int channels) {
	string filename = path+"_test.wav";
	SF_INFO* seginfo = new SF_INFO();
	seginfo->channels = channels;
	seginfo->format = sfinfo->format;
	seginfo->frames = frames;
	seginfo->samplerate = sfinfo->samplerate;
	seginfo->sections = sfinfo->sections;
	seginfo->seekable = sfinfo->seekable;
	if (!sf_format_check(seginfo)) {
		cerr << "Specified format is invalid!" << endl;
		return false;
	}
	SNDFILE* segment = sf_open(filename.c_str(), SFM_WRITE, seginfo);
	if (sf_error(segment)) {
		cerr << sf_strerror(segment) << endl;
		return false;
	}
	sf_count_t check = sf_writef_float(segment, wave, frames);
	cout << "Successfully wrote: " << check << "/" << frames << " frames"
				<< endl << endl;
	sf_write_sync(segment);

	return close(segment);
}

