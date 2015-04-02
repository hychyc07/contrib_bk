/*
 * SpeechSegment.h
 *
 *  Created on: Oct 30, 2012
 *      Author: cdondrup
 */

#ifndef SPEECHSEGMENT_H_
#define SPEECHSEGMENT_H_

#include <string>
#include <iostream>
#include <vector>

#include <sndfile.h>
#include <fftw3.h>

#include "iCub/stft.h"
#include "iCub/istft.h"

using namespace std;

class SpeechSegment {
private:
	SNDFILE* sndfile;
	SF_INFO* sfinfo;
	float* wave;
	sf_count_t frames;
	int channels;
	int arraysize;
	int samplerate;
	string path;
	string name;
	bool loaded;
	vector<fftw_complex*> spectrum;
	bool transformed;
	bool compared;
	double** simMat;
	int simMatM;
	int simMatN;
	double cost;

	bool load();
	bool close(SNDFILE* sndfile);
	bool transform(float* wave, int frames);
	float** decomposeStereo(float* wave, int frames, int arraysize);
	float* stereoToMono(float* wave, int frames, int arraysize);
	bool writeSegment(float* wave, int frames, int channels);

public:
	SpeechSegment(string path) :
			path(path), loaded(false), transformed(false), compared(false) {
	}

//	~SpeechSegment() {
//		delete[] wave;
//	}

	bool isLoaded() const {
		return loaded;
	}

	int getArraysize() {
		if (!isLoaded())
			load();
		return arraysize;
	}

	void setArraysize(int arraysize) {
		this->arraysize = arraysize;
	}

	int getChannels() {
		if (!isLoaded())
			load();
		return channels;
	}

	void setChannels(int channels) {
		this->channels = channels;
	}

	sf_count_t getFrames() {
		if (!isLoaded())
			load();
		return frames;
	}

	void setFrames(sf_count_t frames) {
		this->frames = frames;
	}

	string getName() const {
		return getPath().substr(getPath().rfind("/")+1, getPath().rfind(".wav")-getPath().rfind("/")-1);
	}

	void setName(string name) {
		this->name = name;
	}

	string getPath() const {
		return path;
	}

	void setPath(string path) {
		this->path = path;
	}

	int getSamplerate() {
		if (!isLoaded())
			load();
		return samplerate;
	}

	void setSamplerate(int samplerate) {
		this->samplerate = samplerate;
	}

	float* getWave() {
		if (!isLoaded())
			load();
		return wave;
	}

	void setWave(float* wave) {
		this->wave = wave;
	}

	bool operator<(const SpeechSegment& other) const {
		return true;
	}
	bool operator==(const SpeechSegment& other) const {
		return path == other.path;
	}

	vector<fftw_complex*> getSpectrum() {
		if (!isTransformed())
			transform(getWave(), getFrames());
		return spectrum;
	}

	bool isTransformed() const {
		return transformed;
	}

	bool isCompared() const {
		return compared;
	}

	double** getSimMat() const {
		return simMat;
	}

	void setSimMat(double** simMat) {
		this->simMat = simMat;
	}

	int getSimMatM() const {
		return simMatM;
	}

	void setSimMatM(int simMatM) {
		this->simMatM = simMatM;
	}

	int getSimMatN() const {
		return simMatN;
	}

	void setSimMatN(int simMatN) {
		this->simMatN = simMatN;
	}

	double getCost() const {
		return cost;
	}

	void setCost(double cost) {
		this->cost = cost;
	}

};

#endif /* SPEECHSEGMENT_H_ */
