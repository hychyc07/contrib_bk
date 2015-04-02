/*
 * PortAudioGrabber.cpp
 *
 *  Created on: Aug 17, 2012
 *      Author: cdondrup
 */

#include "iCub/PortAudioGrabber.h"

using namespace std;
using namespace yarp::os;

static double pa_tap_test = 1e6;

PortAudioGrabber::PortAudioGrabber(int amplify) {
	this->amplify = amplify;
	init();
}

PortAudioGrabber::~PortAudioGrabber() {
	release();
}

bool PortAudioGrabber::init() {
	buffer.allocate(NUM_SAMPLES * NUM_CHANNELS * sizeof(SAMPLE));

	pa_tap_test = 1e6; //config.check("tap",Value(1e6)).asDouble();

	PaStreamParameters inputParameters; //, outputParameters;
	PaStream *stream;
	PaError err;

	err = Pa_Initialize();
	if (err != paNoError) {
		printf("portaudio system failed to initialize\n");
		return false;
	}

	inputParameters.device = Pa_GetDefaultInputDevice();
	printf("Device number %d\n", inputParameters.device);
	inputParameters.channelCount = NUM_CHANNELS;
	inputParameters.sampleFormat = PA_SAMPLE_TYPE;
	if ((Pa_GetDeviceInfo(inputParameters.device)) != 0) {
		inputParameters.suggestedLatency = Pa_GetDeviceInfo(
				inputParameters.device)->defaultLowInputLatency;
	}
	inputParameters.hostApiSpecificStreamInfo = NULL;

//	outputParameters.device = Pa_GetDefaultInputDevice();
//	outputParameters.channelCount = NUM_CHANNELS;
//	outputParameters.sampleFormat = PA_SAMPLE_TYPE;
//	//outputParameters.suggestedLatency = Pa_GetDeviceInfo( inputParameters.device )->defaultLowInputLatency;
//	outputParameters.hostApiSpecificStreamInfo = NULL;

	err = Pa_OpenStream(&stream, &inputParameters, NULL, SAMPLE_RATE,
			NUM_SAMPLES, paClipOff, NULL, NULL);
	if (err != paNoError) {
		printf("portaudio stream failed to initialize, check settings\n");
		return false;
	}

	err = Pa_StartStream(stream);
	if (err != paNoError) {
		printf("portaudio stream failed to start, check settings\n");
		return false;
	}
	printf("Reading audio data using portaudio...\n");
	fflush(stdout);
	printf("   Audio parameters:\n");
	printf("     read %d, write %d, loopback %d\n", true, false, false);
	printf("     (sampling) rate (in Hertz) %d\n", SAMPLE_RATE);
	printf("     samples (per block) %d\n", NUM_SAMPLES);
	printf("     channels %d\n", NUM_CHANNELS);
	system_resource = stream;
	return true;
}

bool PortAudioGrabber::release() {
	if (!closePortAudio()) {
		printf("PortAudio could not be closed correctly.\n");
		return false;
	}
	return true;
}

bool PortAudioGrabber::closePortAudio() {
	printf("  Closing PortAudio\n");
	PaError err;
	if (system_resource != NULL) {
		err = Pa_CloseStream((PaStream*) system_resource);
		if (err != paNoError) {
			printf("Audio error -- portaudio close failed (%s)\n",
					Pa_GetErrorText(err));
			return false;
		}
		system_resource = NULL;
	}
	return true;
}

bool PortAudioGrabber::getSound(Sound &sound) {
	sound.resize(NUM_SAMPLES, NUM_CHANNELS);
	sound.setFrequency(SAMPLE_RATE);
	PaError err;
	SAMPLE *pa_pulsecode = (SAMPLE *) buffer.get();
	err = Pa_ReadStream((PaStream*) system_resource, (void*) pa_pulsecode,
			NUM_SAMPLES);
	if (err == paInputOverflowed) {
		printf("Audio warning -- there was an input overflow (%s)\n",
				Pa_GetErrorText(err));
	} else if (err != paNoError) {
		printf("Audio error -- portaudio read failed (%s)\n",
				Pa_GetErrorText(err));
		exit(1);
	}

	int idx = 0;
	for (int i = 0; i < NUM_SAMPLES; i++) {
		for (int j = 0; j < NUM_CHANNELS; j++) {
			sound.set(pa_pulsecode[idx] * amplify, i, j);
			idx++;
		}
	}

	return true;
}

