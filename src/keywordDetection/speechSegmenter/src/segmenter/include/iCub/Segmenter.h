/*
 * Segmenter.h
 *
 *  Created on: Jul 11, 2012
 *      Author: cdondrup
 */

#ifndef SEGMENTER_H_
#define SEGMENTER_H_

#include <float.h>
#include <vector>
#include <math.h>
#include <string>

#include "iCub/FileHandler.h"
#include "iCub/SignalProcessor.h"

#define ENTROPY_WINDOW 1024
#define ENTROPY_WINDOW_NONOVERLAP 0.75
#define ENTROPY_WINDOW_HOP ENTROPY_WINDOW*ENTROPY_WINDOW_NONOVERLAP
#define HISTOGRAM_BINS 100

#define MIN(X,Y) X < Y ? X : Y
#define MAX(X,Y) X > Y ? X : Y

using namespace std;

/**
 * @brief Segments audio data into segments of speech.
 */
class Segmenter {
private:
	FileHandler* fileHandler;
	SignalProcessor* signalProcessor;
	bool verbose, fromFiltered;
	int lambda, delta;
	int choose;
	float mu;

	/**
	 * Computes the entropy of a given histogram. The entropy will be used in finding
	 * a threshold between noise and speech.
	 * @param histogram The histogram to compute the entropy of
	 * @return The entropy value for the histogram
	 */
	float computeEntropy(float* histogram);

	/**
	 * Calculates a threshold between noise and speech given the entropies of
	 * all histograms over the complete data.
	 * Threshold = ((max(entropy) - min(entropy)) / 2) + (mu * min(entropy)); mu > 0
	 * @param entropies A vector of all computed entropies
	 * @return The threshold
	 */
	float calculateThreshold(vector<float> entropies);

	/**
	 * Filters the entropy vector by the given threshold. All values below the threshold will
	 * be set to zero.
	 * @param entropies The entropy vector of the complete data
	 * @param threshold The threshold to filter the data by
	 * @return The filtered entropy vector
	 */
	vector<float> filterByThreshold(vector<float> entropies, float threshold);

	/**
	 * Divides the given audio data into overlapping windows of ENTROPY_WINDOW size with an overlap of
	 * 1 - ENTROPY_WINDOW_NONOVERLAP. Constructs a histogram with HISTOGRAM_BINS bins and sorts
	 * the float values of the data into the corresponding bins.
	 * Afterwards computeEntropy() is used to calculate the entropy for all the windows and create
	 * an entropy profile for the complete data.
	 * If the entropy is low, then the data was widespread over all bins of the histogram. This implies
	 * noise.
	 * If the there are few bins containing much of the data, then there was a significant spike in
	 * the wave and the entropy will be high. This implies speech.
	 * @param signal The data to create an entropy profile of
	 * @param signalLength The size of the signal array
	 * @return A vector containing the entropies for all the histograms
	 */
	vector<float> entropyProfile(float* signal, int signalLength);

	/**
	 * Uses entropyProfile(), calculateThreshold() and filterByThreshold() to segment a given
	 * wave of audio.
	 * @param wave The audio data to segment.
	 * @param totalLength The length of the wave array.
	 * @return A vector of a pair of a pair the start and end frame and the segmented audio data
	 */
	vector<pair<pair<int, int>, float*> > segmentWave(float* wave,
			int totalLength);

public:
	/**
	 * Creates a new Segmenter object. There has to be one Segmenter instance per audio file
	 * to segment.
	 * Uses the FileHandler to read and write files.
	 * Uses the SignalProcessor to preprocess the audio file for segmentation.
	 * @param fileHandler A FileHandler object containing the audio file to segment
	 * @param signalProcessor A SignalProcessor object to preprocess the audio file
	 * @param lambda The minimal number of speech blocks to be considered a speech segment
	 * @param delta The maximal distance between two speech segments to be considered one segment
	 * @param choose Either -1,0,1 chooses the result with less segments if -1 and the result
	 * with more segment if 1. If 0 throws error if not both stereo channel return the
	 * same number of segments.
	 * @param fromFiltered Iff true creates the segment files from the filtered original
	 * @param verbose Iff true, prints additional information to stdio
	 */
	Segmenter(FileHandler* fileHandler, SignalProcessor* signalProcessor,
			int lambda, int delta, float mu, int choose, bool fromFiltered, bool verbose) :
			fileHandler(fileHandler), signalProcessor(signalProcessor), lambda(
					lambda), delta(delta), verbose(verbose), choose(choose), fromFiltered(
					fromFiltered), mu(mu) {
	}

	/**
	 * This function calls the SignalProcessor with the given FileHandler to preprocess
	 * the data and then all needed private functions to segment the wave file.
	 * Afterwards it will call the FileHandler to write the segments and the filtered original file
	 * to the working directory.
	 * @return true if successful.
	 */
	bool segmentSignal();

	vector<string> getSegmentNames();
};

#endif /* SEGMENTER_H_ */
