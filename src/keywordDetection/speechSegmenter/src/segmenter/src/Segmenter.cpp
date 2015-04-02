/*
 * Segmenter.cpp
 *
 *  Created on: Jul 11, 2012
 *      Author: cdondrup
 */

#include "iCub/Segmenter.h"

using namespace std;

bool Segmenter::segmentSignal() {
	//Preprocessing audio data
	if (!signalProcessor->process(fileHandler))
		return false;

	if (signalProcessor->isStereo()) {
		//Segmenting left an right channel of stereo file
		vector<pair<pair<int, int>, float*> > leftSeg = segmentWave(
				signalProcessor->getLeftFiltered(), fileHandler->getFrames());
		vector<pair<pair<int, int>, float*> > rightSeg = segmentWave(
				signalProcessor->getRightFiltered(), fileHandler->getFrames());

		if (choose == 0 && leftSeg.size() != rightSeg.size()) {
			cerr << "ERROR: Audio channels seem to be different." << endl;
			return false;
		}

		if (choose == -1) {
			if (leftSeg.size() < rightSeg.size()) {
				rightSeg = leftSeg;
			} else if (leftSeg.size() > rightSeg.size()) {
				leftSeg = rightSeg;
			}
		} else if (choose == 1) {
			if (leftSeg.size() > rightSeg.size()) {
				rightSeg = leftSeg;
			} else if (leftSeg.size() < rightSeg.size()) {
				leftSeg = rightSeg;
			}
		}

		//Creating final segment vector
		vector<pair<pair<int, int>, float*> > segments;
		for (int i = 0; i < leftSeg.size(); i++) {
			//Getting the maximal time stamp of both channels for this segment
			pair<int, int> time(
			MIN(leftSeg[i].first.first,rightSeg[i].first.first)
			, MAX(leftSeg[i].first.second,rightSeg[i].first.second));
			if (verbose) {
				cout << "Left: " << leftSeg[i].first.first << " - "
						<< leftSeg[i].first.second << " Right: "
						<< rightSeg[i].first.first << " - "
						<< rightSeg[i].first.second << endl;
				cout << "Boundaries: " << time.first << " - " << time.second
						<< endl;
			}
			int length = time.second - time.first;
			//Getting data of segment
			float** data = new float*[2];
			data[0] = new float[length];
			data[1] = new float[length];
			for (int j = time.first; j < time.second; j++) {
				data[0][j - time.first] =
						fromFiltered ?
								signalProcessor->getLeftFiltered()[j] :
								signalProcessor->getLeftOriginal()[j];
				data[1][j - time.first] =
						fromFiltered ?
								signalProcessor->getRightFiltered()[j] :
								signalProcessor->getRightOriginal()[j];
			}
			//Mixing both channels to one file
			float* mix = new float[length * 2];
			long j = 0;
			for (long k = 0; k < length; k++) {
				mix[j++] = data[0][k];
				mix[j++] = data[1][k];
			}
			segments.push_back(pair<pair<int, int>, float*>(time, mix));
			delete[] data;
		}
		//Writing segments to disk
		for (int i = 0; i < segments.size(); i++) {
			fileHandler->writeSegment(segments[i].second,
					segments[i].first.second - segments[i].first.first, i, 2);
		}
	} else {
		//Segmenting audio data
		vector<pair<pair<int, int>, float*> > segments = segmentWave(
				signalProcessor->getFiltered(), fileHandler->getFrames());

		for (int i = 0; i < segments.size(); i++) {
//			cout << "Segment " << i << ": " << segments[i].first.first << " - "
//					<< segments[i].first.second << endl;
			//Getting segment data
			for (int j = segments[i].first.first; j < segments[i].first.second;
					j++) {
				segments[i].second[j - segments[i].first.first] =
						signalProcessor->getLeftOriginal()[j];
			}
			//Writing segment to disk
			fileHandler->writeSegment(segments[i].second,
					segments[i].first.second - segments[i].first.first, i, 1);
		}
	}
	return true;
}

vector<pair<pair<int, int>, float*> > Segmenter::segmentWave(float* wave,
		int totalLength) {
	//Create filtered entropy profile
	vector<float> entropies = entropyProfile(wave, totalLength);
	float threshold = calculateThreshold(entropies);
	if (verbose)
		cout << "Threshold: " << threshold << endl;
	entropies = filterByThreshold(entropies, threshold);

	if (verbose) {
		cout << "Entropy profile of one channel:" << endl;
		float frame = 1000
				/ (float(fileHandler->getSamplerate()) / float(ENTROPY_WINDOW));
		float hop = 1000
				/ (float(fileHandler->getSamplerate())
						/ float(ENTROPY_WINDOW_HOP));
		for (int i = 0; i < entropies.size(); i++) {
			cout << i * hop << "\t- " << (i * hop) + frame << ":\t"
					<< entropies[i] << "," << endl;
		}
		cout << endl;
	}

	//Define segmentation parameters
	int /*lambda = 3,*/length = 0; //Lambda = minimal length of an utterance to be considered speech
	int /*delta = 3,*/distance = 0; //delta = maximal distance between two utterances to be considered as one
	int start = 0, end = 0;
	vector<pair<pair<int, int>, float*> > segments;

	//finding start and end points of speech segments
	for (int i = 0; i < entropies.size(); i++) {
		if (entropies[i] > 0) {
			length++;
			distance = 0;
			start = start > 0 ? start : i * ENTROPY_WINDOW_HOP;
		} else {
			distance++;
			if (length > lambda) {
				end = ((i - 1) * ENTROPY_WINDOW_HOP) + ENTROPY_WINDOW;
			}
			if (distance >= delta) {
				if (start && end)
					segments.push_back(
							pair<pair<int, int>, float*>(
									pair<int, int>(start, end),
									new float[end - start]));
				start = 0;
				end = 0;
			} else if (i == entropies.size() - 1) {
				if (start && end)
					segments.push_back(
							pair<pair<int, int>, float*>(
									pair<int, int>(start, end),
									new float[end - start]));
			}
			length = 0;
		}

		//		cout << i * hop << " - " << (i * hop) + frame << ":\t" << entropies[i]
		//				<< ", " << "length: " << length << " distance: " << distance
		//				<< " Start: " << start << " End: " << end << endl;
	}
	return segments;
}

vector<float> Segmenter::entropyProfile(float* signal, int signalLength) {
	//Finding histogram parameters
	float min = FLT_MAX;
	float max = FLT_MIN;
	for (int k = 0; k < signalLength; k++) {
		min = MIN(signal[k], min);
		max = MAX(signal[k], max);
	}
	float range = max - min;
	float interval = range / HISTOGRAM_BINS;

	//Filling histogram bins
	vector<float*> histograms;
	for (int i = 0; i < signalLength - ENTROPY_WINDOW; i +=
	ENTROPY_WINDOW_HOP) {
		float* tmp = new float[HISTOGRAM_BINS];
		for (int k = 0; k < HISTOGRAM_BINS; k++)
			tmp[k] = 0.0;
		for (int j = 0; j < ENTROPY_WINDOW; j++) {
			for (int k = 0; k < HISTOGRAM_BINS; k++) {
				if (signal[i + j] < min + (k * interval)) {
					tmp[k]++;
					break;
				}
			}
		}
		for (int k = 0; k < HISTOGRAM_BINS; k++) {
			tmp[k] /= ENTROPY_WINDOW;
		}
		histograms.push_back(tmp);
	}

	//Calculating entropy for histograms
	vector<float> result;
	for (int i = 0; i < histograms.size(); i++) {
		result.push_back(computeEntropy(histograms[i]));
	}

	return result;
}

float Segmenter::computeEntropy(float* histogram) {
	float result = 0.0;
	for (int i = 0; i < HISTOGRAM_BINS; i++) {
		if (histogram[i] == 0.0)
			continue;
		result -= histogram[i] * log2(histogram[i]);
	}
	return result;
}

float Segmenter::calculateThreshold(vector<float> entropies) {
	float min = FLT_MAX;
	float max = FLT_MIN;
	for (int k = 0; k < entropies.size(); k++) {
		min = MIN(entropies[k], min);
		max = MAX(entropies[k], max);
	}

	return ((max - min) / 2) + (mu * min);
}

vector<float> Segmenter::filterByThreshold(vector<float> entropies,
		float threshold) {
	for (int i = 0; i < entropies.size(); i++)
		entropies[i] = entropies[i] > threshold ? entropies[i] : 0.0;
	return entropies;
}

vector<string> Segmenter::getSegmentNames() {
	return fileHandler->getSegmentNames();
}

