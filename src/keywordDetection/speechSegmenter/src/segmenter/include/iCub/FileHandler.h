/*
 * FileHandler.h
 *
 *  Created on: Jul 4, 2012
 *      Author: cdondrup
 */

#ifndef FILEHANDLER_H_
#define FILEHANDLER_H_

#include <string>
#include <iostream>
#include <math.h>
#include <vector>

#include <sndfile.h>

#include "iCub/SFTools.h"
#include "iCub/StringConverter.h"

using namespace std;

/**
 * @brief Handling of wav files.
 *
 * This class is designed to handle exactly one wave file and its segments.
 * There has to be a FileHandler instance for every file that should be segmented.
 */
class FileHandler {
private:
	SNDFILE* sndfile;
	SF_INFO* sfinfo;
	float* wave;
	sf_count_t frames;
	int channels;
	int arraysize;
	int samplerate;
	string audiofile;
	string workdir;
	bool verbose;
	vector<string> segmentNames;

	/**
	 * Gives some file format information about encoding, compression and endien-ness.
	 * @param format The integer identifier supplied by libsndfile.
	 * @return A human readable representation of the identifier.
	 */
	string getFormat(int format);

	/**
	 * Closes the original file after reading and the filtered file after writing.
	 */
	bool close();

	/**
	 * Closes the given file. If a created file is not closed it will have missing
	 * information in its header.
	 * @param sndfile The wave file to close.
	 */
	bool close(SNDFILE* sndfile);

	/**
	 * Prints human readable information about the wave file.
	 * @param sfinfo The SF_INFO from a wave file.
	 */
	void printInfo(SF_INFO* sfinfo);

public:
	/**
	 * Creates a new FileHandler for the given wave file and the given working diretory
	 * where the created wave files will be stored.
	 * @param audiofile The wave file to load.
	 * @param workdir The directory where the results should be stored.
	 * @param verbose Iff true, prints additional information to stdio
	 */
	FileHandler(string audiofile, string workdir, bool verbose);

	/**
	 * Deletes all allocated resources.
	 */
	~FileHandler();

	/**
	 * Reads the audiofile that was given on construction.
	 * @return true if successful
	 */
	bool read();

	/**
	 * Writes the filtered original file to the working directory. The file will have the same name
	 * as the original one and will overwrite existing files!
	 * @return true if successful
	 */
	bool writeFilteredFile();

	/**
	 * Writes an extracted segment as a new file to the working directory.
	 * The file will be named "original filename"_Seg-0."original suffix". The number is unique
	 * for each segment. If there exists a file with that name in the working directory, it will
	 * be replaced.
	 * @param wave The audio data of the segment
	 * @param frames The number of frames of data
	 * @param numSeg The number of the segment. Will be included in the filename
	 * @param channels The number of channels (2 for stereo)
	 * @return true if successful
	 */
	bool writeSegment(float* wave, int frames, int numSeg, int channels);

	/**
	 * @return A vector of the segment file names
	 */
	vector<string> getSegmentNames() const {
		return segmentNames;
	}

	/**
	 * @return The sample rate of the original file
	 */
	int getSamplerate() const {
		return samplerate;
	}

	/**
	 * @param wave Replace the data of the original audio file by this new data
	 */
	void setWave(float* wave) {
		this->wave = wave;
	}

	/**
	 * @return frames * channels
	 */
	int getArraySize() const {
		return arraysize;
	}

	/**
	 * @return Number of channels (1: mono, 2: stereo)
	 */
	int getChannels() const {
		return channels;
	}

	/**
	 * @return Number of frames of data per channel
	 */
	sf_count_t getFrames() const {
		return frames;
	}

	/**
	 * @return The data of the original file
	 */
	float* getWave() const {
		return wave;
	}
};

#endif /* FILEHANDLER_H_ */
