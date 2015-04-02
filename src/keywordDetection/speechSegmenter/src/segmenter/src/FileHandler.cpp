/*
 * FileHandler.cpp
 *
 *  Created on: Jul 4, 2012
 *      Author: cdondrup
 */

#include "iCub/FileHandler.h"

using namespace std;

FileHandler::FileHandler(string audiofile, string workdir, bool verbose) {
	//Creating new SF_INFO with format = 0. Has to be in order to work properly
	this->verbose = verbose;
	sfinfo = new SF_INFO();
	sfinfo->format = 0;
	wave = NULL;
	this->audiofile = audiofile;
	this->workdir = workdir;
}

FileHandler::~FileHandler() {
	delete[] wave;
}

bool FileHandler::read() {
	sndfile = sf_open(audiofile.c_str(), SFM_READ, sfinfo);
	if (sf_error(sndfile)) {
		cerr << sf_strerror(sndfile) << endl;
		return false;
	}

	//Storing values that might be overridden in the sfinfo
	frames = sfinfo->frames;
	channels = sfinfo->channels;
	arraysize = frames * channels;
	samplerate = sfinfo->samplerate;

	if (verbose) {
		cout << "Read: " << audiofile << " with options:" << endl;
		printInfo(sfinfo);
	}
	if(verbose)
		cout << endl << "Storing wave file to float array:" << endl;
	if (wave)
		delete[] wave;
	//Readinf sudio data into a float array
	wave = new float[arraysize];
	sf_count_t check = sf_readf_float(sndfile, wave, frames);
	if (verbose)
		cout << "Successfully read: " << check << "/" << frames << " frames"
				<< endl << endl;

	return close();
}

bool FileHandler::writeFilteredFile() {
	string filename = workdir + "/"
			+ audiofile.substr(audiofile.find_last_of("/") + 1,
					audiofile.find_last_of(".")
							- (audiofile.find_last_of("/") + 1))
			+ "_filtered"
			+ audiofile.substr(audiofile.find_last_of("."));
	if (!sf_format_check(sfinfo)) {
		cerr << "Specified format is invalid!" << endl;
		return false;
	}
	sndfile = sf_open(filename.c_str(), SFM_WRITE, sfinfo);
	if (sf_error(sndfile)) {
		cerr << sf_strerror(sndfile) << endl;
		return false;
	}
	if (verbose) {
		cout << "Writing: " << filename << " with options:" << endl;
		printInfo(sfinfo);
	} else {
		cout << "Writing: " << filename << endl;
	}
	sf_count_t check = sf_writef_float(sndfile, wave, frames);
	if (verbose)
		cout << "Successfully wrote: " << check << "/" << frames << " frames"
				<< endl << endl;
	sf_write_sync(sndfile);
	return close();
}

bool FileHandler::writeSegment(float* wave, int frames, int numSeg,
		int channels) {
	string filename = workdir + "/"
			+ audiofile.substr(audiofile.find_last_of("/") + 1,
					audiofile.find_last_of(".")
							- (audiofile.find_last_of("/") + 1)) + "_Seg-"
			+ to_string<int>(numSeg)
			+ audiofile.substr(audiofile.find_last_of("."));
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
	if (verbose)
		cout << "Successfully wrote: " << check << "/" << frames << " frames"
				<< endl << endl;
	sf_write_sync(segment);
	segmentNames.push_back(filename);
	if (verbose) {
		cout << "Writing: " << segmentNames.at(segmentNames.size()-1) << " with options:" << endl;
		printInfo(seginfo);
	} else {
		cout << "Writing: " << segmentNames.at(segmentNames.size()-1) << endl;
	}

	return close(segment);
}

bool FileHandler::close() {
	int err = sf_close(sndfile);
	if (err) {
		cerr << sf_error_number(err) << endl;
		return false;
	}
	return true;
}

bool FileHandler::close(SNDFILE* sndfile) {
	int err = sf_close(sndfile);
	if (err) {
		cerr << sf_error_number(err) << endl;
		return false;
	}
	return true;
}

void FileHandler::printInfo(SF_INFO* sfinfo) {
	cout << "Channels: " << sfinfo->channels << endl << "Format: "
			<< getFormat(SF_CONTAINER(sfinfo->format)) << endl << "Codec: "
			<< getFormat(SF_CODEC(sfinfo->format)) << endl << "Endian: "
			<< getFormat(SF_ENDIAN(sfinfo->format)) << endl << "Frames: "
			<< sfinfo->frames << endl << "Samplerate: " << sfinfo->samplerate
			<< endl << "Sections: " << sfinfo->sections << endl << "Seekable: "
			<< (sfinfo->seekable ? "true" : "false") << endl << endl;
	;
}

string FileHandler::getFormat(int format) {
	string result = "";
	switch (format) {
	case SF_FORMAT_WAV:
		result = "SF_FORMAT_WAV: Microsoft WAV format (little endian default).";
		break;
	case SF_FORMAT_AIFF:
		result = "SF_FORMAT_AIFF: Apple/SGI AIFF format (big endian).";
		break;
	case SF_FORMAT_AU:
		result = "SF_FORMAT_AU: Sun/NeXT AU format (big endian).";
		break;
	case SF_FORMAT_RAW:
		result = "SF_FORMAT_RAW: RAW PCM data.";
		break;
	case SF_FORMAT_PAF:
		result = "SF_FORMAT_PAF: Ensoniq PARIS file format.";
		break;
	case SF_FORMAT_SVX:
		result = "SF_FORMAT_SVX: Amiga IFF / SVX8 / SV16 format.";
		break;
	case SF_FORMAT_NIST:
		result = "SF_FORMAT_NIST: Sphere NIST format.";
		break;
	case SF_FORMAT_VOC:
		result = "SF_FORMAT_VOC: VOC files.";
		break;
	case SF_FORMAT_IRCAM:
		result = "SF_FORMAT_IRCAM: Berkeley/IRCAM/CARL";
		break;
	case SF_FORMAT_W64:
		result = "SF_FORMAT_W64: Sonic Foundry's 64 bit RIFF/WAV";
		break;
	case SF_FORMAT_MAT4:
		result = "SF_FORMAT_MAT4: Matlab (tm) V4.2 / GNU Octave 2.0";
		break;
	case SF_FORMAT_MAT5:
		result = "SF_FORMAT_MAT5: Matlab (tm) V5.0 / GNU Octave 2.1";
		break;
	case SF_FORMAT_PVF:
		result = "SF_FORMAT_PVF: Portable Voice Format";
		break;
	case SF_FORMAT_XI:
		result = "SF_FORMAT_XI: Fasttracker 2 Extended Instrument";
		break;
	case SF_FORMAT_HTK:
		result = "SF_FORMAT_HTK: HMM Tool Kit format";
		break;
	case SF_FORMAT_SDS:
		result = "SF_FORMAT_SDS: Midi Sample Dump Standard";
		break;
	case SF_FORMAT_AVR:
		result = "SF_FORMAT_AVR: Audio Visual Research";
		break;
	case SF_FORMAT_WAVEX:
		result = "SF_FORMAT_WAVEX: MS WAVE with WAVEFORMATEX";
		break;
	case SF_FORMAT_SD2:
		result = "SF_FORMAT_SD2: Sound Designer 2";
		break;
	case SF_FORMAT_FLAC:
		result = "SF_FORMAT_FLAC: FLAC lossless file format";
		break;
	case SF_FORMAT_CAF:
		result = "SF_FORMAT_CAF: Core Audio File format";
		break;
	case SF_FORMAT_WVE:
		result = "SF_FORMAT_WVE: Psion WVE format";
		break;
	case SF_FORMAT_OGG:
		result = "SF_FORMAT_OGG: Xiph OGG container";
		break;
	case SF_FORMAT_MPC2K:
		result = "SF_FORMAT_MPC2K: Akai MPC 2000 sampler";
		break;
	case SF_FORMAT_RF64:
		result = "SF_FORMAT_RF64: RF64 WAV file";
		break;

		/* Subtypes from here on. */

	case SF_FORMAT_PCM_S8:
		result = "SF_FORMAT_PCM_S8: Signed 8 bit data";
		break;
	case SF_FORMAT_PCM_16:
		result = "SF_FORMAT_PCM_16: Signed 16 bit data";
		break;
	case SF_FORMAT_PCM_24:
		result = "SF_FORMAT_PCM_24: Signed 24 bit data";
		break;
	case SF_FORMAT_PCM_32:
		result = "SF_FORMAT_PCM_32: Signed 32 bit data";
		break;

	case SF_FORMAT_PCM_U8:
		result = "SF_FORMAT_PCM_U8: Unsigned 8 bit data (WAV and RAW only)";
		break;

	case SF_FORMAT_FLOAT:
		result = "SF_FORMAT_FLOAT: 32 bit float data";
		break;
	case SF_FORMAT_DOUBLE:
		result = "SF_FORMAT_DOUBLE: 64 bit float data";
		break;

	case SF_FORMAT_ULAW:
		result = "SF_FORMAT_ULAW: U-Law encoded.";
		break;
	case SF_FORMAT_ALAW:
		result = "SF_FORMAT_ALAW: A-Law encoded.";
		break;
	case SF_FORMAT_IMA_ADPCM:
		result = "SF_FORMAT_IMA_ADPCM: IMA ADPCM.";
		break;
	case SF_FORMAT_MS_ADPCM:
		result = "SF_FORMAT_MS_ADPCM: Microsoft ADPCM.";
		break;

	case SF_FORMAT_GSM610:
		result = "SF_FORMAT_GSM610: GSM 6.10 encoding.";
		break;
	case SF_FORMAT_VOX_ADPCM:
		result = "SF_FORMAT_VOX_ADPCM: OKI / Dialogix ADPCM";
		break;

	case SF_FORMAT_G721_32:
		result = "SF_FORMAT_G721_32: 32kbs G721 ADPCM encoding.";
		break;
	case SF_FORMAT_G723_24:
		result = "SF_FORMAT_G723_24: 24kbs G723 ADPCM encoding.";
		break;
	case SF_FORMAT_G723_40:
		result = "SF_FORMAT_G723_40: 40kbs G723 ADPCM encoding.";
		break;

	case SF_FORMAT_DWVW_12:
		result =
				"SF_FORMAT_DWVW_12: 12 bit Delta Width Variable Word encoding.";
		break;
	case SF_FORMAT_DWVW_16:
		result =
				"SF_FORMAT_DWVW_16: 16 bit Delta Width Variable Word encoding.";
		break;
	case SF_FORMAT_DWVW_24:
		result =
				"SF_FORMAT_DWVW_24: 24 bit Delta Width Variable Word encoding.";
		break;
	case SF_FORMAT_DWVW_N:
		result = "SF_FORMAT_DWVW_N: N bit Delta Width Variable Word encoding.";
		break;

	case SF_FORMAT_DPCM_8:
		result = "SF_FORMAT_DPCM_8: 8 bit differential PCM (XI only)";
		break;
	case SF_FORMAT_DPCM_16:
		result = "SF_FORMAT_DPCM_16: 16 bit differential PCM (XI only)";
		break;

	case SF_FORMAT_VORBIS:
		result = "SF_FORMAT_VORBIS: Xiph Vorbis encoding.";
		break;

		/* Endian-ness options. */

	case SF_ENDIAN_FILE:
		result = "SF_ENDIAN_FILE: Default file endian-ness.";
		break;
	case SF_ENDIAN_LITTLE:
		result = "SF_ENDIAN_LITTLE: Force little endian-ness.";
		break;
	case SF_ENDIAN_BIG:
		result = "SF_ENDIAN_BIG: Force big endian-ness.";
		break;
	case SF_ENDIAN_CPU:
		result = "SF_ENDIAN_CPU: Force CPU endian-ness.";
		break;
	}
	return result;
}

