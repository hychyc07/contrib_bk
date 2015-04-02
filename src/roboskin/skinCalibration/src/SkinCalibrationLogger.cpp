#include "iCub/skinCalibration/SkinCalibrationLogger.h"

using namespace iCub::skinCalibration;

FILE * SkinCalibrationLogger::stream_pointer = '\0';
string SkinCalibrationLogger::context_label = "";

int SkinCalibrationLogger::baseWrite(FILE *stream, string format, va_list &args){
	int ret = -1;
	if(stream != '\0'){
		string msg;
		msg = context_label+format;
		ret = vfprintf(stream, msg.c_str(), args);
	}
	return ret;
}

void SkinCalibrationLogger::setContextLabel(string str){
	context_label = str;
}

void SkinCalibrationLogger::setStreamToStdOut(){
	stream_pointer = stdout;
}

void SkinCalibrationLogger::setStreamToStdErr(){
	stream_pointer = stderr;
}

void SkinCalibrationLogger::setStreamToFile(string fname){
	stream_pointer = fopen(fname.c_str(),"a");
}

bool SkinCalibrationLogger::isInitialized(){
	return stream_pointer != '\0';
}

int SkinCalibrationLogger::writeMsg(string format, ...){
	va_list args;
	int ret = -1;
	va_start(args, format);
	ret = baseWrite(stream_pointer, format, args);
	va_end(args);
	return ret;
}

int SkinCalibrationLogger::writeToStdOut(string format, ...){
	va_list args;
	int ret = -1;
	va_start(args, format);
	ret = baseWrite(stdout, format, args);
	va_end(args);
	return ret;
}

int SkinCalibrationLogger::writeToStdErr(string format, ...){
	va_list args;
	int ret = -1;
	va_start(args, format);
	ret = baseWrite(stderr, format, args);
	va_end(args);
	return ret;
}

int SkinCalibrationLogger::writeToFile(string filename, string format, ...){
	va_list args;
	int ret = -1;
	FILE * fstrm = fopen(filename.c_str(),"a");
	if (fstrm!=NULL)
	{
		va_start(args, format);
		ret = baseWrite(stdout, format, args);
		va_end(args);
		fclose(fstrm);
	}
	return ret;
}