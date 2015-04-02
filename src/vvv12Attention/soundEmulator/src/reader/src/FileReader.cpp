/*
 * FileReader.cpp
 *
 *  Created on: Jul 24, 2012
 *      Author: cdondrup
 */

#include "iCub/FileReader.h"

using namespace std;
using namespace yarp::sig;

bool FileReader::readFromFile(string path) {
	currentPos = 0;
	in.resize(0,0);
	file::read(in,path.c_str());
	cout<<in.getSamples()<<endl;
	file::write(in,string(path+"_test.wav").c_str());
	return true;
}

Sound* FileReader::getChunk(Sound* result) {
	result->resize(CHUNK_SIZE,in.getChannels());
	if(currentPos+CHUNK_SIZE > in.getSamples())
		currentPos = 0;
	for(int i = currentPos; i < currentPos+CHUNK_SIZE; i++) {
		result->setSafe(in.getSafe(i,0),i-currentPos,0);
		if(in.getChannels() == 2)
			result->setSafe(in.getSafe(i,1),i-currentPos,1);
	}
	currentPos += CHUNK_SIZE;
	return result;
}




