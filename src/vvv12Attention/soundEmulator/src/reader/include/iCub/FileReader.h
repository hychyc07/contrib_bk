/*
 * FileReader.h
 *
 *  Created on: Jul 24, 2012
 *      Author: cdondrup
 */

#ifndef FILEREADER_H_
#define FILEREADER_H_

#include <string>
#include <iostream>

#include <yarp/sig/all.h>

#define CHUNK_SIZE 512

using namespace std;
using namespace yarp::sig;

class FileReader {
private:
	Sound in;
	int currentPos;
public:
	bool readFromFile(string path);
	Sound* getChunk(Sound* result);
};




#endif /* FILEREADER_H_ */
