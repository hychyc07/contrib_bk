/*
 * ChunkSendere.cpp
 *
 *  Created on: Jul 24, 2012
 *      Author: cdondrup
 */

#include "iCub/ChunkSender.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

bool ChunkSender::send() {
	return true;
}

bool ChunkSender::threadInit() {
	return fileReader.readFromFile(filename);
}

void ChunkSender::threadRelease() {
	return;
}

void ChunkSender::run() {
	cout<<"running";
	while (!stoploop) {
		Sound* sound = &out->prepare();
		fileReader.getChunk(sound);
		sound->setFrequency(48000);
//		for(int i = 0; i < CHUNK_SIZE; i++) {
//			cout<<sound->getSafe(i, 0)<<", ";
//		}
//		cout<<endl;
		out->write();
		usleep(11000);
	}
	cout<<endl;
}

