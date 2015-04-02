/*
 * SkeletonReader.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/ObjectReader.h"

ObjectReader::ObjectReader(BufferedPort<Bottle> *in, DataCollector* parent) {
	this->in = in;
	this->parent = parent;

}

bool ObjectReader::threadInit() {
	stoploop = false;
	return true;
}

void ObjectReader::run() {
	cout<<" ObjectReader"<<endl;
	while(!stoploop) {
		bot = in->read();
		//Checking if the bottle is good
		if(bot != NULL) {
			if(bot->size() > 0) {
				//Creating a new ObjectData object and storing it in the DataCollector
				parent->addObjectData(
						new ObjectData(bot->get(26).asDouble(),
							string(bot->get(0).asString()),
							string(bot->get(24).asString()),
							bot->get(15).asDouble(),
							bot->get(16).asDouble(),
							bot->get(20).asDouble(),
							bot->get(22).asDouble(),
							bot->get(23).asDouble(),
							bot->get(18).asDouble(),
							bot->get(25).asInt()));
			}
		}
	}
}

void ObjectReader::threadRelease() {
	cout<<"  ObjectReader stopped"<<endl;
}




