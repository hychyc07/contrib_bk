/*
 * DataCollector.cpp
 *
 *  Created on: Jun 5, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/DataCollector.h"

void DataCollector::addObjectData(ObjectData* objectData) {
	//Lock the data collection
	mutex.wait();
	//Insert the new object data.
	//Fails if there already is a LDData object with that time stamp.
	ret = data.insert(
			pair<double, LDData>(objectData->getTimestamp(),
					LDData(objectData)));
	//If the insertion failed -> look for the LDData object with the given time stamp
	//and add the new ObjectData object.
	if (ret.second == false) {
		it = data.find(objectData->getTimestamp());
		it->second.setObjectData(objectData);
	}
	//Free the data lock
	mutex.post();
	//Send callback
	sendCallback(&(data.find(objectData->getTimestamp())->second));
}

void DataCollector::addSkeletonData(SkeletonData* skeletonData) {
	//Lock the data collection
	mutex.wait();
	//Insert the new skeleton data.
	//Fails if there already is a LDData object with that time stamp.
	ret = data.insert(
			pair<double, LDData>(skeletonData->getTimestamp(),
					LDData(skeletonData)));
	//If the insertion failed -> look for the LDData object with the given time stamp
	//and add the new SkeletonData object.
	if (ret.second == false) {
		it = data.find(skeletonData->getTimestamp());
		it->second.setSkeletonData(skeletonData);
	}
	//Free the data lock
	mutex.post();
	//Send callback
	sendCallback(&(data.find(skeletonData->getTimestamp())->second));
}

LDData* DataCollector::getLatestSkeletonAndObject() {
	//Loop backwards through data list and return first element with skeleton and object data.
	for(it = data.end();it != data.begin(); --it) {
		if(it->second.isSkeleton() && it->second.isObject())
			return &it->second;
	}
	//Otherwise return NULL
	return NULL;
}

LDData* DataCollector::getLatestSkeletonData() {
	//Loop backwards through data list and return first element with skeleton data.
	for (it = data.end(); it != data.begin(); --it) {
		if (it->second.isSkeleton())
			return &it->second;
	}
	//Otherwise return NULL
	return NULL;
}

LDData* DataCollector::getLatestObjectData() {
	//Loop backwards through data list and return first element with object data.
	for (it = data.end(); it != data.begin(); --it) {
		if (it->second.isObject())
			return &it->second;
	}
	//Otherwise return NULL
	return NULL;
}

void DataCollector::sendCallback(LDData* lddata) {
	//Sends a callback depending on which data is present in the current LDData object
	if(lddata->isSkeleton() && lddata->isObject()) {
		parent->receivedSkeletonAndObject(lddata);
	}
	if (lddata->isSkeleton()) {
		parent->receivedSkeleton(lddata);
	}
	if (lddata->isObject()) {
		parent->receivedObject(lddata);
	}
	parent->receivedAnyThing();
}

