/*
 * DataCollector.h
 *
 *  Created on: Jun 5, 2012
 *      Author: Christian Dondrup
 */

#ifndef DATACOLLECTOR_H_
#define DATACOLLECTOR_H_

#include <map>

#include <yarp/os/all.h>

#include "iCub/Callback.h"

using namespace std;
using namespace yarp::os;

/**
 * This is a data storage class which contains and manages all collected data.
 */
class DataCollector {
private:
	Callback* parent;
	map<double,LDData> data;
	map<double,LDData>::iterator it;
	pair<map<double,LDData>::iterator,bool> ret;
	Semaphore mutex;

public:

	/**
	 * Creates a new DataCollector with the given parent class.
	 * You should only use one DataCollector throughout the entire project.
	 * @param parent Instance of the calling class. Must inherit Callback.
	 */
	DataCollector(Callback* parent)
	:parent(parent), mutex(1){}

	/**
	 * Adds a new ObjectData object to the data collection.
	 * Tries to match the given ObjectData with previously added SkeletonData objects
	 * and matches them if the timestamp is equal.
	 * Calls the sendCallback function.
	 * @param objectData New ObjectData object which hasn't been added to the data collection yet.
	 */
	void addObjectData(ObjectData* objectData);

	/**
	 * Adds a new SkeletonData object to the data collection.
	 * Tries to match the given SkeletonData with previously added ObjectData objects
	 * and matches them if the timestamp is equal.
	 * Calls the sendCallback function.
	 * @param skeletonData New SkeletonData object which hasn't been added to the data collection yet.
	 */
	void addSkeletonData(SkeletonData* skeletonData);

	/**
	 * @return The LDData object with not empty SkeletonData and ObjectData with the highest timestamp
	 */
	LDData* getLatestSkeletonAndObject();

	/**
	 * @return The LDData object with not empty SkeletonData with the highest timestamp
	 */
	LDData* getLatestSkeletonData();

	/**
	 * @return The LDData object with not empty ObjectData with the highest timestamp
	 */
	LDData* getLatestObjectData();

	/**
	 * Chooses the appropriate callback to send to the parent.
	 * Multiple callbacks can be fired in the following order:
	 * * Skeleton and Object
	 * * Skeleton
	 * * Object
	 * * Anything
	 * @param lddata The LDData object which should trigger a callback
	 */
	void sendCallback(LDData* lddata);
};




#endif /* DATACOLLECTOR_H_ */
