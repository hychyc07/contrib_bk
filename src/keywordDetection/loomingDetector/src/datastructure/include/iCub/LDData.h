/*
 * LDData.h
 *
 *  Created on: Jun 5, 2012
 *      Author: Christian Dondrup
 */

#ifndef LDDATA_H_
#define LDDATA_H_

#include <string>

#include "iCub/ObjectData.h"
#include "iCub/SkeletonData.h"

using namespace std;

/**
 * This is a data container for one ObjectData and one SkeletonData object with the same time stamp.
 */
class LDData {
private:
	ObjectData *objectData;
	SkeletonData *skeletonData;

public:
	/**
	 * Creates a new LDData Object with an empty ObjectData and an empty SkeletoData object.
	 */
	LDData()
	{
		objectData = new ObjectData();
		skeletonData = new SkeletonData();
	}

	/**
	 * Creates a new LDData object with an empty SkeletonData object and the given ObjectData object.
	 * @param objectData ObjectData object
	 */
	LDData(ObjectData* objectData)
		:objectData(objectData)
	{
		skeletonData = new SkeletonData();
	}

	/**
	 * Creates a new LDData object with an empty ObjectData object and the given SkeletonData object.
	 * @param skeletonData SkeletonData object
	 */
	LDData(SkeletonData *skeletonData)
		:skeletonData(skeletonData)
	{
		objectData = new ObjectData();
	}

	/**
	 * Creates a new LDData Object with the given ObjectData and the given SkeletonData objects.
	 * @param objectData ObjectData object
	 * @param skeletonData SkeletonData object
	 */
	LDData(ObjectData *objectData, SkeletonData *skeletonData)
		:objectData(objectData),
		 skeletonData(skeletonData){}

	/**
	 * @return The associated ObjectData object
	 */
	ObjectData* getObjectData() const {
		return objectData;
	}

	/**
	 * Set a new ObjectData object.
	 * @param objectData The new ObjectData object
	 */
	void setObjectData(ObjectData* objectData) {
		this->objectData = objectData;
	}

	/**
	 * @return The associated SkeletonData object
	 */
	SkeletonData* getSkeletonData() const {
		return skeletonData;
	}

	/**
	 * Set a new SkeletonData object.
	 * @param skeletonData The new SkeletonData object
	 */
	void setSkeletonData(SkeletonData* skeletonData) {
		this->skeletonData = skeletonData;
	}

	/**
	 * @return true, if the SkeletonData object is not empty
	 */
	bool isSkeleton() {
		return this->skeletonData->getTimestamp() != 0.0;
	}

	/**
	 * @return true, if the ObjectData object is not empty
	 */
	bool isObject() {
		return this->objectData->getTimestamp() != 0.0;
	}

	/** Calls the toString() function of the ObjectData and SkeletonData instances
	 * included in this object and returns a string representation of both.
	 * @return A string containing all data of ObjectData and SkeletonData
	 */
	string toString() {
        string result = "Sensor Data:";

        result += "\n";
        result += getObjectData()->toString();
        result += "\n";
        result += getSkeletonData()->toString();
        result += "\n";

        return result;
    }

};



#endif /* LDDATA_H_ */
