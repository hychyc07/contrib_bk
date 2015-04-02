/*
 * Callback.h
 *
 *  Created on: Jun 5, 2012
 *      Author: Christian Dondrup
 */

#ifndef CALLBACK_H_
#define CALLBACK_H_

#include "iCub/LDData.h"

/**
 * Interface which must be extended by the data manipulating class.
 */
class Callback {
public:
	/**
	 * Virtual destructor to symbolize that this is pure viurtual class
	 */
	virtual ~Callback() {}

	/**
	 * Will be called if there is a LDData object containing a SkeletonData
	 * and an ObjectData object with the same timestamp.
	 * @param data The newest LDData object containing the appropriate SkeltonData and ObjectData
	 */
	virtual void receivedSkeletonAndObject(LDData* data)=0;

	/**
	 * Will be called if there is a new LDData object containing a SkeletonData object.
	 * @param data The newest LDData object containing the appropriate SkeltonData
	 */
	virtual void receivedSkeleton(LDData* data, bool object = false)=0;

	/**
	 * Will be called if there is a new LDData object containing an ObjectData object.
	 * @param data The newest LDData object containing the appropriate ObjectData
	 */
	virtual void receivedObject(LDData* data)=0;

	/**
	 * Will be called if there has been a new LDData object regardless of its content.
	 * Should not be used for heavy calculations.
	 */
	virtual void receivedAnyThing()=0;
};


#endif /* CALLBACK_H_ */
