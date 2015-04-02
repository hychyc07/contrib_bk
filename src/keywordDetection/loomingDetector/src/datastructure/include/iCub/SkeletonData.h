/*
 * SkeletonData.h
 *
 *  Created on: Jun 5, 2012
 *      Author: cdondrup
 */

#ifndef SKELETONDATA_H_
#define SKELETONDATA_H_

#include <string>
#include <vector>

#include "iCub/StringConverter.h"

using namespace std;

#define ARRAY_SIZE 9
#define Z_NORM_FACTOR 1000.0

/**
 * This class is designed to contain one skeleton data object from the yarp KinectDeviceLocal.
 */
class SkeletonData {
private:
	double timestamp;
	double headX;
	double headY;
	double headZ;
	vector<double> headOrientation;
	double leftHandX;
	double leftHandY;
	double leftHandZ;
	vector<double> leftHandOrientation;
	double rightHandX;
	double rightHandY;
	double rightHandZ;
	vector<double> rightHandOrientation;
	double chestX;
	double chestY;
	double chestZ;
	vector<double> chestOrientation;

public:

	/**
	 * Creates a new SkeletonData object where all parameters are set to 0.
	 */
	SkeletonData()
		:timestamp(0.0),
		 headX(0.0),
		 headY(0.0),
		 headZ(0.0),
		 leftHandX(0.0),
		 leftHandY(0.0),
		 leftHandZ(0.0),
		 rightHandX(0.0),
		 rightHandY(0.0),
		 rightHandZ(0.0),
		 chestX(0.0),
		 chestY(0.0),
		 chestZ(0.0)
	{
		headOrientation = vector<double>(ARRAY_SIZE,0.0);
		leftHandOrientation = vector<double>(ARRAY_SIZE,0.0);
		rightHandOrientation = vector<double>(ARRAY_SIZE,0.0);
		chestOrientation = vector<double>(ARRAY_SIZE,0.0);
	}

	/**
	 * Creates a new SkeletonData object with the given parameters and
	 * everything else set to 0.
	 * @param timestamp The timestamp when the SkeletonData was recorded
	 * @param headX The x coordinate of the head
	 * @param headY The y coordinate of the head
	 * @param headZ The z coordinate of the head
	 */
	SkeletonData(double timestamp,
			double headX,
			double headY,
			double headZ)
		:timestamp(timestamp),
		 headX(headX),
		 headY(headY),
		 headZ(headZ),
		 leftHandX(0.0),
		 leftHandY(0.0),
		 leftHandZ(0.0),
		 rightHandX(0.0),
		 rightHandY(0.0),
		 rightHandZ(0.0),
		 chestX(0.0),
		 chestY(0.0),
		 chestZ(0.0)
	{
		headOrientation = vector<double>(ARRAY_SIZE,0.0);
		leftHandOrientation = vector<double>(ARRAY_SIZE,0.0);
		rightHandOrientation = vector<double>(ARRAY_SIZE,0.0);
		chestOrientation = vector<double>(ARRAY_SIZE,0.0);
	}

	/**
	 * Creates a new SkeletonData object with the given parameters and
	 * everything else set to 0.
	 * @param timestamp The timestamp when the SkeletonData was recorded
	 * @param headX The x coordinate of the head
	 * @param headY The y coordinate of the head
	 * @param headZ The z coordinate of the head
	 * @param headOrientation A 3x3 rotation matrix for the head represented by a vector of the rows
	 */
	SkeletonData(double timestamp,
			double headX,
			double headY,
			double headZ,
			vector<double> headOrientation)
		:timestamp(timestamp),
		 headX(headX),
		 headY(headY),
		 headZ(headZ),
		 headOrientation(headOrientation),
		 leftHandX(0.0),
		 leftHandY(0.0),
		 leftHandZ(0.0),
		 rightHandX(0.0),
		 rightHandY(0.0),
		 rightHandZ(0.0),
		 chestX(0.0),
		 chestY(0.0),
		 chestZ(0.0)
	{
		leftHandOrientation = vector<double>(ARRAY_SIZE,0.0);
		rightHandOrientation = vector<double>(ARRAY_SIZE,0.0);
		chestOrientation = vector<double>(ARRAY_SIZE,0.0);
	}

	/**
	 * Creates a new SkeletonData object with the given parameters and
	 * everything else set to 0.
	 * @param timestamp The timestamp when the SkeletonData was recorded
	 * @param headX The x coordinate of the head
	 * @param headY The y coordinate of the head
	 * @param headZ The z coordinate of the head
	 * @param headOrientation A 3x3 rotation matrix for the head represented by a vector of the rows
	 * @param leftHandX The x coordinate of the left hand
	 * @param leftHandY The y coordinate of the left hand
	 * @param leftHandZ The z coordinate of the left hand
	 * @param rightHandX The x coordinate of the right hand
	 * @param rightHandY The y coordinate of the right hand
	 * @param rightHandZ The z coordinate of the right hand
	 */
	SkeletonData(double timestamp,
			double headX,
			double headY,
			double headZ,
			vector<double> headOrientation,
			double leftHandX,
			double leftHandY,
			double leftHandZ,
			double rightHandX,
			double rightHandY,
			double rightHandZ)
		:timestamp(timestamp),
		 headX(headX),
		 headY(headY),
		 headZ(headZ),
		 headOrientation(headOrientation),
		 leftHandX(leftHandX),
		 leftHandY(leftHandY),
		 leftHandZ(leftHandZ),
		 rightHandX(rightHandX),
		 rightHandY(rightHandY),
		 rightHandZ(rightHandZ),
		 chestX(0.0),
		 chestY(0.0),
		 chestZ(0.0)
	{
		leftHandOrientation = vector<double>(ARRAY_SIZE,0.0);
		rightHandOrientation = vector<double>(ARRAY_SIZE,0.0);
		chestOrientation = vector<double>(ARRAY_SIZE,0.0);
	}

	/**
	 * Creates a new SkeletonData object with the given parameters and
	 * everything else set to 0.
	 * @param timestamp The timestamp when the SkeletonData was recorded
	 * @param headX The x coordinate of the head
	 * @param headY The y coordinate of the head
	 * @param headZ The z coordinate of the head
	 * @param headOrientation A 3x3 rotation matrix for the head represented by a vector of the rows
	 * @param leftHandX The x coordinate of the left hand
	 * @param leftHandY The y coordinate of the left hand
	 * @param leftHandZ The z coordinate of the left hand
	 * @param leftHandOrientation A 3x3 rotation matrix for the head represented by a vector of the rows
	 * @param rightHandX The x coordinate of the right hand
	 * @param rightHandY The y coordinate of the right hand
	 * @param rightHandZ The z coordinate of the right hand
	 * @param rightHandOrientation A 3x3 rotation matrix for the head represented by a vector of the rows
	 */
	SkeletonData(double timestamp,
			double headX,
			double headY,
			double headZ,
			vector<double> headOrientation,
			double leftHandX,
			double leftHandY,
			double leftHandZ,
			vector<double> leftHandOrientation,
			double rightHandX,
			double rightHandY,
			double rightHandZ,
			vector<double> rightHandOrientation)
		:timestamp(timestamp),
		 headX(headX),
		 headY(headY),
		 headZ(headZ),
		 headOrientation(headOrientation),
		 leftHandX(leftHandX),
		 leftHandY(leftHandY),
		 leftHandZ(leftHandZ),
		 leftHandOrientation(leftHandOrientation),
		 rightHandX(rightHandX),
		 rightHandY(rightHandY),
		 rightHandZ(rightHandZ),
		 rightHandOrientation(rightHandOrientation),
		 chestX(0.0),
		 chestY(0.0),
		 chestZ(0.0)
	{
		chestOrientation = vector<double>(ARRAY_SIZE,0.0);
	}

	/**
	 * Creates a new SkeletonData object with the given parameters and
	 * everything else set to 0.
	 * @param timestamp The timestamp when the SkeletonData was recorded
	 * @param headX The x coordinate of the head
	 * @param headY The y coordinate of the head
	 * @param headZ The z coordinate of the head
	 * @param headOrientation A 3x3 rotation matrix for the head represented by a vector of its rows
	 * @param leftHandX The x coordinate of the left hand
	 * @param leftHandY The y coordinate of the left hand
	 * @param leftHandZ The z coordinate of the left hand
	 * @param leftHandOrientation A 3x3 rotation matrix for the left hand represented by a vector of its rows
	 * @param rightHandX The x coordinate of the right hand
	 * @param rightHandY The y coordinate of the right hand
	 * @param rightHandZ The z coordinate of the right hand
	 * @param rightHandOrientation A 3x3 rotation matrix for the right hand represented by a vector of its rows
	 * @param chestX The x coordinate of the chest
	 * @param chestY The y coordinate of the chest
	 * @param chestZ The z coordinate of the chest
	 * @param chestOrientation A 3x3 rotation matrix for the chest represented by a vector of its rows
	 */
	SkeletonData(double timestamp,
			double headX,
			double headY,
			double headZ,
			vector<double> headOrientation,
			double leftHandX,
			double leftHandY,
			double leftHandZ,
			vector<double> leftHandOrientation,
			double rightHandX,
			double rightHandY,
			double rightHandZ,
			vector<double> rightHandOrientation,
			double chestX,
			double chestY,
			double chestZ,
			vector<double> chestOrientation)
	:timestamp(timestamp),
	 headX(headX),
	 headY(headY),
	 headZ(headZ),
	 headOrientation(headOrientation),
	 leftHandX(leftHandX),
	 leftHandY(leftHandY),
	 leftHandZ(leftHandZ),
	 leftHandOrientation(leftHandOrientation),
	 rightHandX(rightHandX),
	 rightHandY(rightHandY),
	 rightHandZ(rightHandZ),
	 rightHandOrientation(rightHandOrientation),
	 chestX(chestX),
	 chestY(chestY),
	 chestZ(chestZ),
	 chestOrientation(chestOrientation){}

	/**
	 * The Orientation vector is in fact a 3x3 rotation matrix. The vector consists of the
	 * 3 rows of this matrix.
	 * @return A 3x3 rotation matrix for the chest represented by a vector of its rows
	 */
	const vector<double> getChestOrientation() const
	{
		return chestOrientation;
	}

	/**
	 * The Orientation vector is in fact a 3x3 rotation matrix. The vector consists of the
	 * 3 rows of this matrix.
	 * @param chestOrientation A 3x3 rotation matrix for the chest represented by a vector of its rows
	 */
	void setChestOrientation(vector<double> chestOrientation) {
		this->chestOrientation = chestOrientation;
	}

	/**
	 * @return The x coordinate of the chest
	 */
	double getChestX() const
	{
		return chestX;
	}

	/**
	 * @param chestX The x coordinate of the chest
	 */
	void setChestX(double chestX = 0.0)
	{
		this->chestX = chestX;
	}

	/**
	 * @return The y coordinate of the chest
	 */
	double getChestY() const
	{
		return chestY;
	}

	/**
	 * @param chestY The y coordinate of the chest
	 */
	void setChestY(double chestY = 0.0)
	{
		this->chestY = chestY;
	}

	/**
	 * @return The z coordinate of the chest
	 */
	double getChestZ() const
	{
		return chestZ;
	}

	/**
	 * @param chestZ The z coordinate of the chest
	 */
	void setChestZ(double chestZ = 0.0)
	{
		this->chestZ = chestZ;
	}

	/**
	 * The Orientation vector is in fact a 3x3 rotation matrix. The vector consists of the
	 * 3 rows of this matrix.
	 * @return A 3x3 rotation matrix for the chest represented by a vector of its rows
	 */
	const vector<double> getHeadOrientation() const
	{
		return headOrientation;
	}

	/**
	 * The Orientation vector is in fact a 3x3 rotation matrix. The vector consists of the
	 * 3 rows of this matrix.
	 * @param headOrientation A 3x3 rotation matrix for the head represented by a vector of its rows
	 */
	void setHeadOrientation(vector<double> headOrientation) {
			this->headOrientation = headOrientation;
		}

	/**
	 * @return The x coordinate of the head
	 */
	double getHeadX() const
	{
		return headX;
	}

	/**
	 * @param headX The x coordinate of the head
	 */
	void setHeadX(double headX = 0.0)
	{
		this->headX = headX;
	}

	/**
	 * @return The y coordinate of the head
	 */
	double getHeadY() const
	{
		return headY;
	}

	/**
	 * @param headY The y coordinate of the head
	 */
	void setHeadY(double headY = 0.0)
	{
		this->headY = headY;
	}

	/**
	 * @return The z coordinate of the head
	 */
	double getHeadZ() const
	{
		return headZ;
	}

	/**
	 * @param headZ The z coordinate of the head
	 */
	void setHeadZ(double headZ = 0.0)
	{
		this->headZ = headZ;
	}

	/**
	 * The Orientation vector is in fact a 3x3 rotation matrix. The vector consists of the
	 * 3 rows of this matrix.
	 * @return A 3x3 rotation matrix for the left hand represented by a vector of its rows
	 */
	const vector<double> getLeftHandOrientation() const
	{
		return leftHandOrientation;
	}

	/**
	 * The Orientation vector is in fact a 3x3 rotation matrix. The vector consists of the
	 * 3 rows of this matrix.
	 * @param leftHandOrientation A 3x3 rotation matrix for the left hand represented by a vector of its rows
	 */
	void setLeftHandOrientation(vector<double> leftHandOrientation) {
			this->leftHandOrientation = leftHandOrientation;
		}

	/**
	 * @return The x coordinate of the left hand
	 */
	double getLeftHandX() const
	{
		return leftHandX;
	}

	/**
	 * @param leftHandX The x coordinate of the left hand
	 */
	void setLeftHandX(double leftHandX = 0.0)
	{
		this->leftHandX = leftHandX;
	}

	/**
	 * @return The y coordinate of the left hand
	 */
	double getLeftHandY() const
	{
		return leftHandY;
	}

	/**
	 * @param leftHandY The y coordinate of the left hand
	 */
	void setLeftHandY(double leftHandY = 0.0)
	{
		this->leftHandY = leftHandY;
	}

	/**
	 * @return The z coordinate of the left hand
	 */
	double getLeftHandZ() const
	{
		return leftHandZ;
	}

	/**
	 * This is normed to meters in world coordinates.
	 * @return The normed z coordinate of the left hand
	 */
	double getLeftHandZNormed() const
	{
		return leftHandZ/Z_NORM_FACTOR;
	}

	/**
	 * @param leftHandZ The z coordinate of the left hand
	 */
	void setLeftHandZ(double leftHandZ = 0.0)
	{
		this->leftHandZ = leftHandZ;
	}

	/**
	 * The Orientation vector is in fact a 3x3 rotation matrix. The vector consists of the
	 * 3 rows of this matrix.
	 * @return A 3x3 rotation matrix for the right hand represented by a vector of its rows
	 */
	const vector<double> getRightHandOrientation() const
	{
		return rightHandOrientation;
	}

	/**
	 * The Orientation vector is in fact a 3x3 rotation matrix. The vector consists of the
	 * 3 rows of this matrix.
	 * @param rightHandOrientation A 3x3 rotation matrix for the right hand represented by a vector of its rows
	 */
	void setRightHandOrientation(vector<double> rightHandOrientation) {
			this->rightHandOrientation = rightHandOrientation;
		}

	/**
	 * @return The x coordinate of the right hand
	 */
	double getRightHandX() const
	{
		return rightHandX;
	}

	/**
	 * @param rightHandX The x coordinate of the right hand
	 */
	void setRightHandX(double rightHandX = 0.0)
	{
		this->rightHandX = rightHandX;
	}

	/**
	 * @return The y coordinate of the right hand
	 */
	double getRightHandY() const
	{
		return rightHandY;
	}

	/**
	 * @param rightHandY The y coordinate of the right hand
	 */
	void setRightHandY(double rightHandY = 0.0)
	{
		this->rightHandY = rightHandY;
	}

	/**
	 * @return The z coordinate of the right hand
	 */
	double getRightHandZ() const
	{
		return rightHandZ;
	}

	/**
	 * This is normed to meters in world coordinates.
	 * @return The normed z coordinate of the right hand
	 */
	double getRightHandZNormed() const
	{
		return rightHandZ/Z_NORM_FACTOR;
	}

	/**
	 * @param rightHandZ The z coordinate of the right hand
	 */
	void setRightHandZ(double rightHandZ = 0.0)
	{
		this->rightHandZ = rightHandZ;
	}

	/**
	 * @return The timestamp when this SkeletonData was recorded
	 */
	double getTimestamp() const
	{
		return timestamp;
	}

	/**
	 * @param timestamp Set a new timestamp
	 */
	void setTimestamp(double timestamp = 0.0)
	{
		this->timestamp = timestamp;
	}

	/**
	 * @return true if the coordinates of the left hand are not 0
	 */
	bool isLeftHand() {
		return getLeftHandX() != 0.0
				&& getLeftHandY() != 0.0
				&& getLeftHandZ() != 0.0;
	}

	/**
	 * @return true if the coordinates of the right hand are not 0
	 */
	bool isRightHand() {
	        return getRightHandX() != 0.0
	                && getRightHandY() != 0.0
	                && getRightHandZ() != 0.0;
	}

	/**
	 * Creates a string object which contains every information stored in this object
	 * in a human readable form.
	 * @return A formatted string containing all informations of this object
	 */
	string toString() {
		string result = "Skeleton: ";
		result += to_string<double>(getTimestamp()) + " ";

		result += "Head: ";
		result += to_string<double>(getHeadX()) + " " + to_string<double>(getHeadY()) + " "+ to_string<double>(getHeadZ());
		result += " Orientation: ";
		for (int i = 0; i < getHeadOrientation().size(); i++) {
			result += to_string<double>(getHeadOrientation()[i]) + " ";
		}

		result += "Left Hand: ";
		result += to_string<double>(getLeftHandX()) + " " + to_string<double>(getLeftHandY()) + " " + to_string<double>(getLeftHandZ());
		result += " Orientation: ";
		for (int i = 0; i < getLeftHandOrientation().size(); i++) {
			result += to_string<double>(getLeftHandOrientation()[i]) + " ";
		}

		result += "Right Hand: ";
		result += to_string<double>(getRightHandX()) + " " + to_string<double>(getRightHandY()) + " " + to_string<double>(getRightHandZ());
		result += " Orientation: ";
		for (int i = 0; i < getRightHandOrientation().size(); i++) {
			result += to_string<double>(getRightHandOrientation()[i]) + " ";
		}

		result += "Chest: ";
		result += to_string<double>(getChestX()) + " " + to_string<double>(getChestY()) + " " + to_string<double>(getChestZ());
		result += " Orientation: ";
		for (int i = 0; i < getChestOrientation().size(); i++) {
			result += to_string<double>(getChestOrientation()[i]) + " ";
		}

		return result;
	}
};


#endif /* SKELETONDATA_H_ */
