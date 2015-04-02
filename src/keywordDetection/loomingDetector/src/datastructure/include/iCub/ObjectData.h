/*
 * ObjectData.h
 *
 *  Created on: Jun 5, 2012
 *      Author: Christian Dondrup
 */

#ifndef OBJECTDATA_H_
#define OBJECTDATA_H_

#include <string>

#include "iCub/StringConverter.h"

using namespace std;

/**
 * This class is designed to represent one object data bottle from the ARTrackerUH.
 */
class ObjectData {
private:
	string name;
	string color;
	double x;
	double y;
	double z;
	double nx;
	double ny;
	double size;
	double timestamp;
	int colorid;

public:

	/**
	 * Creates a new ObjectData object with all parameters set to 0 and emtpy strings.
	 */
	ObjectData()
		:timestamp(0.0),
		 name(""),
		 color(""),
		 x(0.0),
		 y(0.0),
		 nx(0.0),
		 ny(0.0),
		 size(0.0),
		 colorid(0){}

	/**
	 * Creates a new ObjectData object with the given parameters.
	 * @param timestamp The timestamp of the object data
	 * @param name The name of the tracked object
	 * @param color The color of the tracker object
	 * @param x The x coordinate
	 * @param y The y coordinate
	 * @param z The z coordinate
	 * @param nx The normalized x coordinate (world coordinates)
	 * @param ny The normelized y coordinate (world coordinates)
	 * @param size The object size
	 * @param colorid The id of the trakced color
	 */
	ObjectData(double timestamp,
			string name,
			string color,
			double x,
			double y,
			double z,
			double nx,
			double ny,
			double size,
			int colorid)
		:timestamp(timestamp),
		 name(name),
		 color(color),
		 x(x),
		 y(y),
		 nx(nx),
		 ny(ny),
		 size(size),
		 colorid(colorid){}

	/**
	 * @return The color id
	 */
	int getColorId() const {
		return colorid;
	}

	/**
	 * @param colorid The new colorid
	 */
	void setColorId(int colorid) {
		this->colorid = colorid;
	}

	/**
	 * @return The name of the object
	 */
	string getName() const {
		return name;
	}

	/**
	 * @param name The new object name
	 */
	void setName(string name) {
		this->name = name;
	}

	/**
	 * @return The normalize x coordinate
	 */
	double getNx() const {
		return nx;
	}

	/**
	 * @param nx The new normalized x coordinate
	 */
	void setNx(double nx) {
		this->nx = nx;
	}

	/**
	 * @return The normalized y coordinate
	 */
	double getNy() const {
		return ny;
	}

	/**
	 * @param ny The new normalized y coordinate
	 */
	void setNy(double ny) {
		this->ny = ny;
	}

	/**
	 * @return The size of the object
	 */
	double getSize() const {
		return size;
	}

	/**
	 * @param size The new size of the object
	 */
	void setSize(double size) {
		this->size = size;
	}

	/**
	 * @return The timestamp of the object detection
	 */
	double getTimestamp() const {
		return timestamp;
	}

	/**
	 * @param timestamp The new timestamp of the object detection
	 */
	void setTimestamp(double timestamp) {
		this->timestamp = timestamp;
	}

	/**
	 * @return The x coordinate of the object
	 */
	double getX() const {
		return x;
	}

	/**
	 * @param x The new x coordinate of the object
	 */
	void setX(double x) {
		this->x = x;
	}

	/**
	 * @return The y coordinate of the object
	 */
	double getY() const {
		return y;
	}

	/**
	 * @param y The new y coordinate of the object
	 */
	void setY(double y) {
		this->y = y;
	}

	/**
	 * @return The z coordinate of the object
	 */
	double getZ() const {
		return z;
	}

	/**
	 * @param z The new z coordinate of the object
	 */
	void setZ(double z) {
		this->z = z;
	}

	/**
	 * @return The color of the object
	 */
	string getColor() const {
		return color;
	}

	/**
	 * @param color The new color of the object
	 */
	void setColor(string color) {
		this->color = color;
	}

	/**
	 * Creates a string object which contains every information stored in this object
	 * in a human readable form.
	 * @return A formatted string containing all informations of this object
	 */
	string toString() {
		string result = "Object: ";
		result += to_string<double>(getTimestamp()) + " ";

		result += getName() + " " + getColor() + " " + to_string<double>(getColorId());
		result += " Pixel Coordinates: ";
		result += to_string<double>(getX()) + " " + to_string<double>(getY()) + " " + to_string<double>(getZ());
		result += " Normalized Coordinates: ";
		result += to_string<double>(getNx()) + " " + to_string<double>(getNy());
		result += " Size: ";
		result += to_string<double>(getSize());

		return result;
	}

};




#endif /* OBJECTDATA_H_ */
