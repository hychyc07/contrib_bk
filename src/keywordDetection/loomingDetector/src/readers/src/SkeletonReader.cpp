/*
 * SkeletonReader.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/SkeletonReader.h"

SkeletonReader::SkeletonReader(BufferedPort<Bottle> *in, DataCollector* parent,
		bool emulator) {
	this->in = in;
	this->parent = parent;
	this->emulator = emulator;
}

bool SkeletonReader::threadInit() {
	stoploop = false;
	return true;
}

void SkeletonReader::run() {
	cout << " SkeletonReader" << endl;
	while (!stoploop) {
		//Checking if the bottle is good
		bot = in->read();
		if (bot != NULL) {
			if (bot->size() > 0) {
				SkeletonData* sd = new SkeletonData();
				//Creating a new ObjectData object and storing it in the DataCollector
				//Parses the Bottle created by the emulator
				if (emulator) {
					sd->setTimestamp(bot->get(0).asDouble());
					for (int i = 1; i < bot->size(); i++) {
						if (bot->get(i).asString() == "Head") {
							sd->setHeadX(bot->get(i + 2).asDouble());
							sd->setHeadY(bot->get(i + 3).asDouble());
							sd->setHeadZ(bot->get(i + 4).asDouble());
							if (i + 5 < bot->size()) {
								if (bot->get(i + 5).asString()
										== "Orientation") {
									double tmp[ARRAY_SIZE] = {
											bot->get(i + 7).asDouble(),
											bot->get(i + 8).asDouble(),
											bot->get(i + 9).asDouble(),
											bot->get(i + 10).asDouble(),
											bot->get(i + 11).asDouble(),
											bot->get(i + 12).asDouble(),
											bot->get(i + 13).asDouble(),
											bot->get(i + 14).asDouble(),
											bot->get(i + 15).asDouble() };
									vector<double> orientation;
									orientation.insert(orientation.begin(), tmp,
											tmp + ARRAY_SIZE);
									sd->setHeadOrientation(orientation);
								}
							}
						} else if (bot->get(i).asString() == "Left_Hand") {
							sd->setLeftHandX(bot->get(i + 2).asDouble());
							sd->setLeftHandY(bot->get(i + 3).asDouble());
							sd->setLeftHandZ(bot->get(i + 4).asDouble());
							if (i + 5 < bot->size()) {
								if (bot->get(i + 5).asString()
										== "Orientation") {
									double tmp[ARRAY_SIZE] = {
											bot->get(i + 7).asDouble(),
											bot->get(i + 8).asDouble(),
											bot->get(i + 9).asDouble(),
											bot->get(i + 10).asDouble(),
											bot->get(i + 11).asDouble(),
											bot->get(i + 12).asDouble(),
											bot->get(i + 13).asDouble(),
											bot->get(i + 14).asDouble(),
											bot->get(i + 15).asDouble() };
									vector<double> orientation;
									orientation.insert(orientation.begin(), tmp,
											tmp + ARRAY_SIZE);
									sd->setLeftHandOrientation(orientation);
								}
							}
						} else if (bot->get(i).asString() == "Right_Hand") {
							sd->setRightHandX(bot->get(i + 2).asDouble());
							sd->setRightHandY(bot->get(i + 3).asDouble());
							sd->setRightHandZ(bot->get(i + 4).asDouble());
							if (i + 5 < bot->size()) {
								if (bot->get(i + 5).asString()
										== "Orientation") {
									double tmp[ARRAY_SIZE] = {
											bot->get(i + 7).asDouble(),
											bot->get(i + 8).asDouble(),
											bot->get(i + 9).asDouble(),
											bot->get(i + 10).asDouble(),
											bot->get(i + 11).asDouble(),
											bot->get(i + 12).asDouble(),
											bot->get(i + 13).asDouble(),
											bot->get(i + 14).asDouble(),
											bot->get(i + 15).asDouble() };
									vector<double> orientation;
									orientation.insert(orientation.begin(), tmp,
											tmp + ARRAY_SIZE);
									sd->setRightHandOrientation(orientation);
								}
							}
						} else if (bot->get(i).asString() == "Chest") {
							sd->setChestX(bot->get(i + 2).asDouble());
							sd->setChestY(bot->get(i + 3).asDouble());
							sd->setChestZ(bot->get(i + 4).asDouble());
							if (i + 5 < bot->size()) {
								if (bot->get(i + 5).asString()
										== "Orientation") {
									double tmp[ARRAY_SIZE] = {
											bot->get(i + 7).asDouble(),
											bot->get(i + 8).asDouble(),
											bot->get(i + 9).asDouble(),
											bot->get(i + 10).asDouble(),
											bot->get(i + 11).asDouble(),
											bot->get(i + 12).asDouble(),
											bot->get(i + 13).asDouble(),
											bot->get(i + 14).asDouble(),
											bot->get(i + 15).asDouble() };
									vector<double> orientation;
									orientation.insert(orientation.begin(), tmp,
											tmp + ARRAY_SIZE);
									sd->setChestOrientation(orientation);
								}
							}
						}
					}
				} else {
					//Creating a new ObjectData object and storing it in the DataCollector
					//Parses the Bottle created by the KinectDeviceLocal of yarp
					in->getEnvelope(time);
					sd->setTimestamp(time.getTime());
					Bottle pos;
					int j = 0;
					for (int i = 0; i < bot->size(); i++) {
						switch (bot->get(i).asInt()) {
						case VOCAB3('P','O','S'):
							switch (j) {
							case HEAD_POS:
								sd->setHeadX(
										bot->get(i + 1).asList()->get(0).asDouble());
								sd->setHeadY(
										bot->get(i + 1).asList()->get(1).asDouble());
								sd->setHeadZ(
										bot->get(i + 1).asList()->get(2).asDouble());
								break;
							case CHEST_POS:
								sd->setChestX(
										bot->get(i + 1).asList()->get(0).asDouble());
								sd->setChestY(
										bot->get(i + 1).asList()->get(1).asDouble());
								sd->setChestZ(
										bot->get(i + 1).asList()->get(2).asDouble());
								break;
							case LEFT_HAND_POS:
								sd->setLeftHandX(
										bot->get(i + 1).asList()->get(0).asDouble());
								sd->setLeftHandY(
										bot->get(i + 1).asList()->get(1).asDouble());
								sd->setLeftHandZ(
										bot->get(i + 1).asList()->get(2).asDouble());
								break;
							case RIGHT_HAND_POS:
								sd->setRightHandX(
										bot->get(i + 1).asList()->get(0).asDouble());
								sd->setRightHandY(
										bot->get(i + 1).asList()->get(1).asDouble());
								sd->setRightHandZ(
										bot->get(i + 1).asList()->get(2).asDouble());
								break;
							}
							j++;
							break;
						}
					}
				}
				parent->addSkeletonData(sd);
			}
		}
	}
}

void SkeletonReader::threadRelease() {
	cout << "  SkeletonReader stopped" << endl;
}

