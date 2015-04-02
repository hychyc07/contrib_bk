/*
 * LoomingWriter.cpp
 *
 *  Created on: Jun 11, 2012
 *      Author: Christian Dondrup
 */

#include "iCub/LoomingWriter.h"

using namespace std;
using namespace yarp::os;

void LoomingWriter::write(string data) {
	bot = &this->data->prepare();
	bot->clear();
	bot->fromString(data.c_str());
	this->data->writeStrict();
}

void LoomingWriter::setLoomingStart(Bottle data, bool right) {
	Bottle bot;
	bot.addVocab(START);
	bot.addVocab(right ? RIGHT : LEFT);
	addAsList(&bot, &data);
	out->write(bot);
}

void LoomingWriter::setLoomingStop(Bottle data, bool right) {
	Bottle bot;
	bot.addVocab(STOP);
	bot.addVocab(right ? RIGHT : LEFT);
	addAsList(&bot, &data);
	out->write(bot);
}

void LoomingWriter::sendResetFlag(Bottle data, bool right) {
	bot = &this->data->prepare();
	bot->clear();
	addAsList(bot, &data);
	bot->addVocab(right ? RIGHT : LEFT);
	bot->addVocab(RESET);
	this->data->writeStrict();
}

void LoomingWriter::writeData(Bottle data, double mean, double var, double dist,
		bool right) {
	bot = &this->data->prepare();
	bot->clear();
	addAsList(bot, &data);
	bot->addVocab(right ? RIGHT : LEFT);
	bot->addVocab(MEAN);
	bot->addDouble(mean);
	bot->addVocab(VARIANCE);
	bot->addDouble(var);
	bot->addVocab(DISTANCE);
	bot->addDouble(dist);
	this->data->writeStrict();
}

void LoomingWriter::sendLooming(Bottle data, bool right) {
	Bottle bot;
	bot.addVocab(right ? RIGHT : LEFT);
	addAsList(&bot, &data);
	out->write(bot);
}
