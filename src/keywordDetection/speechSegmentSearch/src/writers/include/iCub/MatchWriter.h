/*
 * MatchWriter.h
 *
 *  Created on: Jun 11, 2012
 *      Author: Christian Dondrup
 */

#ifndef MATCHWRITER_H_
#define MATCHWRITER_H_

#include <string>
#include <fstream>
#include <util.h>
#include <iostream>

#include <yarp/os/all.h>

#include "iCub/ListCreator.h"
#include "iCub/StringConverter.h"

using namespace std;
using namespace yarp::os;

/**
 * This class writes to the given output port.
 */
class MatchWriter {
private:

	Port* out;
	Bottle bot;

public:

	/**
	 * Creates a new MatchWriter object with the given port to write on.
	 * @param out The port to write the looming results to
	 */
	MatchWriter(Port *out) :
			out(out){
	}

	/**
	 * Adds the given Bottle to the data collection. Data will be sent if send() is called.
	 * prepare() has to be called before adding data. The given Bottle will be added as a list
	 * to the existing Bottle.
	 * @param data The Bottle containing the data
	 */
	void setData(Bottle data, string filename);

	/**
	 * Getting the Bottle from the port and clearing it. HAs to be called first.
	 * Using BufferedPort::prepare() and Bottle::clear().
	 */
	void prepare();

	/**
	 * Sending the data which was added by setData().
	 * Using BufferedPort::writeStrict().
	 */
	void send();

	/**
	 * Sending an error Bottle with the Vocab [ERR] and a custom message string.
	 * @param msg The error message
	 */
	void error(string msg, string filename);

	bool writeToFile(string filename, Bottle data, string orig);
};

#endif /* MATCHWRITER_H_ */
