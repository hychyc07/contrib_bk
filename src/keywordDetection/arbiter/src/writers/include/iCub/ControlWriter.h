/*
 * ControlWriter.h
 *
 *  Created on: Aug 28, 2012
 *      Author: cdondrup
 */

#ifndef CONTROLWRITER_H_
#define CONTROLWRITER_H_

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class ControlWriter {
private:
	Port *out;
public:
	ControlWriter(Port *out) :
		out(out) {}
	void write(string data) {
		Bottle bot;
		bot.addString(data.c_str());
		out->write(bot);
	}
};


#endif /* CONTROLWRITER_H_ */
