/*
 * LoomingWriter.h
 *
 *  Created on: Jun 11, 2012
 *      Author: Christian Dondrup
 */

#ifndef LOOMINGWRITER_H_
#define LOOMINGWRITER_H_

#include <string>

#include <yarp/os/all.h>

#include "iCub/ListCreator.h"

using namespace std;
using namespace yarp::os;

#define START VOCAB4('s','t','a','r')
#define STOP VOCAB4('s','t','o','p')
#define RIGHT VOCAB4('r','i','g','h')
#define LEFT VOCAB4('l','e','f','t')
#define RESET VOCAB3('r','e','s')
#define VARIANCE VOCAB3('v','a','r')
#define MEAN VOCAB4('m','e','a','n')
#define DISTANCE VOCAB4('d','i','s','t')

/**
 * This class writes to the given output port.
 */
class LoomingWriter {
private:

	Port *out;
	BufferedPort<Bottle> *data;
	Bottle* bot;

public:

	/**
	 * Creates a new LoomingWriter object with the given port to write on.
	 * @param out The port to write the looming results to
	 * @param data The port to write some calculation data to
	 */
	LoomingWriter(Port *out,BufferedPort<Bottle> *data) :
			out(out), data(data) {
	}

	/**
	 * Writes the given string to the data output port given on creation.
	 * The string can contain any data and will be split accordingly by yarp::os::Bottle::fromString().
	 * @param data The string containing the data
	 */
	void write(string data);

	/**
	 * Creates a Bottle to indicate when the looming started and what hand looms.
	 * The Bottle consists of two yarp::os::Vocab and one double and is send to the output port.
	 * The output looks like:
	 * [star] [left/righ] (secondes.milliseconds x.x y.y z.z)
	 * @param data The Bottle containing the time stamp and the coordinates of the looming hand
	 * @param right true if the right hand is looming
	 */
	void setLoomingStart(Bottle data, bool right);

	/**
	 * Creates a Bottle with the current time stamp and the coordinates of the looming hand.
	 * The Bottle will look like:
	 * [left/right] (secondes.milliseconds x.x y.y z.z)
	 * @param data The Bottle containing the time stamp and the coordinates of the looming hand
	 * @param right true if the right hand is looming
	 */
	void sendLooming(Bottle data, bool right);

	/**
	 * Creates a Bottle to indicate when the looming stopped and what hand did loom before.
	 * The Bottle consists of two yarp::os::Vocab and one double and is send to the output port.
	 * The output looks like:
	 * [stop] [left/righ] (secondes.milliseconds x.x y.y z.z)
	 * @param data The Bottle containing the time stamp and the coordinates of the looming hand
	 * @param right true if the right hand was looming
	 */
	void setLoomingStop(Bottle data, bool right);

	/**
	 * Creates a Bottle containing the calculated mean, variance and current distance to that mean
	 * of one of the hands and writes it to the data output port. The Bottle contains several
	 * yarp::os::Vocab variables and the according double values. The output looks like:
	 * (secondes.milliseconds x.x y.y z.z) [righ/left] [mean] mean.value [var] variance.value [dist] current.distance
	 * @param data The Bottle containing the time stamp and the coordinates of the looming hand
	 * @param mean The mean value
	 * @param var The variance from the given mean
	 * @param dist The current distance of the hand from the given mean
	 * @param right true if values are concerning the right hand
	 */
	void writeData(Bottle data, double mean, double var, double dist, bool right);

	/**
	 * Creates a Bottle to signal a value reset. Cotains two yarp::os::Vocab variables.
	 * This Bottle is sent to the data output port and looks like this:
	 * (secondes.milliseconds x.x y.y z.z) [righ/left] [res]
	 * @param data The Bottle containing the time stamp and the coordinates of the looming hand
	 * @param right true if right hand is reseted
	 */
	void sendResetFlag(Bottle data, bool right);
};

#endif /* LOOMINGWRITER_H_ */
