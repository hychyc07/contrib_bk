/*
 * Converter.h
 *
 *  Created on: Jul 23, 2012
 *      Author: cdondrup
 */

#ifndef CONVERTER_H_
#define CONVERTER_H_

#define FLOAT_TRANFORMATION_FACTOR 32768.0

inline float pcmToFloat(int sample) {
	return ((float)sample)/FLOAT_TRANFORMATION_FACTOR;
}

inline int floatToPcm(float sample) {

	return sample*FLOAT_TRANFORMATION_FACTOR;
}



#endif /* CONVERTER_H_ */
