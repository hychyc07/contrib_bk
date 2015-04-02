/*
 * StringConverter.h
 *
 *  Created on: Jun 5, 2012
 *      Author: Christian Dondrup
 */

#ifndef STRINGCONVERTER_H_
#define STRINGCONVERTER_H_

#include <sstream>
#include <string>

/**
 * Converts any number to a string object.
 * Specify the origin class like to_string<double>(value)
 * @param t The number that should be a string
 */
template<class T>
inline std::string to_string(const T& t) {
	std::stringstream ss;
	ss << std::fixed << t;
	return ss.str();
}

#endif /* STRINGCONVERTER_H_ */
