/*
 * ListCreator.h
 *
 *  Created on: Jun 21, 2012
 *      Author: cdondrup
 */

#ifndef LISTCREATOR_H_
#define LISTCREATOR_H_

#include <yarp/os/all.h>

using namespace yarp::os;
using namespace std;

/**
 * Adds a Bottle to the end of another Bottle as a List.
 * @param destination The Bottle which should contain the new List
 * @param origin The Bottle containing the values for the new List
 */
void inline addAsList(Bottle* destination, Bottle* origin) {
	Bottle* tmp = &destination->addList();
	for(int i = 0; i < origin->size(); i++) {
		tmp->add(origin->get(i));
	}
}




#endif /* LISTCREATOR_H_ */
