/*
 * Hand.cpp
 *
 *  Created on: Jun 13, 2012
 *      Author: Christian Dondrup
 */

/* Not used at the moment */

#include "iCub/Hand.h"

using namespace std;

void Hand::calculateLooming(double dist_hand) {
	if(checkVarianceCounter()) {
		var[var_n++] = dist_hand;
	} else {
		looming = pow(dist_hand - mean_dist,2) > getVariance();
	}

}

double Hand::getNewMean(double dist_hand) {
	mean_dist = ((mean_dist * n) + dist_hand) / ++n;
	return mean_dist;
}

double Hand::getVariance() {
	double result = 0.0;
	for(int i = 0; i < VARIANCE_RESET_THRESHOLD; i++) {
		result += pow(var[i]-mean_dist,2);
	}
	return result / VARIANCE_RESET_THRESHOLD;
}

bool Hand::checkVarianceCounter() {
	return var_n < VARIANCE_RESET_THRESHOLD;
}

void Hand::resetVariance() {
	var_n = 0;
}

