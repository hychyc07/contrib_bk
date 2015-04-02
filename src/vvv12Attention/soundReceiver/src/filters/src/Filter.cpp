/*
 * Filter.cpp
 *
 *  Created on: Jul 11, 2012
 *      Author: cdondrup
 */

#include "iCub/Filter.h"

using namespace std;

fftw_complex* Filter::filterWindow(fftw_complex* in) {
	for(int i = 0; i < size; i++){
		in[i][0] *= filter[i];
		in[i][1] *= filter[i];
	}
	return in;
}

vector<fftw_complex*> Filter::filterSignal(vector<fftw_complex*> in) {
	for(int i = 0; i < in.size(); i++) {
		filterWindow(in[i]);
	}
	return in;
}



