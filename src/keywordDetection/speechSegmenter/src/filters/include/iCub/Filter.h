/*
 * Filter.h
 *
 *  Created on: Jul 11, 2012
 *      Author: cdondrup
 */

#ifndef FILTER_H_
#define FILTER_H_

#include <vector>

#include <fftw3.h>

using namespace std;

/**
 * @brief Provides functionality for filtering.
 *
 * This class does not create a filter on its own but takes a constructed filter
 * and uses (multiplication) it on the data.
 * Has to be used on a DFTed window of the wave file.
 */
class Filter {
private:
	float* filter;
	int size;

public:
	/**
	 * Sets the filter use.
	 * @param filter A float array of filter arguments
	 */
	void setFilter(float* filter) {
		this->filter = filter;
	}

	/**
	 * @return The current filter array
	 */
	float* getFilter() const {
		return filter;
	}

	/**
	 * @param size The size of the window to filter and the filter array
	 */
	void setSize(int size) {
		this->size = size;
	}

	/**
	 * Multiplies the given complex DFTed window with the filter array.
	 * @param in The complex window to filter.
	 */
	fftw_complex* filterWindow(fftw_complex* in);

	/**
	 * Takes the whole signal (all windows in a vector) and calls filterWindow
	 * for each of the windows.
	 * @param in The signal to filter represented as a vector of DFTed windows
	 */
	vector<fftw_complex*> filterSignal(vector<fftw_complex*> in);
};

#endif /* FILTER_H_ */
