/*
 * PathFinder.h
 *
 *  Created on: Nov 2, 2012
 *      Author: cdondrup
 */

#ifndef PATHFINDER_H_
#define PATHFINDER_H_

#include <limits>
#include <iostream>

class PathFinder {
private:

public:
	static double findPath(double** simmat, int m, int n) {

		double** costMat = new double*[m + 1];
		int** traceback = new int*[m];

		for (int i = 0; i < m + 1; i++) {
			costMat[i] = new double[n + 1];
			for (int j = 0; j < n + 1; j++) {
				if (i == 0 && j == 0) {
					costMat[i][j] = 0.0;
				} else if (i == 0 || j == 0) {
					costMat[i][j] = numeric_limits<double>::max();
				} else {
					costMat[i][j] = simmat[i - 1][j - 1];
				}
			}
		}

		for (int i = 0; i < m; i++) {
			traceback[i] = new int[n];
			for (int j = 0; j < n; j++) {
				if (costMat[i][j] <= costMat[i][j + 1]
						&& costMat[i][j] <= costMat[i + 1][j]) {
					costMat[i + 1][j + 1] += costMat[i][j];
					traceback[i][j] = 1;
				} else if (costMat[i][j + 1] <= costMat[i][j]
						&& costMat[i][j + 1] <= costMat[i + 1][j]) {
					costMat[i + 1][j + 1] += costMat[i][j + 1];
					traceback[i][j] = 2;
				} else if (costMat[i + 1][j] <= costMat[i][j]
						&& costMat[i + 1][j] <= costMat[i][j + 1]) {
					costMat[i + 1][j + 1] += costMat[i + 1][j];
					traceback[i][j] = 3;
				}
			}
		}

		return costMat[m][n] / (m + n);
	}
};

#endif /* PATHFINDER_H_ */
