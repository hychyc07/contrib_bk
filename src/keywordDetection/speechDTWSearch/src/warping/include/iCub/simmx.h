/*
 * simmx.h
 *
 *  Created on: Oct 30, 2012
 *      Author: cdondrup
 */

#ifndef SIMMX_H_
#define SIMMX_H_

#include <vector>
#include <math.h>
#include <cmath>

#include <fftw3.h>

#include "iCub/stft.h"

using namespace std;

class simmx {
private:
	static double complexAbs(double real, double imag) {
		return sqrt(pow(real, 2) + pow(imag, 2));
	}

	static double** vectorMultiplication(double* vec1, int vec1size,
			double* vec2, int vec2size) {
		double** result = new double*[vec1size];

		for (int i = 0; i < vec1size; i++) {
			result[i] = new double[vec2size];
			for (int j = 0; j < vec2size; j++) {
				result[i][j] = vec1[i] * vec2[j];
			}
		}

		return result;
	}

	static void matrixElementviseDevision(double** mat1, double** mat2, int m, int n) {
		for(int i = 0; i < m; i++) {
			for(int j = 0; j < n; j++) {
				mat1[i][j] = mat1[i][j] / mat2[i][j];
			}
		}
	}

	static double** matrixMultiplication(const vector<fftw_complex*> spectrum1,
			const vector<fftw_complex*> spectrum2) {
		double** result = new double*[spectrum1.size()];

		cout << "Creating Sim-Matrix: " << spectrum1.size() << "x" << spectrum2.size();

		for (int i = 0; i < spectrum1.size(); i++) {
			result[i] = new double[spectrum2.size()];
			for (int j = 0; j < spectrum2.size(); j++) {
				result[i][j] = 0.0; //Fuck you random number placement in random memory spaces!!!
				for (int k = 0; k < FRAMESIZE / 2 + 1; k++) {
					result[i][j] += (complexAbs(spectrum1.at(i)[k][0],
							spectrum1.at(i)[k][1])
							* complexAbs(spectrum2.at(j)[k][0],
									spectrum2.at(j)[k][1]));
				}
			}
		}
		cout<<" ... done."<<endl;
		return result;
	}

public:
	static double** calculateSimMatrix(const vector<fftw_complex*> spectrum1,
			const vector<fftw_complex*> spectrum2) {
		double** result = matrixMultiplication(spectrum1, spectrum2);
		double sum1[spectrum1.size()];
		double sum2[spectrum2.size()];
		for (int i = 0; i < spectrum1.size(); i++) {
			sum1[i] = 0.0;
			for (int j = 0; j < FRAMESIZE / 2 + 1; j++) {
				sum1[i] += pow(
						complexAbs(spectrum1.at(i)[j][0],
								spectrum1.at(i)[j][1]), 2.0);
			}
			sum1[i] = sqrt(sum1[i]);
		}
		for (int i = 0; i < spectrum2.size(); i++) {
			sum2[i] = 0.0;
			for (int j = 0; j < FRAMESIZE / 2 + 1; j++) {
				sum2[i] += pow(
						complexAbs(spectrum2.at(i)[j][0],
								spectrum2.at(i)[j][1]), 2.0);
			}
			sum2[i] = sqrt(sum2[i]);
		}

		double** sumMat = vectorMultiplication(sum1, spectrum1.size(), sum2, spectrum2.size());
		matrixElementviseDevision(result, sumMat, spectrum1.size(), spectrum2.size());

		for(int i = 0; i < spectrum1.size(); i++) {
			for(int j = 0; j < spectrum2.size(); j++) {
				result[i][j] = 1 - result[i][j];
			}
		}

		return result;
	}
};

#endif /* SIMMX_H_ */
