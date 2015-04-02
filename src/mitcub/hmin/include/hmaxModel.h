/*
 * The software presented in this package is largely based on 
 * HMIN (http://cbcl.mit.edu/jmutch/hmin/) by Jim Mutch.
 * Most of the classes and their implementation have been designed and
 * written by Jim Mutch.
 *
 * You are permitted to reproduce, modify and redistribute this software
 * according to the terms and conditions stated in "License.txt" as long as
 * you reproduce this note in its entirety and without any modification and 
 * ship it alongside the software.
 */

#ifndef _HMAX_MODEL_H_
#define _HMAX_MODEL_H_

#include <stdlib.h>
#include "layer.h"
#include "filter.h"
#include "gaborfilter.h"
#include "maxfilter.h"
#include "ndpfilter.h"
#include "gmaxfilter.h"
#include "main.h"
#include "dictionary.h"
#include <cv.h>

using namespace std;

class hmaxModel;

class hmaxModel {
		private:
				int depth;

				GaborFilter *fs1;
				MaxFilter   *fc1;
				NDPFilter	*fs2;
				GMaxFilter	*fc2;
				
				int s1SStep;
				int c1SStep;
				int s2SStep;
				int c2SStep;	

				int nsi;
				int ns1;
				int nc1;
				int ns2;
				int nc2;

				float 		**sub_buffer;
				IplImage 	**subsample;

				Layer **si;
				Layer **s1;
				Layer **c1;
				Layer **s2;
				Layer **c2;

				void scaleImage(IplImage *img, float **in);

		public:
				hmaxModel(int _depth, dictionary *_dict);
				void response(IplImage *img, float *out);
				int FCount() {return (int) nc2 * fs2->FCount();}
				~hmaxModel();
};				
#endif

