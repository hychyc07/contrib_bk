#ifndef __ICUB_CALIBRATIONSAMPLESCOLLECTOR_H__
#define __ICUB_CALIBRATIONSAMPLESCOLLECTOR_H__

#include <map>
#include "iCub/skinCalibration/CalibrationSample.h"
#include "iCub/skinCalibration/CalibrationBuffer.h"
#include "iCub/skinCalibration/CalibrationEngine.h"
#include "iCub/skinCalibration/CalibrationAlgorithm.h"

using namespace std;

namespace iCub{
	namespace skinCalibration{

		class CalibrationSamplesCollector{
			map<long int, CalibrationBuffer *> mapping;
			CalibrationEngine *ce;
			unsigned int bsize;     // Del Prete: I changed these to unsigned int
			unsigned int bnum;
			public:
				CalibrationSamplesCollector(CalibrationEngine *calibration_engine, int buffer_size, int buffer_number=0);
				~CalibrationSamplesCollector();
				bool push(CalibrationSample *calibration_sample);
				bool push(Vector &force,Vector &moment, map<long int, double>& taxels_measures,Matrix &R_ws, Vector &o_ws,Matrix &R_w, Vector &o_w);
		};

	}
}

#endif
