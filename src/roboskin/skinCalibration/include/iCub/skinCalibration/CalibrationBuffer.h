#ifndef __ICUB_CALIBRATIONBUFFER_H__
#define __ICUB_CALIBRATIONBUFFER_H__


#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <stdexcept>
#include <vector>
#include "iCub/skinCalibration/CalibrationSample.h"
#include "iCub/skinCalibration/SkinCalibrationLogger.h"


using namespace std;
using namespace yarp::sig;

namespace iCub{
	namespace skinCalibration{

		class CalibrationBuffer{

			int count;
			int size;
			bool overflow;
			Matrix* forces;
			Vector* moments;
			Vector taxel_measures;
			long int taxel_id;
			vector<CalibrationSample *> samples;
			public:
				CalibrationBuffer(long int taxelid, int size);
				~CalibrationBuffer();
				bool push(CalibrationSample *);
				long int getTaxelID();
				Matrix* getForces();
				Vector* getMoments();
				Vector* getTaxelMeasures();
				const vector<CalibrationSample *> *getSamples();
				int getCount();
				int getSize();
				bool isFull();
				bool isEmpty();
		};

	}
}

#endif //__ICUB_CALIBRATIONBUFFER_H__