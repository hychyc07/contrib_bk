#ifndef __ICUB_CALIBRATIONALGORITHM_H__
#define __ICUB_CALIBRATIONALGORITHM_H__

#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <iCub/skinDynLib/common.h>

#include "iCub/skinCalibration/CalibrationBuffer.h"

using namespace std;
using namespace iCub::skinDynLib;
using namespace yarp::os;
using namespace yarp::sig;

namespace iCub{
	namespace skinCalibration{
		class CalibrationAlgorithm{
			string name;
		protected:
			bool initialized;
		public:
			CalibrationAlgorithm(string name){this->name = name;};
			virtual ~CalibrationAlgorithm(){};
			string getName(){return name;};
			virtual bool init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate) = 0;
			virtual bool isInitialized(){return initialized;};
			virtual bool calibrate(CalibrationBuffer *cb,map<long int, Vector> *estimates) = 0;
		};
	}
}

#endif //__ICUB_CALIBRATIONALGORITHM_H__