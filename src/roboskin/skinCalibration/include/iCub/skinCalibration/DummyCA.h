#ifndef __ICUB_DUMMYCA_H__
#define __ICUB_DUMMYCA_H__

#include <yarp/os/ResourceFinder.h>
#include "iCub/skinCalibration/CalibrationAlgorithm.h"

using namespace std;
using namespace yarp::os;

namespace iCub{
	namespace skinCalibration{
		class DummyCA: public CalibrationAlgorithm{
		public:
			DummyCA():CalibrationAlgorithm("Dummy"){};
			virtual bool init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate){ initialized = true; return true;};
			virtual bool calibrate(CalibrationBuffer *cb, map<long int, Vector> *estimates){return false;};
		};
	}
}

#endif //__ICUB_DUMMYCA_H__
