#ifndef __ICUB_CALIBRATIONMODULE_H__
#define __ICUB_CALIBRATIONMODULE_H__

#include <sstream>
#include <yarp/os/RFModule.h>
#include <yarp/math/Math.h>

#include "iCub/skinCalibration/CalibrationThread.h"
#include "iCub/skinDynLib/common.h"
#include "iCub/skinCalibration/CalibrationSamplesCollector.h"
#include "iCub/skinCalibration/CalibrationEngine.h"
#include "iCub/skinForceControl/skinCalibrationClient.h"
#include "iCub/skinCalibration/SkinCalibrationLogger.h"

using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace std;

namespace iCub
{	namespace skinCalibration
	{
		class CalibrationModule: public RFModule
		{
			CalibrationThread *ct;
			CalibrationSamplesCollector *csc;
			CalibrationEngine *ce;

			Port rpcPort;

			bool identifyCommand(const Bottle &commandBot, SkinCalibrationCtrlCommand &com, Bottle &param);

			public:
				bool configure(ResourceFinder &rf);
				bool respond(const Bottle& command, Bottle& reply);
				bool updateModule();
				bool close();
		};
	}
}

#endif //__ICUB_CALIBRATIONMODULE_H__
