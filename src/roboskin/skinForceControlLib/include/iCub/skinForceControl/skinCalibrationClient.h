
#ifndef __ICUB_SKINCALIBRATIONCLIENT_H__
#define __ICUB_SKINCALIBRATIONCLIENT_H__

#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>
#include <iCub/skinDynLib/common.h>
#include <map>
#include <vector>
#include <fstream>
#include <string.h>

//#include "iCub/skinCalibration/SkinCalibrationCtrlInterface.h"

using namespace std;
using namespace yarp::os;

namespace iCub{
	namespace skinCalibration{
        enum SkinCalibrationCtrlCommand{quit,help,get_algorithm_list, set_algorithm, start_calibration, pause_calibration, SCC_COMMAND_COUNT};

		const std::string SkinCalibrationCtrlCommand_s[]  = {"quit","help","get algorithm list","set algorithm","start_calibration","pause_calibration"};

		const std::string SkinCalibrationCtrlCommand_desc[]  = {"quit the module", "get this list","returns the skin calibration algorithm list", "set the skin calibration algorithm","start the calibration process","pause the calibration process"};

		class skinCalibrationClient{
            Port   skinCalibrationRpcPort;
			string portname;

		public:
			skinCalibrationClient(const char* name);
			~skinCalibrationClient();
            bool init();

			bool setAlgorithm(int);
			bool pauseCalibration();
			bool startCalibration();
			bool quitModule();
			
		};
	}
}

#endif //__ICUB_SKINCALIBRATIONCLIENT_H__