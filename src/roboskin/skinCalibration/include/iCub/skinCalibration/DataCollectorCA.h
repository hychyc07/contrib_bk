#ifndef __ICUB_DATACOLLECTORCA_H__
#define __ICUB_DATACOLLECTORCA_H__

#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/os/BufferedPort.h>
#include <iCub/skinDynLib/common.h>

#include <map>
#include <fstream>
#include <algorithm>
#include <sstream>
#include "iCub/skinCalibration/CalibrationAlgorithm.h"
#include "iCub/skinCalibration/CalibrationBuffer.h"
#include "yarp/math/Math.h"
#include "yarp/math/SVD.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


namespace iCub{
	namespace skinCalibration{

		class DataCollectorCA: public CalibrationAlgorithm{
				
				map< long int, vector< Vector > > Forces;
				map< long int, vector< Vector > > Moments;
				map< long int, vector< Vector > > T;
				vector<long int> taxel_id;
				BodyPart robotBodyPart;
				SkinPart skinPart;
				string filefullpath;
				BufferedPort<Bottle> dataPort;
				bool SaveData;
				
			public:
				DataCollectorCA():CalibrationAlgorithm("DataCollector"){};
				~DataCollectorCA();
				virtual bool init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate);
				virtual bool calibrate(CalibrationBuffer *cb, map<long int, Vector> *estimates);
		};

	}
}

#endif //__ICUB_DATACOLLECTORCA_H__
