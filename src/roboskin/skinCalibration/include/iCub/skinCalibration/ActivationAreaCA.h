#ifndef __ICUB_ACTIVATIONAREACA_H__
#define __ICUB_ACTIVATIONAREACA_H__

#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/skinDynLib/common.h>
#include <iCub/ctrl/math.h>

#include <map>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include "iCub/skinCalibration/SkinCalibrationLogger.h"
#include "iCub/skinCalibration/CalibrationAlgorithm.h"
#include "iCub/skinCalibration/CalibrationBuffer.h"
#include "iCub/skinCalibration/MeshBodyPart.h"
#include "yarp/math/Math.h"
#include "yarp/math/SVD.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


namespace iCub{
	namespace skinCalibration{

		class ActivationAreaCA: public CalibrationAlgorithm{

				
				
				map<long int, vector<Vector> > activation_points;
				MeshBodyPart *meshBP;
				BodyPart robotBodyPart;
				SkinPart skinPart;
				string filefullpath;
				string meshfilename;
				vector<long int> taxel_id;
				double scale_factor;
				Matrix rotRefFrame;
				Vector traslRefFrame;

				bool parseFile(string filename);
				void skewSymmetric(Vector *,Matrix *);

			public:
				ActivationAreaCA();
				~ActivationAreaCA();
				virtual bool init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate);
				virtual bool calibrate(CalibrationBuffer *cb, map<long int, Vector> *estimates);
		};

	}
}

#endif //__ICUB_ACTIVATIONAREACA_H__
