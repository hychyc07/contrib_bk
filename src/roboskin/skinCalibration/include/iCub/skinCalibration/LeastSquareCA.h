#ifndef __ICUB_LEASTSQUARECA_H__
#define __ICUB_LEASTSQUARECA_H__

#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/skinDynLib/common.h>

#include <map>
#include <fstream>
#include "iCub/skinCalibration/CalibrationAlgorithm.h"
#include "iCub/skinCalibration/CalibrationBuffer.h"
#include "iCub/skinCalibration/SkinCalibrationLogger.h"
#include "yarp/math/SVD.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


namespace iCub{
	namespace skinCalibration{

		class LeastSquareCA: public CalibrationAlgorithm{

				map<long int, Matrix *> H;
				BodyPart robotBodyPart;
				SkinPart skinPart;
				string filefullpath;

				bool parseFile(string filename);
				void skewSymmetric(Vector *,Matrix *);
				Matrix computeWeightMatrix(CalibrationSample *);

				bool UsePreviousInformation;
				bool UseWeightMatrix;

			public:
				LeastSquareCA():CalibrationAlgorithm("LeastSquare"){};
				~LeastSquareCA();
				virtual bool init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate);
				virtual bool calibrate(CalibrationBuffer *cb, map<long int, Vector> *estimates);
		};

	}
}

#endif //__ICUB_LEASTSQUARECA_H__
