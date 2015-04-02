#ifndef __ICUB_CHOLESKYLEASTSQUARECA_H__
#define __ICUB_CHOLESKYLEASTSQUARECA_H__

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

#include "yarp/math/Math.h"
#include "yarp/math/SVD.h"
#include <gsl/gsl_linalg.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


namespace iCub{
	namespace skinCalibration{

		class CholeskyLeastSquareCA: public CalibrationAlgorithm{

				map<long int, Matrix *> R;
				BodyPart robotBodyPart;
				SkinPart skinPart;
				string filefullpath;

				bool parseFile(string filename);
				void skewSymmetric(Vector *,Matrix *);
				Matrix computeWeightMatrix(CalibrationSample *);

				bool UsePreviousInformation;
				bool UseWeightMatrix;

				void CholeskyFactorization(Matrix& R,int size);
				void CholeskyUpdate(Matrix& R, Vector x);
				void fbSubstitution(Matrix &R, Vector& x);

			public:
				CholeskyLeastSquareCA():CalibrationAlgorithm("CholeskyLeastSquare"){};
				~CholeskyLeastSquareCA();
				virtual bool init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate);
				virtual bool calibrate(CalibrationBuffer *cb, map<long int, Vector> *estimates);
		};

	}
}

#endif //__CHOLESKYICUB_LEASTSQUARECA_H__
