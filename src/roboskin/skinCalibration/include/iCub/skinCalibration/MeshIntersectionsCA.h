#ifndef __ICUB_MESHINTERSECTIONSCA_H__
#define __ICUB_MESHINTERSECTIONSCA_H__

#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/skinDynLib/common.h>
#include <iCub/ctrl/math.h>

#include <map>
#include <fstream>
#include "iCub/skinCalibration/CalibrationAlgorithm.h"
#include "iCub/skinCalibration/MeshBodyPart.h"
#include "iCub/skinCalibration/CalibrationBuffer.h"
#include "iCub/skinCalibration/SkinCalibrationLogger.h"
#include "yarp/math/SVD.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


namespace iCub{
	namespace skinCalibration{

		class MeshIntersectionsCA: public CalibrationAlgorithm{

				map<long int, double> w;
				map<long int, double> counterConfidence;
				MeshBodyPart *meshBP;
				BodyPart robotBodyPart;
				SkinPart skinPart;
				string filefullpath;
				string meshfilename;
				double scale_factor;
				Matrix rotRefFrame;
				Vector traslRefFrame;
				double estimateCorrection;
				double maxWeight;
				double weightCorrection;

				bool parseFile(string filename);
				void skewSymmetric(Vector *,Matrix *);
				Matrix computeWeightMatrix(CalibrationSample *);

				bool UsePreviousInformation;
				bool UseWeightMatrix;
				bool SaveWeight;
				unsigned int maxTaxelPerSample;
			    static const int maxNumAxes = 20000;

			public:
				MeshIntersectionsCA();
				~MeshIntersectionsCA();
				virtual bool init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate);
				virtual bool calibrate(CalibrationBuffer *cb, map<long int, Vector> *estimates);
		};

	}
}

#endif //__ICUB_MESHINTERSECTIONSCA_H__
