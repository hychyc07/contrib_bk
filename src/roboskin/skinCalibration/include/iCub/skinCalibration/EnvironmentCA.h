#ifndef __ICUB_ENVIRONMENTCA_H__
#define __ICUB_ENVIRONMENTCA_H__

#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/skinDynLib/common.h>

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

		class EnvironmentCA: public CalibrationAlgorithm{


				typedef struct trilat_struct
				{
					Vector center;
					Vector normal;
					double radius;
				} trilat_record;
				
				MeshBodyPart *meshBP;
				BodyPart robotBodyPart;
				SkinPart skinPart;
				string filefullpath;
				string meshfilename;
				double scale_factor;
				Matrix rotRefFrame;
				Vector traslRefFrame;

				map<long int, vector<trilat_record> > triples;
				double w,centroid_mindist;

				map<long int, double> mean_w;

				double sphere_radius;
				Vector sphere_center;

				bool circ_intersections(Vector c1, Vector c2, double r1, double r2, Vector &i1, Vector &i2);
				bool trilateration(long int id, Vector& pos_estimate);

		public:
				EnvironmentCA();
				~EnvironmentCA();
				virtual bool init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate);
				virtual bool calibrate(CalibrationBuffer *cb, map<long int, Vector> *estimates);
		};
	}
}

#endif //__ICUB_ENVIRONMENTCA_H__