#ifndef __ICUB_CALIBRATIONSAMPLE_H__
#define __ICUB_CALIBRATIONSAMPLE_H__

//#ifdef SKIN_CALIBRATION_USE_DOUBLE
//	#define SKIN_CALIBRATION_TYPE double
//#else
//	#define SKIN_CALIBRATION_TYPE float
//#endif

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <iCub/skinDynLib/skinContact.h>
#include <map>

using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::skinDynLib;
using namespace std;

namespace iCub{
	namespace skinCalibration{

		class CalibrationSample{

				int *counter;
				CalibrationSample();
				static long int allocated;
				static long int heavyweight;
				static Semaphore statistics;
				Semaphore *access;
			public:
				long int taxel_id;
				Vector *force;
				Vector *moment;
				Matrix *Rw; //Rotation matrix of the link where the skin is attached to w.r.t the base ref frame
				Vector *ow; //Position of the link where the skin is attached to w.r.t the base ref frame
				Matrix *Rws; //Orientation of the link where the skin is attached to w.r.t the F/T sensor ref frame
				Vector *ows; //Position of the link where the skin is attached to w.r.t the F/T sensor ref frame
				map<long int,double> *taxels_measures;

				CalibrationSample(long int taxel_id, Vector &force, Vector &moment, map<long int,double> &taxels_measures, Matrix &R_ws, Vector &o_ws, Matrix &R_w, Vector &o_w);
				~CalibrationSample();
				CalibrationSample* CloneFor(long int this_taxel_id);
				static long int getAllocated();
				static long int getHeavyWeight();
				bool lastRemaining();
				
		};

	}
}

#endif //__ICUB_CALIBRATIONSAMPLE_H__