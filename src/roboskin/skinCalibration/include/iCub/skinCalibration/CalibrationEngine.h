#ifndef __ICUB_CALIBRATIONENGINE_H__
#define __ICUB_CALIBRATIONENGINE_H__

#include <queue>
#include <map>
#include <yarp/os/Thread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Event.h>

#define DEFINE_ALGORITHM(NAME) new NAME(),
#define APEX(y) #y
#define INCLUDE(x) APEX(iCub/skinCalibration/x.h)

#include "iCub/skinCalibration/CalibrationBuffer.h"
#include "iCub/skinCalibration/CalibrationAlgorithm.h"
#include "iCub/skinCalibration/CalibrationAlgorithmList.h"
//#include "iCub/skinCalibration/SkinResourceFinder.h"
#include "iCub/skinForceControl/skinManagerClient.h"

#include "iCub/skinCalibration/SkinCalibrationLogger.h"

using namespace std;
using namespace yarp::os;
using namespace iCub::skinForceControl;

#define INITIALIZE_ALGORITHM_LIST CalibrationAlgorithm* const CalibrationEngine::algorithms[] = {ALGORITHM_TABLE(DEFINE_ALGORITHM) '\0'};

namespace iCub{
	namespace skinCalibration{
		class CalibrationEngine: public Thread{

				Semaphore qaccess;
				Semaphore changealgorithm;
				ResourceFinder *rf;
				Event startprocessing;
				queue<CalibrationBuffer* > ceQueue;
				int nalgorithms;
				CalibrationAlgorithm *ca;
				map<long int, Vector> position_estimates;
				//SkinResourceFinder srf;
				skinManagerClient srf;
				BodyPart robotBodyPart;
				SkinPart skinPart;
				bool isInitialized;
				
			public:
				static CalibrationAlgorithm* const algorithms[];
				CalibrationEngine(ResourceFinder *resf, BodyPart robotBP, SkinPart skinP);
				~CalibrationEngine();
				void push(CalibrationBuffer *cb);
                virtual bool threadInit();
				virtual void run();
				virtual void onStop();
				int getAlgorithmsNumber();
				bool changeCalibrationAlgorithm(int calnum);
		};

		
	}
}

#endif //__ICUB_CALIBRATIONENGINE_H__
