#ifndef __ICUB_CALIBRATIONTHREAD_H__
#define __ICUB_CALIBRATIONTHREAD_H__


#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>
#include <iCub/skinDynLib/common.h>
#include <iCub/iKin/iKinFwd.h>

#include "iCub/skinCalibration/CalibrationSamplesCollector.h"
#include "iCub/skinCalibration/SkinCalibrationLogger.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace iCub::ctrl; 
using namespace std;

//#define SKINCALIBRATION_SIMULATION
namespace iCub
{	namespace skinCalibration
	{
		class CalibrationThread: public RateThread
		{
		        BufferedPort<Vector> *FTDevice;
		        BufferedPort<Vector> *skinDevice;
		        Vector *tempFT;
				Vector *tempSkin;
			#ifndef SKINCALIBRATION_SIMULATION
				PolyDriver *jointsPositionsDevice;
			#else
				BufferedPort<Bottle> *jointsPositionsDevice;
				Bottle *tempJoints;
				bool startup;
				int startup_counter;
				Vector skinDataComp;
				Vector FTDataComp;
				Vector baseline;
			#endif

			double skinvalue;

//			IAnalogSensor* FTSensor, *skinSensor;
			IEncoders* encoders;

			string robotName;
			string name;                                
			double contact_threshold;
			double force_threshold;
			int	max_taxels_in_contact;
			SkinPart skinPart;                          // id of the part of the skin (hand, forearm_lower, arm_internal, ...)
			BodyPart robotBodyPart; 

			string localFTPortName, FTPortName;
			string localSkinPortName, SkinPortName;
			string localJointsPositionsPortName, JointsPositionsPortName;
			string infoPortName; 

			BufferedPort<Bottle> infoPort;

			Vector skinData;
			Vector FTData;
			Vector jointsPositionsData;
			
			iKinLimb *limb;
			int skinLink;
			int FTLink;
			Matrix FTLink2FT;

			CalibrationSamplesCollector *csc;

			void readSkinData();

			long int cycle_counter;

			public:

			CalibrationThread(int period, string name, string robotName, BodyPart robotBodyPart, SkinPart skinPart, int skinLink, int FTLink, Matrix FTLink2FT, double contact_threshold, double force_thrs, int max_txs_in_contact, CalibrationSamplesCollector *calsempcoll);
			virtual bool threadInit();
			virtual void threadRelease();
			virtual void run();
			virtual void stop();

			
		};
	}
}

#endif //__ICUB_CALIBRATIONTHREAD_H__
