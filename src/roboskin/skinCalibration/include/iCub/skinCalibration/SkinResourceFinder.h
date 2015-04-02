#ifndef __ICUB_SKINRESOURCEFINDER_H__
#define __ICUB_SKINRESOURCEFINDER_H__

#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>
#include <iCub/skinDynLib/common.h>
#include <iCub/skinCalibration/SkinCalibrationLogger.h>
#include <map>
#include <vector>
#include <fstream>

namespace iCub{
	namespace skinCalibration{
		class SkinResourceFinder{
			std::map<iCub::skinDynLib::BodyPart, std::map<iCub::skinDynLib::SkinPart, std::map<long int, yarp::sig::Vector > > > taxelPositions;
            std::map<iCub::skinDynLib::BodyPart, std::map<iCub::skinDynLib::SkinPart, std::map<long int, yarp::sig::Vector > > > taxelOrientations;
            std::map<iCub::skinDynLib::BodyPart, std::map<iCub::skinDynLib::SkinPart, bool > > arePosesUptodate;
            yarp::os::Port  skinManagerRpcPort;

            void addToBottle(yarp::os::Bottle& b, const yarp::sig::Vector& v);
            void addToBottle(yarp::os::Bottle& b, const std::vector<yarp::sig::Vector>& v);
            void addPosesToBottle(yarp::os::Bottle& b, const std::map<long int, yarp::sig::Vector>& v);
            bool bottleToVector(const yarp::os::Bottle& b, yarp::sig::Vector& v);
            bool updatePoses(iCub::skinDynLib::BodyPart robotBP, iCub::skinDynLib::SkinPart skinP);
			
		public:
			SkinResourceFinder(const char* name);
			~SkinResourceFinder();
            bool init();

			bool setTaxelPosition(iCub::skinDynLib::BodyPart robotBP, iCub::skinDynLib::SkinPart skinP, long int taxel_id, yarp::sig::Vector* position);
            bool setTaxelPositions(iCub::skinDynLib::BodyPart robotBP, iCub::skinDynLib::SkinPart skinP, std::map<long int, yarp::sig::Vector>* position);
			yarp::sig::Vector* getTaxelPosition(iCub::skinDynLib::BodyPart robotBP, iCub::skinDynLib::SkinPart skinP, long int taxel_id);
			void getTaxelPositions(iCub::skinDynLib::BodyPart robotBP, iCub::skinDynLib::SkinPart skinP, std::map<long int, yarp::sig::Vector> *positions);
		};
	}
}

#endif //__ICUB_SKINRESOURCEFINDER_H__
