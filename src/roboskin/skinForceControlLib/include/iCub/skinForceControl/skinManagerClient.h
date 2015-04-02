#ifndef __ICUB_SKINRESOURCEFINDER_H__
#define __ICUB_SKINRESOURCEFINDER_H__

#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>
#include <iCub/skinDynLib/common.h>
#include <map>
#include <vector>
#include <fstream>
#include <string.h>

namespace iCub{
	namespace skinForceControl{
		class skinManagerClient{
            yarp::os::Port  skinManagerRpcPort;
			std::string portname;

            void addPosesToBottle(yarp::os::Bottle& b, const std::vector<yarp::sig::Vector>& v);
			
		public:
			skinManagerClient(const char* name);
			~skinManagerClient();
            bool init();

			bool setTaxelPosition(iCub::skinDynLib::SkinPart skinP, long int taxel_id, const yarp::sig::Vector &pos);
            bool setTaxelPositions(iCub::skinDynLib::SkinPart skinP, const std::vector<yarp::sig::Vector> &pos);
			const yarp::sig::Vector getTaxelPosition(iCub::skinDynLib::SkinPart skinP, long int taxel_id);
			const std::vector<yarp::sig::Vector> getTaxelPositions(iCub::skinDynLib::SkinPart skinP);
			const yarp::sig::Vector getTaxelPositionAndConfidence(iCub::skinDynLib::SkinPart skinP, long int taxel_id);
			const std::vector<yarp::sig::Vector> getTaxelPositionsAndConfidences(iCub::skinDynLib::SkinPart skinP);
			double getPoseConfidence(iCub::skinDynLib::SkinPart skinP, long int taxel_id);
			yarp::sig::Vector getPoseConfidences(iCub::skinDynLib::SkinPart sp);
		};
	}
}

#endif //__ICUB_SKINRESOURCEFINDER_H__
