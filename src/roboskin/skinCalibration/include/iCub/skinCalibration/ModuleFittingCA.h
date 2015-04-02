#ifndef __ICUB_MODULEFITTINGCA_H__
#define __ICUB_MODULEFITTINGCA_H__

#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <iCub/skinDynLib/common.h>
#include <map>
#include <fstream>

#include "iCub/skinCalibration/CalibrationAlgorithm.h"
#include "iCub/skinCalibration/MeshBodyPart.h"
#include "iCub/skinCalibration/CalibrationBuffer.h"
#include "iCub/skinCalibration/SkinCalibrationLogger.h"
#include "iCub/skinCalibration/CalibrationEngine.h"
#include "iCub/skinForceControl/skinManagerClient.h"

using namespace std;
using namespace iCub::skinDynLib;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::skinForceControl;

namespace iCub{
	namespace skinCalibration{
		class  ModuleFittingCA: public CalibrationAlgorithm{
			
			MeshBodyPart *meshBP;
			BodyPart robotBodyPart;
			SkinPart skinPart;
			string filefullpath;
			string meshfilename;
			double scale_factor;
			Matrix rotRefFrame;
			Vector traslRefFrame;

			
			
			unsigned int taxelPerModule;
			vector<Matrix>	moduleStructure;
			vector<int>		moduleVertexIndex[3];
			vector<int>		moduleRefTaxel;
			vector<int>		modulePlacingSequence;
			double neighborsMinDist;
			double moduleCentroidDistance;
			
			CalibrationAlgorithm *sub_algorithm;
			skinManagerClient smc;

			void computeModuleRefTaxel();
			bool parseModuleStructureFile(string filename);

			void findNeighbors(long startid, Vector centroid, map<long int, Vector> *estimates, vector<unsigned int> *neighborsStartId);
			bool fitModuleToEstimate(long startid, map<long int, Vector> *estimates, vector<Vector> *newEstimates);
			
		public:
			ModuleFittingCA();
			~ModuleFittingCA();
			virtual bool init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate);
			virtual bool calibrate(CalibrationBuffer *cb,map<long int, Vector> *estimates);
		};
	}
}

#endif //__ICUB_MODULEFITTINGCA_H__