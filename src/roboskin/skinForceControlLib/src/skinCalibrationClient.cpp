#include "iCub/skinForceControl/skinCalibrationClient.h"

using namespace yarp::os;
using namespace iCub::skinCalibration;

skinCalibrationClient::skinCalibrationClient(const char* name)
{
    portname = name;
}
//-----------------------------------------------------------------------------------------------------
skinCalibrationClient::~skinCalibrationClient()
{
    skinCalibrationRpcPort.interrupt();
    skinCalibrationRpcPort.close();
}
//-----------------------------------------------------------------------------------------------------
bool skinCalibrationClient::init()
{
	skinCalibrationRpcPort.open((string("/")+portname+"/skinCalibrationClientRpc").c_str());
	return Network::connect(skinCalibrationRpcPort.getName().c_str(), "/skinCalibration/rpc");
}

bool skinCalibrationClient::setAlgorithm(int algorithmnum){
	Bottle b, reply;
    b.addInt(set_algorithm);
    b.addInt(algorithmnum);
	if(!skinCalibrationRpcPort.write(b, reply))
        return false;
    return true;
}

bool skinCalibrationClient::pauseCalibration(){
	Bottle b;
    b.addInt(pause_calibration);
	if(!skinCalibrationRpcPort.write(b))
        return false;
    return true;
}

bool skinCalibrationClient::startCalibration(){
	Bottle b;
    b.addInt(start_calibration);
	if(!skinCalibrationRpcPort.write(b))
        return false;
    return true;
}

bool skinCalibrationClient::quitModule(){
	Bottle b;
    b.addInt(quit);
	if(!skinCalibrationRpcPort.write(b))
        return false;
    return true;
}