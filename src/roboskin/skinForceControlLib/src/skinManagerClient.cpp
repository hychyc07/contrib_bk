#include <yarp/math/Math.h>
#include "iCub/skinForceControl/skinManagerClient.h"
#include "iCub/skinForceControl/util.h"
#include "iCub/skinDynLib/rpcSkinManager.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace iCub::skinManager;
using namespace std;
using namespace iCub::skinForceControl;

skinManagerClient::skinManagerClient(const char* name)
{
    portname = name;
}
//-----------------------------------------------------------------------------------------------------
skinManagerClient::~skinManagerClient()
{
    skinManagerRpcPort.interrupt();
    skinManagerRpcPort.close();
}
//-----------------------------------------------------------------------------------------------------
bool skinManagerClient::init()
{
	skinManagerRpcPort.open((string("/")+portname+"/skinManagerRpc").c_str());
	return Network::connect(skinManagerRpcPort.getName().c_str(), "/skinManager/rpc");
}
//-----------------------------------------------------------------------------------------------------
bool skinManagerClient::setTaxelPosition(SkinPart sp, long int taxel_id, const Vector &pos)
{
    Bottle b, reply;
    b.addInt(set_position);
    b.addInt(sp);
    b.addInt(taxel_id);
    addToBottle(b, pos);
	if(!skinManagerRpcPort.write(b, reply))
        return false;

    return true;
}
//-----------------------------------------------------------------------------------------------------
bool skinManagerClient::setTaxelPositions(SkinPart sp, const vector<Vector> &pos)
{
    Bottle b, reply;
    b.addInt(set_position);
    b.addInt(sp);
    addPosesToBottle(b, pos);
	if(!skinManagerRpcPort.write(b, reply))
        return false;

    return true;
}

//-----------------------------------------------------------------------------------------------------
const Vector skinManagerClient::getTaxelPosition(SkinPart sp, long int taxel_id)
{
	Vector taxelPosition;
	Bottle b, reply;
    b.addInt(get_position);
    b.addInt(sp);
	b.addInt(taxel_id);
    if(!skinManagerRpcPort.write(b, reply))
        return taxelPosition;
	
    if(reply.size()%4 != 0){
        printf("[skinManagerClient] Something went wrong trying to get taxel poses. Vector size not a multiple of 4 (reply=%s)\n", reply.toString().c_str());
        return taxelPosition;
    }
	taxelPosition.resize(3);
	taxelPosition[0] = reply.get(0).asDouble();
    taxelPosition[1] = reply.get(1).asDouble();
    taxelPosition[2] = reply.get(2).asDouble();
   
    return taxelPosition;
}

//-----------------------------------------------------------------------------------------------------
const vector<Vector> skinManagerClient::getTaxelPositions(SkinPart sp)
{
    //if(!arePosesUptodate[sp])
    //    updatePoses(sp);
	vector<Vector> taxelPositions;
	Bottle b, reply;
    b.addInt(get_position);
    b.addInt(sp);
    if(!skinManagerRpcPort.write(b, reply))
        return taxelPositions;
    double x, y, z;
	
    if(reply.size()%4 != 0){
        printf("[skinManagerClient] Something went wrong trying to get taxel poses. Vector size not a multiple of 4 (reply=%s)\n", reply.toString().c_str());
        return taxelPositions;
    }
	taxelPositions.resize(reply.size()/4);
    for(int i=0; i<reply.size()/4; i++){
        x = reply.get(4*i).asDouble();
        y = reply.get(4*i+1).asDouble();
        z = reply.get(4*i+2).asDouble();
        taxelPositions[i].resize(3);
		taxelPositions[i][0] = x;
        taxelPositions[i][1] = y;
        taxelPositions[i][2] = z;
    }
    
    return taxelPositions;
}

//-----------------------------------------------------------------------------------------------------
const Vector skinManagerClient::getTaxelPositionAndConfidence(SkinPart sp, long int taxel_id)
{
    //if(!arePosesUptodate[sp])
    //    updatePoses(sp);
	Vector taxelPosition;
	Bottle b, reply;
    b.addInt(get_position);
    b.addInt(sp);
	b.addInt(taxel_id);
    if(!skinManagerRpcPort.write(b, reply))
        return taxelPosition;
	
    if(reply.size()%4 != 0){
        printf("[skinManagerClient] Something went wrong trying to get taxel poses. Vector size not a multiple of 4 (reply=%s)\n", reply.toString().c_str());
        return taxelPosition;
    }
	taxelPosition.resize(4);
	taxelPosition[0] = reply.get(0).asDouble();
    taxelPosition[1] = reply.get(1).asDouble();
    taxelPosition[2] = reply.get(2).asDouble();
	taxelPosition[3] = reply.get(3).asDouble();
   
    return taxelPosition;
}
//-----------------------------------------------------------------------------------------------------
const vector<Vector> skinManagerClient::getTaxelPositionsAndConfidences(SkinPart sp)
{
	vector<Vector> taxelPositions;
	Bottle b, reply;
    b.addInt(get_position);
    b.addInt(sp);
    if(!skinManagerRpcPort.write(b, reply))
        return taxelPositions;
    double x, y, z, w;
	
    if(reply.size()%4 != 0){
        printf("[skinManagerClient] Something went wrong trying to get taxel poses. Vector size not a multiple of 4 (reply=%s)\n", reply.toString().c_str());
        return taxelPositions;
    }
	taxelPositions.resize(reply.size()/4);
    for(int i=0; i<reply.size()/4; i++){
        x = reply.get(4*i).asDouble();
        y = reply.get(4*i+1).asDouble();
        z = reply.get(4*i+2).asDouble();
        w = reply.get(4*i+3).asDouble();
        taxelPositions[i].resize(4);
		taxelPositions[i][0] = x;
        taxelPositions[i][1] = y;
        taxelPositions[i][2] = z;
		taxelPositions[i][3] = w;
    }
    
    return taxelPositions;
}

Vector skinManagerClient::getPoseConfidences(SkinPart sp){
    //printf("get pose confidences result: ");
	Bottle b, reply;
    Vector res(0);
    b.addInt(get_confidence);
    b.addInt(sp);
    if(!skinManagerRpcPort.write(b, reply))
        return res;
	if(reply.size() > 0)
		bottleToVector(reply, res);
	//printf("%s\n", res.toString(1).c_str());
    return res;
}

double skinManagerClient::getPoseConfidence(SkinPart sp, long int taxel_id){
	Bottle b, reply;
    b.addInt(get_confidence);
    b.addInt(sp);
	b.addInt(taxel_id);
    if(!skinManagerRpcPort.write(b, reply))
        return -1.0;
	if(reply.size() >= 1)
		return reply.get(0).asDouble();
	return -1.0;
}

//-----------------------------------------------------------------------------------------------------
void skinManagerClient::addPosesToBottle(Bottle& b, const vector<Vector>& v)
{
    for(vector<Vector>::const_iterator it=v.begin(); it!=v.end(); it++){
        unsigned int j=0;
        for(; j<it->size(); j++)        b.addDouble((*it)[j]);
        //for(; j<6; j++)                 b.addDouble(0.0);               // fill the missing data with zeros
    }
}
