#include <yarp/math/Math.h>
#include "iCub/skinForceControl/skinForceControlClient.h"
#include "iCub/skinDynLib/rpcSkinManager.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace iCub::skinManager;
using namespace std;
using namespace iCub::skinForceControl;


skinForceControlClient::skinForceControlClient(const char* _name, const char* _sfcName): name(_name)
{
    if(_sfcName==NULL)
        sfcName = "skinForceControl";
    else
        sfcName = _sfcName;
    lastReadTimestamp = 0.0;
}
//----------------------------------------------------------------------------------------
skinForceControlClient::~skinForceControlClient()
{
    rpcPort.interrupt();
    streamPort.interrupt();
    rpcPort.close();
    streamPort.close();
}
//----------------------------------------------------------------------------------------
bool skinForceControlClient::init()
{
    if(!rpcPort.open((string("/")+name+"/sfc_rpc").c_str()))
        return false;
    if(!streamPort.open((string("/")+name+"/sfc_stream:i").c_str()))
        return false;
    
	if( !Network::connect(rpcPort.getName().c_str(), ("/"+sfcName+"/rpc").c_str()))
        return false;
    
    if( !Network::connect(("/"+sfcName+"/monitor:o").c_str(), streamPort.getName().c_str()))
        return false;
    streamPort.useCallback(*this);
    
    return true;
}
//----------------------------------------------------------------------------------------
void skinForceControlClient::onRead(Vector& v)
{
    mutex.wait();
    streamData.setData(v);
    lastReadTimestamp=yarp::os::Time::now(); 
    mutex.post();
}
//----------------------------------------------------------------------------------------
bool skinForceControlClient::getStreamData(SfcMonitorData &data)
{   
    if(Time::now()-lastReadTimestamp>1.0)
        return false;
    mutex.wait();
    data = streamData; 
    mutex.post();
    return true;
}
//----------------------------------------------------------------------------------------
bool skinForceControlClient::startControl(ControlLaw cl)
{
    switch(cl)
    {
        case FLOAT_CTRL:            return sendCommand(float_ctrl);
        case JPOS_CTRL:             return sendCommand(jpos_ctrl);
        case POS_CTRL:              return sendCommand(pos_ctrl);
        case TORQUE_CTRL:           return sendCommand(torque_ctrl);
        case FORCE_CTRL:            return sendCommand(force_ctrl);
        case PARAL_CTRL:            return sendCommand(paral_ctrl);
        case DYN_PARAL_CTRL:        return sendCommand(dparal_ctrl);
        case PRESS_CTRL:            return sendCommand(press_ctrl);
        case CONTACT_CTRL:          return sendCommand(cont_ctrl);
        case SAFE_REACH_RIGID_CTRL: return sendCommand(reach_ctrl);
    }
    printf("Unrecognized control law %d", cl);
    return false;
}

bool skinForceControlClient::setCtrlPnt(unsigned int linkNum, const Vector &xd)
{ 
    Bottle b, reply;
    stringstream command(SfcCommand_s[set_ctrlPnt]);
	string word;
	while(command>>word){
		b.addString(word.c_str());
	}
	b.addInt(linkNum);
    addToBottle(b, xd);
	if(!rpcPort.write(b, reply))
        return false;
    return true;
}

bool skinForceControlClient::setXd(unsigned int linkNum, const Vector &xdenv)
{ 
    Bottle b, reply;
    stringstream command(SfcCommand_s[set_xd]);
	string word;
	while(command>>word){
		b.addString(word.c_str());
	}
	b.addInt(linkNum);
    addToBottle(b, xdenv);
	if(!rpcPort.write(b, reply))
        return false;
    return true;
}


// ***************************************************************************************
// *********************************** PRIVATE MEMBERS ***********************************
// ***************************************************************************************
bool skinForceControlClient::sendCommand(SfcCommand c)
{
    Bottle b, reply;
    stringstream command(SfcCommand_s[c]);
	string word;
	while(command>>word){
		b.addString(word.c_str());
	}
	if(!rpcPort.write(b, reply))
        return false;
    return true;
}
//----------------------------------------------------------------------------------------
bool skinForceControlClient::sendCommand(SfcCommand c, int d)
{
    Bottle b, reply;
    stringstream command(SfcCommand_s[c]);
	string word;
	while(command>>word){
		b.addString(word.c_str());
	}
    b.addInt(d);
	if(!rpcPort.write(b, reply))
        return false;
    return true;
}
//----------------------------------------------------------------------------------------
bool skinForceControlClient::sendCommand(SfcCommand c, double d)
{
    Bottle b, reply;
    stringstream command(SfcCommand_s[c]);
	string word;
	while(command>>word){
		b.addString(word.c_str());
	}
    b.addDouble(d);
	if(!rpcPort.write(b, reply))
        return false;
    return true;
}
//----------------------------------------------------------------------------------------
bool skinForceControlClient::sendCommand(SfcCommand c, const Vector &v)
{
    Bottle b, reply;
    stringstream command(SfcCommand_s[c]);
	string word;
	while(command>>word){
		b.addString(word.c_str());
	}
    addToBottle(b, v);
	if(!rpcPort.write(b, reply))
        return false;
    return true;
}
//----------------------------------------------------------------------------------------
Vector skinForceControlClient::sendCommandThatReturnsVector(SfcCommand c)
{
    Bottle b, reply;
    Vector v;
    stringstream command(SfcCommand_s[c]);
	string word;
	while(command>>word){
		b.addString(word.c_str());
	}
	if(rpcPort.write(b, reply))
        if(!bottleToVector(reply, v))
            printf("[skinForceControlClient] Unexpected reply to command %s: %s\n", SfcCommand_s[c].c_str(), reply.toString().c_str());
    else
        printf("[skinForceControlClient] Write on rpc port failed.\n");
    return v;
}
//----------------------------------------------------------------------------------------
double skinForceControlClient::sendCommandThatReturnsDouble(SfcCommand c)
{
    Bottle b, reply;
    double v=0.0;
    stringstream command(SfcCommand_s[c]);
	string word;
	while(command>>word){
		b.addString(word.c_str());
	}
	if(rpcPort.write(b, reply))
        if(reply.size()>0 && (reply.get(0).isDouble() || reply.get(0).isInt()))
            v = reply.get(0).asDouble();
        else
            printf("[skinForceControlClient] Unexpected reply to command %s: %s\n", SfcCommand_s[c].c_str(), reply.toString().c_str());
    else
        printf("[skinForceControlClient] Write on rpc port failed.\n");
    return v;
}
//----------------------------------------------------------------------------------------
int skinForceControlClient::sendCommandThatReturnsInt(SfcCommand c)
{
    Bottle b, reply;
    int v=0;
    stringstream command(SfcCommand_s[c]);
	string word;
	while(command>>word){
		b.addString(word.c_str());
	}
	if(rpcPort.write(b, reply))
        if(reply.size()>0 && reply.get(0).isInt())
            v = reply.get(0).asInt();
        else
            printf("[skinForceControlClient] Unexpected reply to command %s: %s\n", SfcCommand_s[c].c_str(), reply.toString().c_str());
    else
        printf("[skinForceControlClient] Write on rpc port failed.\n");
    return v;
}
