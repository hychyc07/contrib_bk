#include <yarp/math/Math.h>
#include "iCub/skinCalibration/SkinResourceFinder.h"
#include "iCub/skinDynLib/rpcSkinManager.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace iCub::skinManager;
using namespace std;
using namespace iCub::skinCalibration;

SkinResourceFinder::SkinResourceFinder(const char* name){
    skinManagerRpcPort.open((string("/")+name+"/skinManagerRpc").c_str());
	//ifstream file;
	//filename += "skin_";
	//filename += BodyPart_s[robotBodyPart]+"_"+ SkinPart_s[skinPart];
	//filename = rf->findFile(filename.c_str());
	//if(!filename.empty()){
	//	file.open(filename);
	//	resource.clear();
	//	while(!file.eof()){
	//		int id;
	//		file >> id;
	//		if(file.fail())
	//			continue;
	//		resource[id].resize(3);
	//		for(int j=0; j<3; j++){
	//			file >> resource[id].data()[j];
	//		}
	//		file>>id;
	//		file>>id;
	//		file>>id; //discard orientations
	//	}
	//	file.close();
	//}
	//else{
	//	resource.clear();
	//}
}

SkinResourceFinder::~SkinResourceFinder(){
	//ofstream ofile;
	//filename = rf->getContextPath();
	//filename += "/skin_";
	//filename += BodyPart_s[robotBodyPart]+"_"+ SkinPart_s[skinPart];
	//ofile.open(filename);
	//map<long int, Vector>::iterator it;
	//for(it = resource.begin(); it != resource.end();){
	//	ofile<< it->first;
	//	ofile<<" "<<it->second[0];
	//	ofile<<" "<<it->second[1];
	//	ofile<<" "<<it->second[2];
	//	ofile<<" "<<0<<" "<<0<<" "<<0;
	//	it++;
	//	if(it!=resource.end())
	//		ofile<<endl;
	//}
	//ofile.close();
}

bool SkinResourceFinder::init(){
	return Network::connect(skinManagerRpcPort.getName().c_str(), "/skinManager/rpc");
}

bool SkinResourceFinder::setTaxelPosition(BodyPart bp, SkinPart sp, long int taxel_id, Vector* position){
    if(arePosesUptodate[bp][sp]==false)
        SkinCalibrationLogger::writeMsg("[SkinResourceFinder] WARNING: you are setting the position of a taxel that has not been read before.\n");
    Bottle b, reply;
    b.addInt(set_pose);
    b.addInt(bp);
    b.addInt(sp);
    b.addInt(taxel_id);
    addToBottle(b, *position);
    addToBottle(b, zeros(3));   //add 3 zeros for orientation
	if(!skinManagerRpcPort.write(b, reply))
        return false;
    /*if(!reply.get(0).isInt())
        return false;
    if(reply.get(0).asInt() != skin_manager_ok)
        return false;*/
	
	taxelPositions[bp][sp][taxel_id] = *position;
    return true;
}

bool SkinResourceFinder::setTaxelPositions(BodyPart bp, SkinPart sp, map<long int, Vector> *positions){
    if(arePosesUptodate[bp][sp]==false)
        SkinCalibrationLogger::writeMsg("[SkinResourceFinder] WARNING: you are setting the position of a taxel that has not been read before.\n");
    Bottle b, reply;
    b.addInt(set_pose);
    b.addInt(bp);
    b.addInt(sp);
    addPosesToBottle(b, *positions);
	if(!skinManagerRpcPort.write(b, reply))
        return false;
	//SkinCalibrationLogger::writeMsg("Set pose reply: %s\n", reply.toString().c_str());
    /*if(!reply.get(0).isInt())
        return false;
    if(reply.get(0).asInt() != skin_manager_ok)
        return false;*/

	taxelPositions[bp][sp] = *positions;
    arePosesUptodate[bp][sp] = true;
    return true;
}

Vector* SkinResourceFinder::getTaxelPosition(BodyPart bp, SkinPart sp, long int taxel_id){
    if(!arePosesUptodate[bp][sp])
        updatePoses(bp, sp);
	return &(taxelPositions[bp][sp][taxel_id]);
}

void SkinResourceFinder::getTaxelPositions(BodyPart bp, SkinPart sp, map<long int, Vector> *positions){
    if(!arePosesUptodate[bp][sp])
        updatePoses(bp, sp);
    positions = &(taxelPositions[bp][sp]);
}

// ***************************************************************************************
// *********************************** PRIVATE MEMBERS ***********************************
// ***************************************************************************************

bool SkinResourceFinder::updatePoses(iCub::skinDynLib::BodyPart bp, iCub::skinDynLib::SkinPart sp){
    Bottle b, reply;
    b.addInt(get_pose);
    b.addInt(bp);
    b.addInt(sp);
    if(!skinManagerRpcPort.write(b, reply))
        return false;
    double x, y, z, alfa, beta, gamma;
    if(reply.size()%6 != 0){
        SkinCalibrationLogger::writeMsg("[SkinResourceFinder] Something went wrong trying to get taxel poses. Vector size not a multiple of 6 (reply=%s)\n", reply.toString().c_str());
        return false;
    }
    for(int i=0; i<reply.size()/6; i++){
        x = reply.get(6*i).asDouble();
        y = reply.get(6*i+1).asDouble();
        z = reply.get(6*i+2).asDouble();
        alfa = reply.get(6*i+3).asDouble();
        beta = reply.get(6*i+4).asDouble();
        gamma = reply.get(6*i+5).asDouble();
        if(!(x==0.0 && y==0.0 && z==0.0)){  // if they are all zeros then the taxel doesn't exist
            taxelPositions[bp][sp][i].resize(3);
            taxelPositions[bp][sp][i][0] = x;
            taxelPositions[bp][sp][i][1] = y;
            taxelPositions[bp][sp][i][2] = z;
            taxelOrientations[bp][sp][i].resize(3);
            taxelOrientations[bp][sp][i][0] = alfa;
            taxelOrientations[bp][sp][i][1] = beta;
            taxelOrientations[bp][sp][i][2] = gamma;
        }
    }
    arePosesUptodate[bp][sp] = true;
    return true;
}

void SkinResourceFinder::addToBottle(Bottle& b, const Vector& v){
    for(unsigned int i=0; i<v.size(); i++)
        b.addDouble(v[i]);
}

void SkinResourceFinder::addToBottle(Bottle& b, const vector<Vector>& v){
    for(unsigned int i=0; i<v.size(); i++)
        for(unsigned int j=0; j<v[i].size(); j++)
            b.addDouble(v[i][j]);
}

void SkinResourceFinder::addPosesToBottle(Bottle& b, const map<long int, Vector>& v){
    for(map<long int, Vector>::const_iterator it=v.begin(); it!=v.end(); it++){
        unsigned int j=0;
        for(; j<it->second.size(); j++) b.addDouble(it->second[j]);
        for(; j<6; j++)                 b.addDouble(0.0);               // fill the missing data with zeros
    }
}

bool SkinResourceFinder::bottleToVector(const yarp::os::Bottle& b, yarp::sig::Vector& v){
    for(int i=0; i<b.size(); i++)
        if(b.get(i).isDouble() || b.get(i).isInt())
            v.push_back(b.get(i).asDouble());
        else
            return false;
    return true;
}
