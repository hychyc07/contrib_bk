#include "iCub/skinCalibration/ModuleFittingCA.h"
#include <math.h>

using namespace iCub::skinCalibration;

ModuleFittingCA::ModuleFittingCA():  smc("ModuleFittingCA"), CalibrationAlgorithm("ModuleFittingCA"){
	SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Constructor\n");
	meshBP='\0';
	sub_algorithm='\0';
};

ModuleFittingCA::~ModuleFittingCA(){
	SkinCalibrationLogger::writeMsg("[ModuleFittingCA] removing algorithm\n");
	if(meshBP){
		delete meshBP;
		meshBP='\0';
	}
	//We don't have to deallocate the algorithm;
	sub_algorithm ='\0';
}

void ModuleFittingCA::computeModuleRefTaxel(){
	
	queue<int> qid;
	vector<int> tnotplaced;
	vector<int>::iterator it;
	int currentid;
	double neighbordistance = 0.007;
	vector<Matrix>	moduleStructureExt = moduleStructure;
	Matrix Eye(4,4);
	Eye.eye();
	moduleStructureExt.insert(moduleStructureExt.begin(),Eye);
	for(unsigned int i=0; i<moduleStructure.size();i++){
		tnotplaced.push_back(i+1);
	}
	qid.push(0);
	moduleRefTaxel.resize(taxelPerModule);
	while(qid.size() != 0){
		currentid = qid.front();
		qid.pop();
		for(it = tnotplaced.begin(); it !=tnotplaced.end();){
			//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Evaluating taxel %i %f\n",*it,norm(moduleStructureExt[*it].subcol(0,3,3)-moduleStructureExt[currentid].subcol(0,3,3)));
			if(norm(moduleStructureExt[*it].subcol(0,3,3)-moduleStructureExt[currentid].subcol(0,3,3)) < neighbordistance){
				modulePlacingSequence.push_back(*it-1);
				moduleRefTaxel[*it-1] = currentid-1;
				SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Placing taxel %i w.r.t. %i\n",*it-1,currentid-1);
				qid.push(*it);
				it = tnotplaced.erase(it);
			}
			else{
				it++;
			}
		}	
	}
}

bool ModuleFittingCA::parseModuleStructureFile(string filename){
	ifstream file;

	file.open(filename.c_str());
	if(file.peek() == ifstream::traits_type::eof()) //The file is empty
		return false;
	
	for(int i=0; i<3; i++){
		moduleVertexIndex[i].resize(3);
		file>>moduleVertexIndex[i][0]>>moduleVertexIndex[i][1]>>moduleVertexIndex[i][2];
	}
	taxelPerModule = 0;
	while(!file.eof()){
		Matrix m(4,4);
		
		for(int i=0;i<16;i++){
			file>>m(i/4,i-((i/4)*4));
			if(file.fail() && i == 0)
				return true;
		}

		moduleStructure.push_back(m);
		taxelPerModule++;
		if(file.fail())
			return false;
	}
	return true;
}

void ModuleFittingCA::findNeighbors(long startid, Vector centroid, map<long int, Vector> *estimates, vector<unsigned int> *neighborsStartId){
	
	neighborsStartId->clear();
	for(unsigned int i=0; i< (*estimates).size(); i+=taxelPerModule){
		if(i == startid)
			continue;
		for(unsigned int j=i; j< i+taxelPerModule; j++){
			if(norm(centroid - (*estimates)[j].subVector(0,2)) < neighborsMinDist){
				neighborsStartId->push_back(i);
				break;
			}
		}
	}
}

bool ModuleFittingCA::fitModuleToEstimate(long startid, map<long int, Vector> *estimates, vector<Vector> *newEstimates){

	int counta=0,countb=0,countg=0;
	int cprjface;
	long endid;
	Vector centroid(3,0.0);
	Vector centroid_prj(3,0.0);
	//Vector centroid_prj2(3,0.0);
	Vector alphadir(3,0.0);
	Vector betadir(3,0.0);
	Vector gammadir(3,0.0);
	Vector neighborsCorr(3,0.0);
	double alpha,beta,gamma = 0.0;
	Matrix T(4,4);
	vector<Matrix>	placedTaxels(taxelPerModule);
	Vector prjdir(3,0.0);
	Vector temp;
	Matrix currentFrame;
	vector<CalibrationSample *>::const_iterator it;
	vector<unsigned int> neighborsStartId;
	//startid = (cb->getTaxelID()/taxelPerModule)*taxelPerModule;
	endid = startid+taxelPerModule-1;
		
		
	for(long i=startid; i<=endid; i++){
		if(norm((*estimates)[i].subVector(0,2)) != 0.0){
			centroid += (*estimates)[i].subVector(0,2);
			counta++;
		}
	}
	if(counta == 0){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] The module cannot be fitted with the current estimate!\n");
		return false;
	}
	centroid /= counta;
	
	//Move the centroid according to the centroids of the surrounding modules
	findNeighbors(startid,centroid,estimates,&neighborsStartId);
	for(unsigned int i=0; i<neighborsStartId.size(); i++){
		Vector temp(3,0.0);
		int countt=0;
		double distance;
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] \tNeighbor %i\n",neighborsStartId[i]);
		for(unsigned int j=neighborsStartId[i]; j<neighborsStartId[i]+taxelPerModule; j++){
			if(norm((*estimates)[j].subVector(0,2)) != 0.0){
				temp += (*estimates)[j].subVector(0,2);
				countt++;
			}	
		}
		if(countt == 0)
			continue;
		temp /= countt;
		distance = norm(temp-centroid);
		neighborsCorr =  ((moduleCentroidDistance-distance)/2)*((temp-centroid)/distance);
	}
	
	centroid -= neighborsCorr;

	counta = 0;
	for(int i=0; i<3; i++){
		if(norm((*estimates)[startid+moduleVertexIndex[0][i]].subVector(0,2)) != 0.0){
			alphadir += (*estimates)[startid+moduleVertexIndex[0][i]].subVector(0,2);
			counta++;
		}
		if(norm((*estimates)[startid+moduleVertexIndex[1][i]].subVector(0,2)) != 0.0){
			betadir += (*estimates)[startid+moduleVertexIndex[1][i]].subVector(0,2);
			countb++;
		}
		if(norm((*estimates)[startid+moduleVertexIndex[2][i]].subVector(0,2)) != 0.0){
			gammadir += (*estimates)[startid+moduleVertexIndex[2][i]].subVector(0,2);
			countg++;
		}
	}
	if(counta == 0 || countb == 0 || countg ==0){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] The module cannot be fitted with the current estimate!\n");
		return false;
	}
	//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Centroid is %f %f %f\n",centroid[0],centroid[1],centroid[2]);
	alphadir /= counta;
	betadir  /= countb;
	gammadir /= countg;

	//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Alpha centroid is %f %f %f\n",alphadir[0],alphadir[1],alphadir[2]);
	//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Beta centroid is %f %f %f\n",betadir[0],betadir[1],betadir[2]);
	//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Gamma centroid is %f %f %f\n",gammadir[0],gammadir[1],gammadir[2]);
	alphadir -= neighborsCorr;
	betadir -= neighborsCorr;
	gammadir -= neighborsCorr;

	alphadir -= centroid;
	alphadir /= norm(alphadir);
	betadir -= centroid;
	betadir /= norm(betadir);
	gammadir -= centroid;
	gammadir /= norm(gammadir);
	
	prjdir = cross(alphadir,betadir);
	temp = cross(betadir,gammadir);
	if(dot(prjdir,temp)> 0)
		prjdir += temp;
	else
		prjdir -= temp;
	temp = cross(gammadir,alphadir);
	if(dot(prjdir,temp)> 0)
		prjdir += temp;
	else
		prjdir -= temp;
	prjdir /= norm(prjdir);
	
	//Make the directions normal to prjdir;
	alphadir = cross(prjdir,cross(alphadir,prjdir));
	alphadir /= norm(alphadir);
	betadir = cross(prjdir,cross(betadir,prjdir));
	betadir /= norm(betadir);
	gammadir = cross(prjdir,cross(gammadir,prjdir));
	gammadir /= norm(gammadir);
	

	beta = (2*M_PI)/3 -acos(dot(alphadir,betadir));
	gamma = (2*M_PI)/3 - acos(dot(alphadir,gammadir));
	
	if(beta > 0 && gamma > 0){
		if(gamma < beta)
			alpha = gamma/2;
		else if(beta < gamma)
			alpha = beta/2;
	}
	else if(beta < 0 && gamma < 0){
		if(gamma > beta)
			alpha = -gamma/2;
		else if(beta > gamma)
			alpha = -beta/2;
	}
	else if(beta >= 0 && gamma <= 0){
		if(beta > -gamma)
			alpha = (beta+gamma)/2;
		else if(beta < -gamma)
			alpha = (-gamma -beta)/2;
	}
	else if(beta <= 0 && gamma >= 0){
		if(-beta > gamma)
			alpha = -(-beta-gamma)/2;
		else if(-beta < gamma)
			alpha = -(gamma+beta)/2;
	}
	//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Alpha is %f\n",alpha);
	//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Beta is %f\n",beta);
	//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Gamma is %f\n",gamma);
	//Rotate the alphadir about alpha around prjdir (Rodrigues' rotation formula)
	alphadir = alphadir*cos(alpha) + cross(prjdir,alphadir)*sin(alpha) + prjdir*dot(prjdir,alphadir)*(1-cos(alpha));

	//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Alpha dir is %f %f %f\n",alphadir[0],alphadir[1],alphadir[2]);
	//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Beta dir is %f %f %f\n",betadir[0],betadir[1],betadir[2]);
	//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Gamma dir is %f %f %f\n",gammadir[0],gammadir[1],gammadir[2]);

	meshBP->findClosestProjectionOnMesh(centroid,prjdir,cprjface,centroid_prj);
	//meshBP->findRayMeshIntersectionPoint(centroid,prjdir,centroid_prj1);
	//meshBP->findRayMeshIntersectionPoint(centroid,-1*prjdir,centroid_prj2);
	if(centroid_prj.size() != 0){
		centroid = centroid_prj;
	}
	//else if(centroid_prj1.size() == 0 && centroid_prj2.size() != 0){
	//	centroid = centroid_prj2;
	//}
	else{
		//if(norm(centroid-centroid_prj1) < norm(centroid-centroid_prj2)){
		//	centroid = centroid_prj1;
		//}
		//else{
		//	centroid = centroid_prj2;
		//}
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Cannot project the centroid on mesh!\n");
		return false;
	}
	if(dot(prjdir,meshBP->getFaceNormal(cprjface))<0)
		prjdir = -1*prjdir;

	//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Projected centroid is %f %f %f on face %i\n",centroid[0],centroid[1],centroid[2],cprjface);
		
	T.eye();
	T.setSubcol(cross(alphadir,prjdir),0,0);
	T.setSubcol(alphadir,0,1);
	T.setSubcol(prjdir,0,2);
	T.setSubcol(centroid,0,3);
	placedTaxels.insert(placedTaxels.begin(),T);

	//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] T matrix\n");
	//for(int i = 0; i<4; i++)
	//	SkinCalibrationLogger::writeMsg("[ModuleFittingCA] %f %f %f %f\n",T(i,0),T(i,1),T(i,2),T(i,3));

	for(unsigned int i=0; i<modulePlacingSequence.size(); i++){
		
		int taxelid = modulePlacingSequence[i];
		int currentid = moduleRefTaxel[taxelid];
		int prjface;
		Vector prj;
		Matrix TaxelToBePlaced;
		Matrix TON;
		if(placedTaxels[currentid+1].rows()!=0)
			currentFrame = placedTaxels[currentid+1];
		else
			continue;
		//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Placing Taxel %i\n",taxelid);
		if(currentid >= 0)
			TON = SE3inv(moduleStructure[currentid])*moduleStructure[taxelid];
		else
			TON = moduleStructure[taxelid];
        TaxelToBePlaced = currentFrame*TON;
		//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Current Frame\n");
		//for(int k = 0; k<4; k++)
		//	SkinCalibrationLogger::writeMsg("[ModuleFittingCA] %f %f %f %f\n",currentFrame(k,0),currentFrame(k,1),currentFrame(k,2),currentFrame(k,3));
		Vector TTBPV = TaxelToBePlaced.subcol(0,3,3);
		Vector currentFrameV = currentFrame.subcol(0,2,3);
		meshBP->findClosestProjectionOnMesh(TTBPV,currentFrameV,prjface,prj);
		if(prj.size() != 0){
			//SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Projected taxel %i is %f %f %f on face %i\n",taxelid,prj[0],prj[1],prj[2],prjface);
			Vector tempn = meshBP->getFaceNormal(prjface);
			Vector tempv;
			placedTaxels[taxelid+1].resize(4,4);
			placedTaxels[taxelid+1].eye();
			placedTaxels[taxelid+1].setSubcol(tempn,0,2);
			tempv = cross(TaxelToBePlaced.subcol(0,1,3),tempn);
			tempv=tempv/norm(tempv);
			placedTaxels[taxelid+1].setSubcol(tempv,0,0);
			tempv = cross(tempn,tempv);
			placedTaxels[taxelid+1].setSubcol(tempv,0,1);
			placedTaxels[taxelid+1].setSubcol(prj,0,3);
		}
		else{
			SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Taxel %i cannot be placed!\n",taxelid);
			placedTaxels[taxelid+1].resize(0,0);
		}
	}
	for(unsigned int i=1; i<placedTaxels.size(); i++){
		if(placedTaxels[i].rows()!=0){
			//(*estimates)[startid+i-1].setSubvector(0,placedTaxels[i].getCol(3).subVector(0,2));
			(*newEstimates)[i-1].resize(4);
			(*newEstimates)[i-1].setSubvector(0,placedTaxels[i].getCol(3).subVector(0,2));
			if((*estimates)[startid+i-1].size() == 4)
				(*newEstimates)[i-1][3] = (*estimates)[startid+i-1][3];
			else
				(*newEstimates)[i-1][3] = 0.0;
		}
	}
	return true;
}

bool ModuleFittingCA::init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate){
	string filename = "";
	string modulefilename = "";
	int nalgorithms=0;
	int nsubalgorithm=0;
	SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Init\n");

	this->skinPart = skinPart;
	this->robotBodyPart = robotBodyPart;

	yarp::os::Property config;
	filename = "ModuleFittingCA.ini";
	filefullpath = rf->findFile(filename.c_str());
	if(filefullpath.empty()){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Config file not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	config.fromConfigFile(filefullpath.c_str());

	if(!config.check("moduleStructureFile")){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] moduleStructureFile not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	modulefilename = config.find("moduleStructureFile").asString();
	modulefilename = rf->findFile(modulefilename.c_str());
	if(!parseModuleStructureFile(modulefilename)){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] module structure file cannot be parsed, aborting!\n");
		initialized = false;
		return initialized;
	}
	taxelPerModule = moduleStructure.size();
	computeModuleRefTaxel();

	if(!config.check("subAlgorithm")){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] subAlgorithm not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	while(CalibrationEngine::algorithms[nalgorithms]!='\0')nalgorithms++;
	nsubalgorithm = config.find("subAlgorithm").asInt();
	if(nsubalgorithm<0 || nsubalgorithm>nalgorithms-1){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] invalid sub algorithm number, aborting!\n");
		initialized = false;
		return initialized;
	}
	sub_algorithm = CalibrationEngine::algorithms[nsubalgorithm];
	if(!sub_algorithm->init(rf,robotBodyPart,skinPart,initial_estimate)){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] cannot initialize the sub algorithm, aborting!\n");
		initialized = false;
		return initialized;
	}

	if(!config.check("scaleFactor")){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] scaleFactor not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	scale_factor = config.find("scaleFactor").asDouble();


	if(!config.check("neighborsMinDist")){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] neighborsMinDist not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	neighborsMinDist = config.find("neighborsMinDist").asDouble();

	if(!config.check("moduleCentroidDistance")){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] moduleCentroidDistance not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	moduleCentroidDistance = config.find("moduleCentroidDistance").asDouble();

	rotRefFrame.resize(3,3);
	rotRefFrame.zero();
	traslRefFrame.resize(3);
	traslRefFrame.zero();
	if (config.check("meshToRefFrameTrans")){
		Vector angaxis(4);
		angaxis.zero();
		Bottle *temp;
		temp = config.find("meshToRefFrameTrans").asList();
		if(temp && temp->size() == 7){
			for(int i=3; i<7; i++){
				angaxis[i-3] = (temp->get(i).asDouble());
				SkinCalibrationLogger::writeMsg("[ModuleFittingCA] meshToRefFrameTrans angaxis %i %f\n",i-3,temp->get(i).asDouble());
			}
			rotRefFrame = axis2dcm(angaxis,1).submatrix(0,2,0,2);
			for(int i=0; i<3; i++)
				traslRefFrame[i] = temp->get(i).asDouble();
		}
		else{
			SkinCalibrationLogger::writeMsg("[ModuleFittingCA] meshToRefFrameTrans wrong format, aborting!\n");
			return false;
		}
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] meshToRefFrameTrans matrix %f %f %f; %f %f %f; %f %f %f; %f %f %f;\n",rotRefFrame(0,0),rotRefFrame(0,1),rotRefFrame(0,2),rotRefFrame(1,0),rotRefFrame(1,1),rotRefFrame(1,2),rotRefFrame(2,0),rotRefFrame(2,1),rotRefFrame(2,2),traslRefFrame[0],traslRefFrame[1],traslRefFrame[2]);
	}
	else{
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] meshToRefFrameTrans not found, aborting!\n");
		initialized = false;
		return initialized;
	}

	if(!config.check("meshFileName")){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] meshFileName not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	meshfilename =  config.find("meshFileName").asString();
	meshfilename =  rf->findFile(meshfilename.c_str());
	if(meshfilename.empty()){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Mesh file not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Parsing the mesh file...\n");
	meshBP = new MeshBodyPart(meshfilename.c_str(),scale_factor,rotRefFrame,traslRefFrame);
	if(!meshBP->isInitialized()){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] ...mesh file corrupted, aborting!\n");
		initialized = false;
		return initialized;
	}
	if(!smc.init()){
        SkinCalibrationLogger::writeMsg("[ModuleFittingCA] ...SkinManagerClient initialization failed.\n");
        return false;
    }

	SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Starting the module fitting with the current estimation...\n");
	SkinCalibrationLogger::writeMsg("[ModuleFittingCA] Estimates size %i\n", (*initial_estimate).size());
	for(unsigned int i=0; i< (*initial_estimate).size(); i+=taxelPerModule){
	//for(unsigned int i=0; i< 2*taxelPerModule; i+=taxelPerModule){
		vector<Vector> newEstimates(taxelPerModule);
		if(!fitModuleToEstimate(i,initial_estimate,&newEstimates)){
			SkinCalibrationLogger::writeMsg("[ModuleFittingCA] module fitting was not successful for taxels %i %i.\n",i,i+taxelPerModule-1);
		}
		else{
			for(unsigned int j=0; j <taxelPerModule; j++)
				smc.setTaxelPosition(skinPart, i+j,newEstimates[j]);
			SkinCalibrationLogger::writeMsg("[ModuleFittingCA] module successfully fitted for taxels %i %i.\n",i,i+taxelPerModule-1);
		}
	}
	SkinCalibrationLogger::writeMsg("[ModuleFittingCA] ...Done.\n");

	initialized = true;
	return initialized;
}

bool ModuleFittingCA::calibrate(CalibrationBuffer *cb,map<long int, Vector> *estimates){

	//The sub algorithm has updated a taxel position estimate
	if(sub_algorithm->calibrate(cb,estimates)){
		SkinCalibrationLogger::writeMsg("[ModuleFittingCA] New estimate available, starting the module fitting.\n");
		long startid;
		startid = (cb->getTaxelID()/taxelPerModule)*taxelPerModule;
		vector<Vector> newEstimates(taxelPerModule);
		if(!fitModuleToEstimate(startid,estimates,&newEstimates)){
			SkinCalibrationLogger::writeMsg("[ModuleFittingCA] module fitting was not successful.\n");
		}
		else{
			for(unsigned int i=0; i <taxelPerModule; i++)
				//smc.setTaxelPosition(skinPart, startid+i,(*estimates)[startid+i]);
				smc.setTaxelPosition(skinPart, startid+i,newEstimates[i]);
		}
	}
	return false;
}