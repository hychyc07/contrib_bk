#include "iCub/skinCalibration/MeshIntersectionsCA.h"
#include <math.h>

using namespace iCub::skinCalibration;

MeshIntersectionsCA::MeshIntersectionsCA():CalibrationAlgorithm("MeshIntersections"){
	meshBP='\0';
	w.clear();
	counterConfidence.clear();
};

MeshIntersectionsCA::~MeshIntersectionsCA(){
	SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] removing algorithm\n");
	if(initialized){
		if(SaveWeight){
			ofstream file;
			map<long int, double>::iterator it;
			file.open(filefullpath.c_str());
			for(it = w.begin(); it!=w.end();it++){
				SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Writing weight for taxel %i\n",it->first);
				file<<it->first<<" "<<it->second<<endl;
			}
			file.close();
		}
		if(meshBP){
			delete meshBP;
			meshBP='\0';
		}
	}
}

bool MeshIntersectionsCA::parseFile(string filename){
	ifstream file;
	file.open(filename.c_str());
	if(file.peek() == ifstream::traits_type::eof()) //The file is empty
		return true;
	while(!file.eof()){
		long int id;
		file>>id;
		if(file.fail())
			return false;
		file>>w[id];
	}
	return true;
}


bool MeshIntersectionsCA::init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate){
	SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Init\n");
	string filename = "";
	
	this->skinPart = skinPart;
	this->robotBodyPart = robotBodyPart;
	SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Initial estimate size %i.\n",initial_estimate->size());
	yarp::os::Property config;
	filename = "MeshIntersectionsCA.ini";
	filefullpath = rf->findFile(filename.c_str());
	if(filefullpath.empty()){
		SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Config file not found, aborting!\n");
		initialized = false;
		return initialized;
	}

	config.fromConfigFile(filefullpath.c_str());
	if(!config.check("UsePreviousInformation")){
		UsePreviousInformation = false;
	}
	else{
		UsePreviousInformation = (config.find("UsePreviousInformation").asInt() == 1);
	}

	config.fromConfigFile(filefullpath.c_str());
	if(!config.check("UseWeightMatrix")){
		UseWeightMatrix = false;
	}
	else{
		UseWeightMatrix = (config.find("UseWeightMatrix").asInt() == 1);
	}

	config.fromConfigFile(filefullpath.c_str());
	if(!config.check("SaveWeight")){
		SaveWeight = false;
	}
	else{
		SaveWeight = (config.find("SaveWeight").asInt() == 1);
	}
	
	filename = "MeshIntersectionsCA_";
	filename += BodyPart_s[robotBodyPart]+"_"+ SkinPart_s[skinPart];
	filefullpath = rf->findFile(filename.c_str());
	if(filefullpath.empty()){ //if we don't have any previuos information
		SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] previous information not found!\n");
		ofstream fileo;
		filefullpath = rf->getContextPath();
		filefullpath += "/";
		filefullpath += filename;
		if(SaveWeight){
			SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Creating a new file: %s\n",filefullpath.c_str());
			fileo.open(filefullpath.c_str());
		}
		map<long int, Vector>::iterator it;
		for(it = initial_estimate->begin(); it!=initial_estimate->end();it++){
			it->second.zero();
			w[it->first] = 0.0;
			if(SaveWeight)
				fileo<<it->first<<" "<<0.0<<endl;
		}
		if(SaveWeight)
			fileo.close();
	}
	else{
		if(UsePreviousInformation){
			SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Parsing the file %s...\n",filefullpath.c_str());
			if(!parseFile(filefullpath)){ //the file is corrupted
				w.clear();
				SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] ...the file is corrupted!\n");
				ofstream fileo;
				filefullpath = rf->getContextPath();
				filefullpath += "/";
				filefullpath += filename;
				if(SaveWeight){
					SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Creating a new file: %s\n",filefullpath.c_str());
					fileo.open(filefullpath.c_str());
					fileo.close();
				}
				map<long int, Vector>::iterator it;
				for(it = initial_estimate->begin(); it!=initial_estimate->end();it++){
					it->second.zero();
					w[it->first] = 0.0;
				}
			}
			SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] ...done\n");
		}
		else{
			ofstream fileo;
			filefullpath = rf->getContextPath();
			filefullpath += "/";
			filefullpath += filename;
			if(SaveWeight){
				SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Creating a new file: %s\n",filefullpath.c_str());
				fileo.open(filefullpath.c_str());
				fileo.close();
			}
			map<long int, Vector>::iterator it;
			for(it = initial_estimate->begin(); it!=initial_estimate->end();it++){
				it->second.zero();
				w[it->first] = 0.0;
				counterConfidence[it->first] = 0.0;
			}
		}
	}

	if(!config.check("scaleFactor")){
		SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] scaleFactor not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	scale_factor = config.find("scaleFactor").asDouble();

	if(!config.check("maxTaxelPerSample")){
		SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] maxTaxelPerSample not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	maxTaxelPerSample = config.find("maxTaxelPerSample").asInt();


	if(!config.check("estimateCorrection")){
		SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] estimateCorrection not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	estimateCorrection = config.find("estimateCorrection").asDouble();

	if(!config.check("maxWeight")){
		SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] maxWeight not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	maxWeight = config.find("maxWeight").asDouble();

	if(!config.check("weightCorrection")){
		SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] weightCorrection not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	weightCorrection = config.find("weightCorrection").asDouble();

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
				SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] meshToRefFrameTrans angaxis %i %f\n",i-3,temp->get(i).asDouble());
			}
			rotRefFrame = axis2dcm(angaxis,1).submatrix(0,2,0,2);
			for(int i=0; i<3; i++)
				traslRefFrame[i] = temp->get(i).asDouble();
		}
		else{
			SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] meshToRefFrameTrans wrong format, aborting!\n");
			return false;
		}
		SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] meshToRefFrameTrans matrix %f %f %f; %f %f %f; %f %f %f; %f %f %f;\n",rotRefFrame(0,0),rotRefFrame(0,1),rotRefFrame(0,2),rotRefFrame(1,0),rotRefFrame(1,1),rotRefFrame(1,2),rotRefFrame(2,0),rotRefFrame(2,1),rotRefFrame(2,2),traslRefFrame[0],traslRefFrame[1],traslRefFrame[2]);
	}
	else{
		SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] meshToRefFrameTrans not found, aborting!\n");
		initialized = false;
		return initialized;
	}

	if(!config.check("meshFileName")){
		SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] meshFileName not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	meshfilename =  config.find("meshFileName").asString();
	meshfilename =  rf->findFile(meshfilename.c_str());
	if(meshfilename.empty()){
		SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Mesh file not found, aborting!\n");
		initialized = false;
		return initialized;
	}
	SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Parsing the mesh file...\n");
	meshBP = new MeshBodyPart(meshfilename.c_str(),scale_factor,rotRefFrame,traslRefFrame);
	if(!meshBP->isInitialized()){
		SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] ...mesh file corrupted, aborting!\n");
		initialized = false;
		return initialized;
	}
	SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] ...done!\n");
	w.clear();
	SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Init end\n");
	initialized = true;
	return initialized;
}


bool MeshIntersectionsCA::calibrate(CalibrationBuffer *cb, map<long int, Vector> *estimates){
	long int id = cb->getTaxelID();
	vector<CalibrationSample *>::const_iterator it;
	bool result = false;
	if((*estimates)[id].size() == 0){
		(*estimates)[id].resize(4);
		(*estimates)[id].zero();
	}
	for(it = cb->getSamples()->begin(); it!=cb->getSamples()->end(); it++){
		Vector point;
		//int prjface;
		double fnorm,wnew,tmeasure,tmaxmeasure;
		unsigned int highertaxels=0;
		double tmes = (*it)->taxels_measures->operator[](id);
		long int tmax;
		Vector est = (*estimates)[id].subVector(0,2);
		map<long int, double>::iterator tm;
		//We want to find the taxel with the highest response and to check if the current taxel is among the maxTaxelPerSample taxels with higher responses.
		for(tm = (*it)->taxels_measures->begin(); tm != (*it)->taxels_measures->end() && highertaxels<maxTaxelPerSample; tm++){
			if(tmaxmeasure < (*tm).second){
				tmaxmeasure = (*tm).second;
				tmax = (*tm).first;
			}
			if((*tm).second>tmes) highertaxels++;
		}
		// If the taxel is not among the maxTaxelPerSample taxels with higher responses, skip the sample
		if(highertaxels>=maxTaxelPerSample)
			continue;

		//We need the sensor-to-wrist transformation that is the inverse of the one provided by the calibrationSample
		Vector o = cross(*(*it)->force,*(*it)->moment) / pow(norm(*(*it)->force),2);
		o = (*(*it)->Rws).transposed()*o - (*(*it)->Rws).transposed()*(*(*it)->ows);
		Vector dir=  (*(*it)->Rws).transposed()*(*(*it)->force);
		meshBP->findRayMeshIntersectionPoint(o,dir,point);
		//meshBP->findClosestProjectionOnMesh(o,dir,prjface,point);
		if(point.size()!=0){
		//	printf("MeshIntersectionsCA Intersection point %f %f %f\n",point[0],point[1],point[2]);
		//	printf("MeshIntersectionsCA taxel measure %f\n",(*(*it)->taxels_measures)[id]);
			fnorm=norm((*(*it)->force));
			tmeasure = (*(*it)->taxels_measures)[(*it)->taxel_id];
			//wnew= (tmeasure/244)/fnorm;
			wnew = 1.0;
			tmaxmeasure=0.0;
			//SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Old estimate %li %f %f %f\n",id,est[0],est[1],est[2]);
			if((*estimates)[id][0] == 0.0 && (*estimates)[id][1] == 0.0 && (*estimates)[id][2] == 0.0){
				est= point;
				est.push_back(0.0);
				(*estimates)[id] = est;
			}
			else{
				//Updating weighted mean
				int prj_face;
				//if(norm(est-point)<0.03 && w[id]<maxWeight){
					est=((w[id]/(w[id]+wnew))*est + ((wnew/(w[id]+wnew))*point));
					if((*estimates)[tmax].size() != 0 && tmax!=id && estimateCorrection != 0.0){
						double dist = norm(est-(*estimates)[tmax].subVector(0,2));
						if(dist != 0){
							double corr = dist >= estimateCorrection? 0.0: estimateCorrection-dist;
							est+= corr* ((est-(*estimates)[tmax].subVector(0,2))/dist);
						}
					}
				/*}
				else{
					//SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Re-weighting %f\n",w[id]);
					est=((w[id]/((w[id]/weightCorrection)+wnew))*(est/weightCorrection) + ((wnew/((w[id]/weightCorrection)+wnew))*point));
					w[id] /=weightCorrection;
					//SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] New-weight %f\n",w[id]);
					if((*estimates)[tmax].size() != 0 && tmax!=id && estimateCorrection != 0.0){
						double dist = norm(est-(*estimates)[tmax].subVector(0,2));
						if(dist != 0){
							double corr = dist >= estimateCorrection ? 0.0: estimateCorrection-dist;
								est+= corr* ((est-(*estimates)[tmax].subVector(0,2))/dist);
						}
					}
				}*/
				//meshBP->findRayMeshIntersectionPoint(est,dir,point);
				meshBP->findClosestProjectionOnMesh(est,dir,prj_face,point);
				if(point.size() != 0)
					est = point;
				//SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] New estimate %li %f %f %f\n",id,est[0],est[1],est[2]);
				w[id]+=wnew;
				counterConfidence[id]=counterConfidence[id]>=maxNumAxes? maxNumAxes : counterConfidence[id]+1;
				//SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Estimate confidence of taxel %i is %f\n",id,counterConfidence[id]/maxNumAxes);
				est.push_back(counterConfidence[id]/maxNumAxes);
				(*estimates)[id] = est;
			}
			result = true;
		}
		else{
			SkinCalibrationLogger::writeMsg("[MeshIntersectionsCA] Intersection not found (taxel id %li)\n",id);
		}
	}
	return result;
}