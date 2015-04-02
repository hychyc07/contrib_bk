#include "iCub/skinCalibration/LeastSquareCA.h"
#include <math.h>

using namespace iCub::skinCalibration;

LeastSquareCA::~LeastSquareCA(){
	SkinCalibrationLogger::writeMsg("[LeastSquareCA] removing algorithm\n");
	if(initialized){
		ofstream file;
		map<long int, Matrix *>::iterator it;
		file.open(filefullpath.c_str());
		for(it = H.begin(); it!=H.end();){
			SkinCalibrationLogger::writeMsg("[LeastSquareCA] Writing H matrix for taxel %i\n",it->first);
			if(it->second!='\0'){
				file<<it->first;
				for(int i=0; i<9; i++){
					file<<" "<<(*it->second)(i/3,i-(3*(i/3)));
				}
				delete it->second;
				it++;
				if(it!=H.end())
					file<<endl;
			}
		}
		file.close();
	}
}

bool LeastSquareCA::parseFile(string filename){
	ifstream file;
	file.open(filename.c_str());
	if(file.peek() == ifstream::traits_type::eof()) //The file is empty
		return true;
	while(!file.eof()){
		long int id;
		file>>id;
		if(file.fail())
			return false;
		H[id] = new Matrix(3,3);
		for(int i =0;i <9; i++){
			file>>(*H[id])(i/3,i-(3*(i/3)));
			if(file.fail())
				return false;
		}
	}
	return true;
}

void LeastSquareCA::skewSymmetric(Vector *v,Matrix *M){
	M->resize(3,3);
	M->zero();
	(*M)(0,1) = -(*v)[2];
	(*M)(0,2) = (*v)[1];
	(*M)(1,0) = (*v)[2];
	(*M)(1,2) = -(*v)[0];
	(*M)(2,0) = -(*v)[1];
	(*M)(2,1) = (*v)[0];
}

Matrix LeastSquareCA::computeWeightMatrix(CalibrationSample *cs){
	Matrix K(3,3);
	Vector diag(3);
	double fnorm = norm(*cs->force);
	map<long int, double>::const_iterator it;
	double max = 0;
	for(it = cs->taxels_measures->begin(); it!=cs->taxels_measures->end(); it++){
		if(it->second>max)
			max = it->second;
	}
	diag[0]=sqrt((*cs->taxels_measures)[cs->taxel_id]/max);
	diag[1]=diag[0];
	diag[2]=diag[0];
	return K.diagonal(diag);
}

bool LeastSquareCA::init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate){
	SkinCalibrationLogger::writeMsg("[LeastSquareCA] Init\n");
	string filename = "";
	
	this->skinPart = skinPart;
	this->robotBodyPart = robotBodyPart;
	
	yarp::os::Property config;
	filename = "LeastSquareCA.ini";
	filefullpath = rf->findFile(filename.c_str());
	if(filefullpath.empty()){
		SkinCalibrationLogger::writeMsg("[LeastSquareCA] Config file not found, aborting!\n");
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

	
		filename = "LeastSquareCA_";
		filename += BodyPart_s[robotBodyPart]+"_"+ SkinPart_s[skinPart];
		filefullpath = rf->findFile(filename.c_str());
		if(filefullpath.empty()){ //if we don't have any previuos information
			SkinCalibrationLogger::writeMsg("[LeastSquareCA] Previous information not found!\n");
			ofstream fileo;
			filefullpath = rf->getContextPath();
			filefullpath += "/";
			filefullpath += filename;
			SkinCalibrationLogger::writeMsg("[LeastSquareCA] Creating a new file: %s\n",filefullpath.c_str());
			fileo.open(filefullpath.c_str());
			map<long int, Vector>::iterator it;
			for(it = initial_estimate->begin(); it!=initial_estimate->end();it++){
				it->second.zero(); // can the old estimate be updated? to be investigated.
				H[it->first] = new Matrix(3,3);
				H[it->first]->eye();
				fileo<<it->first<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<endl;
			}
			fileo.close();
		}
		else{
			if(UsePreviousInformation){
				SkinCalibrationLogger::writeMsg("[LeastSquareCA] parsing the file %s...\n",filefullpath.c_str());
				if(!parseFile(filefullpath)){ //the file is corrupted
					H.clear();
					SkinCalibrationLogger::writeMsg("[LeastSquareCA] ...the file is corrupted!\n");
					ofstream fileo;
					filefullpath = rf->getContextPath();
					filefullpath += "/";
					filefullpath += filename;
					SkinCalibrationLogger::writeMsg("[LeastSquareCA] Creating a new file: %s\n",filefullpath.c_str());
					fileo.open(filefullpath.c_str());
					map<long int, Vector>::iterator it;
					for(it = initial_estimate->begin(); it!=initial_estimate->end();it++){
						it->second.zero();
						H[it->first] = new Matrix(3,3);
						H[it->first]->eye();
					}
				}
				SkinCalibrationLogger::writeMsg("[LeastSquareCA] ...done!\n");
			}
			else{
				ofstream fileo;
				filefullpath = rf->getContextPath();
				filefullpath += "/";
				filefullpath += filename;
				SkinCalibrationLogger::writeMsg("[LeastSquareCA]Creating a new file: %s\n",filefullpath.c_str());
				fileo.open(filefullpath.c_str());
				map<long int, Vector>::iterator it;
				for(it = initial_estimate->begin(); it!=initial_estimate->end();it++){
					it->second.zero();
					H[it->first] = new Matrix(3,3);
					H[it->first]->eye();
				}
			}
		}
	
	SkinCalibrationLogger::writeMsg("[LeastSquareCA] Init end\n");
	initialized = true;
	return initialized;
}

bool LeastSquareCA::calibrate(CalibrationBuffer *cb, map<long int, Vector> *estimates){
	long int id = cb->getTaxelID();
	int counter=0;
	vector<CalibrationSample *>::const_iterator it;
	Matrix Hn(3,3);
	Matrix A(cb->getSize()*3,3);
	Vector b(cb->getSize()*3);
	SkinCalibrationLogger::writeMsg("[LeastSquareCA] Starting calibration step for taxel %i\n",id);
	if(H[id]=='\0'){
		H[id] = new Matrix(3,3);
		H[id]->zero();
		(*estimates)[id].zero();
	}
	if((*estimates)[id].size() == 0){
		(*estimates)[id].resize(3);
		(*estimates)[id].zero();
	}

	for(it = cb->getSamples()->begin(); it!=cb->getSamples()->end(); it++){
		//fprintf(stderr,"LeastSquareCA sample for taxel %li: %i %f %f\n",cb->getTaxelID(),(*it)->taxels_measures->size(),norm((*(*it)->force)),abs(dot((*(*it)->force),(*(*it)->moment))));
		Matrix ForceDotP;
		Vector Force = *(*it)->force;

		skewSymmetric(&Force,&ForceDotP);
		Vector Moment(*(*it)->moment);
		Matrix R(*(*it)->Rws);
		//printf("R matrix %f %f %f; %f %f %f; %f %f %f;\n",R(0,0),R(0,1),R(0,2),R(1,0),R(1,1),R(1,2),R(2,0),R(2,1),R(2,2));
		Vector o(*(*it)->ows);
		//printf("ovecotr %f %f %f;\n",o[0],o[1],o[2]);
		
		Moment = (Moment)+(ForceDotP*o);
		Moment = Moment/(-norm(Force));
		ForceDotP = ForceDotP*R;
		ForceDotP = ForceDotP /norm(Force);
		if(UseWeightMatrix)
			ForceDotP= computeWeightMatrix(*it)*ForceDotP;
		A.setSubmatrix(ForceDotP,counter,0);
		
		if(UseWeightMatrix)
			Moment= computeWeightMatrix(*it)*Moment;
		b.setSubvector(counter,Moment);
		counter += 3;
	}

	//Hn = H + An'An
	SkinCalibrationLogger::writeMsg("[LeastSquareCA] Computing H matrix for taxel %i\n",id);
	Hn = (*(H[id]))+ (A.transposed() *A);
	//pn = p + inv(Hn)An'(b - An p);
	SkinCalibrationLogger::writeMsg("[LeastSquareCA] Computing position estimate for taxel %i\n",id);
	//printf("estimates size %i\n",(*estimates)[id].size());
	//printf("A size %i %i\n",A.rows(),A.cols());
	SkinCalibrationLogger::writeMsg("[LeastSquareCA] H matrix %f %f %f; %f %f %f; %f %f %f;\n",((*H[id]))(0,0),(*(H[id]))(0,1),(*(H[id]))(0,2),(*H[id])(1,0),(*H[id])(1,1),(*H[id])(1,2),(*H[id])(2,0),(*H[id])(2,1),(*H[id])(2,2));
	SkinCalibrationLogger::writeMsg("[LeastSquareCA] Previous Estimate %f %f %f\n",(*estimates)[id][0],(*estimates)[id][1],(*estimates)[id][2]);
	(*estimates)[id] = (*estimates)[id] + pinv(Hn)*A.transposed()*(b - (A*(*estimates)[id]));
	SkinCalibrationLogger::writeMsg("[LeastSquareCA] Estimate %f %f %f\n",(*estimates)[id][0],(*estimates)[id][1],(*estimates)[id][2]);
	(*(H[id])) = Hn;
	return true;
}
