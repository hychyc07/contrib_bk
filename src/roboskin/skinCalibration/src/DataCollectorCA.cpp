#include "iCub/skinCalibration/DataCollectorCA.h"


using namespace iCub::skinCalibration;


DataCollectorCA::~DataCollectorCA(){
	SkinCalibrationLogger::writeMsg("[DataCollectorCA] Deleting algorithm\n");
	if(initialized){
		ofstream file;
		vector<long int>::iterator taxel;
		if(SaveData){
			for(taxel = taxel_id.begin(); taxel != taxel_id.end(); taxel++){
				stringstream filename;
				filename << filefullpath<<"DataCollectorCA_";
				filename << BodyPart_s[robotBodyPart]<<"_"<<SkinPart_s[skinPart]<<"_"<<*taxel;

				vector<Vector>::iterator it;
				file.open((filename.str()+"_forces.txt").c_str());
				for(it = Forces[*taxel].begin(); it!=Forces[*taxel].end();){
						file<<(*it)[0]<<" "<<(*it)[1]<<" "<<(*it)[2];
						it++;
						if(it!=Forces[*taxel].end())
							file<<endl;
				}
				file.close();
				file.open((filename.str()+"_moments.txt").c_str());
				for(it = Moments[*taxel].begin(); it!=Moments[*taxel].end();){
						file<<(*it)[0]<<" "<<(*it)[1]<<" "<<(*it)[2];
						it++;
						if(it!=Moments[*taxel].end())
							file<<endl;
				}
				file.close();
				file.open((filename.str()+"_T.txt").c_str());
				for(it = T[*taxel].begin(); it!=T[*taxel].end();){
						file<<(*it)[0]<<" "<<(*it)[1]<<" "<<(*it)[2]<<" ";
						file<<(*it)[3]<<" "<<(*it)[4]<<" "<<(*it)[5]<<" ";
						file<<(*it)[6]<<" "<<(*it)[7]<<" "<<(*it)[8]<<" ";
						file<<(*it)[9]<<" "<<(*it)[10]<<" "<<(*it)[11];
						it++;
						if(it!=T[*taxel].end())
							file<<endl;
				}
				file.close();
			}
		}
		dataPort.interrupt();
		dataPort.close();
	}

}
bool DataCollectorCA::init(ResourceFinder *rf, BodyPart robotBodyPart, SkinPart skinPart, map<long int, Vector> *initial_estimate){
	SkinCalibrationLogger::writeMsg("[DataCollectorCA] Init\n");
	yarp::os::Property config;
	string filename = "";
	filename = "DataCollectorCA.ini";
	filefullpath = rf->findFile(filename.c_str());
	if(filefullpath.empty()){
		SkinCalibrationLogger::writeMsg("[DataCollectoreCA] Config file not found, aborting!\n");
		initialized = false;
		return initialized;
	}

	config.fromConfigFile(filefullpath.c_str());
	if(!config.check("taxelid")){
		SkinCalibrationLogger::writeMsg("[DataCollectoreCA] taxelid not found, aborting!\n");
		initialized = false;
		return initialized;
	}

	Bottle * taxel_id_list = config.find("taxelid").asList();
	if(taxel_id_list){
		for(int i=0; i<taxel_id_list->size(); i++){
			if(taxel_id_list->get(i).isInt()){
				taxel_id.push_back(taxel_id_list->get(i).asInt());
				SkinCalibrationLogger::writeMsg("[DataCollectoreCA] Added id %i.\n",taxel_id_list->get(i).asInt());
			}
			else if(taxel_id_list->get(i).isString()){
				SkinCalibrationLogger::writeMsg("[DataCollectoreCA] Range id detected.\n");
				int low,high;
				ConstString s = taxel_id_list->get(i).asString();
				if(s.find("#")<0){
					SkinCalibrationLogger::writeMsg("[DataCollectoreCA] Wrong format, aborting.\n");
					initialized = false;
					return initialized;
				}
				low = atoi(s.substr(0,s.find("#")-1).c_str());
				high = atoi(s.substr(s.find("#")+1,-1).c_str());
				for(int j=low; j<high; j++){
					taxel_id.push_back(j);
				}
				SkinCalibrationLogger::writeMsg("[DataCollectoreCA] Added id from %i to %i.\n",low,high);
			}
		}
	}
	config.fromConfigFile(filefullpath.c_str());
	if(!config.check("SaveData")){
		SaveData = false;
	}
	else{
		SaveData = (config.find("SaveData").asInt() == 1);
	}

	this->skinPart = skinPart;
	this->robotBodyPart = robotBodyPart;
	filename = "DataCollectorCA_";
	filename += BodyPart_s[robotBodyPart]+"_"+ SkinPart_s[skinPart];
	filefullpath = rf->getContextPath();
	filefullpath += "/";
	//filefullpath += filename;
	this->robotBodyPart = robotBodyPart;
    this->skinPart = skinPart;
	dataPort.open("/DataCollectorCA/data");
	SkinCalibrationLogger::writeMsg("[DataCollectoreCA] Init end\n");
	initialized = true;
	return initialized;
}

bool DataCollectorCA::calibrate(CalibrationBuffer *cb, map<long int, Vector> *estimates){
	long int id = cb->getTaxelID();
	if(find(taxel_id.begin(), taxel_id.end(), id)!=taxel_id.end()){
		SkinCalibrationLogger::writeMsg("[DataCollectorCA] Receiving %i measure set for taxel %i\n",Forces[id].size()+1,id);
		vector<CalibrationSample *>::const_iterator it;
		
		for(it = cb->getSamples()->begin(); it!=cb->getSamples()->end(); it++){
			Bottle &data = dataPort.prepare();
			data.clear();
			data.addDouble(id);
			Vector Ttemp(12);
			Forces[id].push_back(*((*it)->force));
			Moments[id].push_back(*((*it)->moment));
			for(int i=0;i<3;i++){
				data.addDouble((*(*it)->force)[i]);
			}
			for(int i=0;i<3;i++){
				data.addDouble((*(*it)->moment)[i]);
			}
			for(int i=0;i<3;i++){	
				Ttemp[i] = (*(*it)->ows)[i];
				data.addDouble(Ttemp[i]);
			}
			for(int i=3;i<12;i++){
				Ttemp[i] = (*(*it)->Rws)((i-3)/3,(i-3)-(3*((i-3)/3)));
				data.addDouble(Ttemp[i]);
			}
			T[id].push_back(Ttemp);
			dataPort.write();
		}
	}
	return false;
}
