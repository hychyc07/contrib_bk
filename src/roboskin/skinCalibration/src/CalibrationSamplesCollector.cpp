
#include "iCub/skinCalibration/CalibrationSamplesCollector.h"

using namespace iCub::skinCalibration;
using namespace std;

CalibrationSamplesCollector::CalibrationSamplesCollector(CalibrationEngine *calibration_engine, int buffer_size, int buffer_number){
	ce = calibration_engine;
    bsize = buffer_size;
	bnum = buffer_number;
}	

CalibrationSamplesCollector::~CalibrationSamplesCollector(){
	map<long int, CalibrationBuffer *>::iterator it;
	for(it=mapping.begin(); it!=mapping.end(); it++){
		delete (*it).second;
	}
}

bool CalibrationSamplesCollector::push(Vector &force,Vector &moment, map<long int, double>& taxels_measures,Matrix &R_ws, Vector &o_ws,Matrix &R_w, Vector &o_w){
	map<long int, double>::iterator it;
	vector<CalibrationSample *> list;
	CalibrationSample *cs;
	list.reserve(taxels_measures.size());
	//First, create a list with all the samples
	for(it=taxels_measures.begin(); it!= taxels_measures.end(); it++){
		if(it == taxels_measures.begin()){ //Create the first one
			cs = new CalibrationSample(it->first,force,moment,taxels_measures,R_ws,o_ws,R_w,o_w);
			list.push_back(cs);
		}
		else{
			//Clone in the others (flyweight pattern)
			list.push_back(cs->CloneFor(it->first));
		}
	}
	//Then commit the list to the calibration engine. We cannot create-and-commit a sample otherwise it could be deleted before 
	//all the cloned samples are created thus breaking the flyweight pattern
	for(unsigned int i=0; i<list.size(); i++){
		if(!this->push(list[i])){

			return false;
		}
	}
	list.clear();

	return true;
}



bool CalibrationSamplesCollector::push(CalibrationSample *calibration_sample){
	long int taxel_id = calibration_sample->taxel_id;
    if(bnum==0 || mapping.size()<=bnum){
		
		if(mapping[taxel_id] == '\0')
			mapping[taxel_id] = new CalibrationBuffer(taxel_id,bsize);
		mapping[taxel_id]->push(calibration_sample);
		if(mapping[taxel_id]->isFull()){ //ready to be processed
			//fprintf(stderr,"[skinCalibration]: CSC buffer of taxel %i is full, ready to be processed\n",taxel_id);
			ce->push(mapping[taxel_id]); //push the buffer in the processing queue
			mapping.erase(taxel_id);     //remove the pointer from the map(the buffer will be deallocated by the CalibrationEngine) 
		}
		return true;
	}
	else{
		return false;
	}
}
