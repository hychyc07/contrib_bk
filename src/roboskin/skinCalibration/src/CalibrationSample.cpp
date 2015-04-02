#include "iCub/skinCalibration/CalibrationSample.h"

using namespace iCub::skinCalibration;


long int CalibrationSample::allocated = 0;
long int CalibrationSample::heavyweight = 0;
Semaphore CalibrationSample::statistics;

long int CalibrationSample::getAllocated(){
	return allocated;
}

long int CalibrationSample::getHeavyWeight(){
	return heavyweight;
}

bool CalibrationSample::lastRemaining(){
	return (*counter) == 1;
}

CalibrationSample::CalibrationSample(){
	taxel_id = 0;
	force = '\0';
	moment = '\0';
	taxels_measures = '\0';
	counter = '\0';
	access='\0';
	statistics.wait();
	allocated++;
	statistics.post();
}

CalibrationSample::CalibrationSample(long int taxel_id, Vector &force, Vector &moment, map<long int,double> &taxels_measures,Matrix &R_ws, Vector &o_ws, Matrix &R_w, Vector &o_w){
	this->taxel_id=taxel_id;
	this->force = new Vector(force);
	this->moment = new Vector(moment);
	this->taxels_measures = new map<long int,double>(taxels_measures);
	this->access = new Semaphore();
	this->Rws = new Matrix(R_ws);
	this->ows = new Vector(o_ws);
	this->ow = new Vector(o_w);
	this->Rw = new Matrix(R_w);
	counter = new int;
	*counter = 1;
	statistics.wait();
	allocated++;
	heavyweight++;
	statistics.post();
}

CalibrationSample::~CalibrationSample(){
	access->wait();
	(*counter)--;
	statistics.wait();
	allocated--;
	statistics.post();
	if(*counter == 0){
		delete counter;
		delete force;
		delete moment;
		delete taxels_measures;
		delete Rws;
		delete ows;
		delete Rw;
		delete ow;
		counter = '\0';
		force = '\0';
		moment = '\0';
		taxels_measures = '\0';
		Rws='\0';
		Rw='\0';
		ows='\0';
		ow='\0';
		statistics.wait();
		heavyweight--;
		statistics.post();
		access->post();
		delete access;
		access='\0';
	}
	else{
		counter = '\0';
		force = '\0';
		moment = '\0';
		taxels_measures = '\0';
		access->post();
		access = '\0';
	}
}

CalibrationSample* CalibrationSample::CloneFor(long int this_taxel_id){
	//If I am inside this function I am sure that at least one CalibrationSample exists. The data cannot be eliminated until this function ends thus I can safely create a new CalibrationSample.
	CalibrationSample *t = new CalibrationSample();
	t->counter = counter;
	t->force = force;
	t->moment = moment;
	t->Rws = Rws;
	t->Rw = Rw;
	t->ows = ows;
	t->ow = ow;
	t->access = access;
	t->taxels_measures = taxels_measures;
	t->taxel_id = this_taxel_id;
	access->wait();
	(*counter)++;
	access->post();
	return t;
}
