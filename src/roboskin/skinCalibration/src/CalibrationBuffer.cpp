#include "iCub/skinCalibration/CalibrationSample.h"
#include "iCub/skinCalibration/CalibrationBuffer.h"
#include <stdio.h>

using namespace std;
using namespace yarp::sig;
using namespace iCub::skinCalibration;

CalibrationBuffer::CalibrationBuffer(long int taxelid, int size) {

	forces = '\0';
	moments = '\0';
	count = 0;
	this->size = size;
	this->taxel_id = taxelid;
	forces = new Matrix(size,3);
	moments = new Vector(size*3);
	//Wrong, it does not grant that an automatic reallocation will happens after calling vector::insert or vector::push_back
	//samples.resize(size); 
	taxel_measures.resize(size,0);
	samples.reserve(size);
	
	if(taxel_measures.size() != size || forces == '\0' || moments == '\0'){
		SkinCalibrationLogger::writeMsg("[CalibrationBuffer] Cannot allocate memory for the buffer.\n");
	}
}

CalibrationBuffer::~CalibrationBuffer(){
	//fprintf(stderr,"[skinCalibration]: CB deleting %i\n",taxel_id);
	if(forces)
		delete forces;
	if(moments)
		delete moments;
	forces='\0';
	moments='\0';
	taxel_measures.clear();
	//fprintf(stderr,"[skinCalibration]: CB samples to be deleted %i\n",samples.size());
	for(unsigned int i = 0; i< samples.size(); i++){
		   //fprintf(stderr,"[skinCalibration]: CB samples remaining %i\n",samples.size()-i);
		delete samples[i];
	}
	//fprintf(stderr,"[skinCalibration]: CB samples deleted\n");
	samples.clear();

}

bool CalibrationBuffer::push(CalibrationSample *cs){
	if(count == size)
		return false;

	count++;
	forces->setRow(count,*cs->force);
	moments->push_back((*cs->moment)[0]);
	moments->push_back((*cs->moment)[1]);
	moments->push_back((*cs->moment)[2]);
	taxel_measures.push_back((*cs->taxels_measures)[cs->taxel_id]);
	samples.push_back(cs);
	return true;
}

long int CalibrationBuffer::getTaxelID(){
	return taxel_id;
}

Matrix * CalibrationBuffer::getForces(){
	return forces;
}

Vector * CalibrationBuffer::getMoments(){
	return moments;
}

Vector * CalibrationBuffer::getTaxelMeasures(){
	return &taxel_measures;
}

const vector<CalibrationSample *> *CalibrationBuffer::getSamples(){
	return &samples;
}

int CalibrationBuffer::getCount(){
	return count;
}

int CalibrationBuffer::getSize(){
	return size;
}

bool CalibrationBuffer::isFull(){
	return count == size;
}

bool CalibrationBuffer::isEmpty(){
	return count == 0;
}
