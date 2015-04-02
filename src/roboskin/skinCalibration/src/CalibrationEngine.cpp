#include "iCub/skinCalibration/CalibrationEngine.h"

using namespace iCub::skinCalibration;

INITIALIZE_ALGORITHM_LIST

CalibrationEngine::CalibrationEngine(ResourceFinder *resf,BodyPart robotBP, SkinPart skinP):srf("skinCalibration"){
	ca ='\0';
	nalgorithms = 0;
	robotBodyPart = robotBP;
	skinPart = skinP;
	this->rf = resf;
	while(algorithms[nalgorithms]!='\0')nalgorithms++;
	isInitialized = false;
}

CalibrationEngine::~CalibrationEngine(){
	if(ceQueue.size() != 0){
		delete ceQueue.front();
		ceQueue.pop();
	}
	for(int i=0; i<nalgorithms; i++){
		//if(algorithms[i])
			delete algorithms[i];
	}
}

bool CalibrationEngine::threadInit(){
	SkinCalibrationLogger::writeMsg("[CalibrationEngine] Init...\n");
	vector<Vector> v;
    if(!srf.init()){
        SkinCalibrationLogger::writeMsg("[CalibrationEngine] ...SkinManagerClient initialization failed.\n");
        return false;
    }
	v = srf.getTaxelPositionsAndConfidences(skinPart);
	for(unsigned int i=0; i<v.size();i++){
		//if(v[i][0] != 0 || v[i][1] != 0 || v[i][2] != 0){
			position_estimates[i] = v[i];
		//}
	}
	if(ca)
		if(!ca->init(rf,robotBodyPart, skinPart, &position_estimates)){
			SkinCalibrationLogger::writeMsg("[CalibrationEngine] ...Calibration algorithm initialization failed.\n");
			return false;
		}
	SkinCalibrationLogger::writeMsg("[CalibrationEngine] ...Done.\n");
	isInitialized = true;
    return true;
}

void CalibrationEngine::push(CalibrationBuffer *cb){
	qaccess.wait();
		//SkinCalibrationLogger::writeMsg("[CalibrationEngine]: Pushing %li\n",cb->getTaxelID());
		ceQueue.push(cb);
		startprocessing.signal();
	qaccess.post();
}

void CalibrationEngine::run(){
	CalibrationBuffer *cb;
	while (!isStopping()) {
		qaccess.wait();
			if(isStopping()){
			    qaccess.post();
				break;
			}
			//SkinCalibrationLogger::writeMsg("[CalibrationEngine] Queue size %i\n",ceQueue.size());
			while(ceQueue.size() == 0 && !isStopping()){
			    startprocessing.reset();
				qaccess.post(); //...ugly.
				//SkinCalibrationLogger::writeMsg("[CalibrationEngine] Signal wait\n");
				startprocessing.wait();
				//SkinCalibrationLogger::writeMsg("[CalibrationEngine] Signalaled\n");
				qaccess.wait();
			}
			if(isStopping()){
			    qaccess.post();
				break;
			}
			cb = ceQueue.front();
			ceQueue.pop();
		qaccess.post();
		changealgorithm.wait();
			if(isStopping()){
			    changealgorithm.post();
				break;
			}
			if(cb){
				//SkinCalibrationLogger::writeMsg("[CalibrationEngine] Calibrating taxel %li\n",cb->getTaxelID());
				if(ca && ca->isInitialized()){ // if a calibration algorithm has been chosen
					if(ca->calibrate(cb,&position_estimates)){
						srf.setTaxelPosition( skinPart, cb->getTaxelID(),position_estimates[cb->getTaxelID()]);
					}
				}
				//SkinCalibrationLogger::writeMsg(stderr,"[CalibrationEngine] Deleting %li\n",cb->getTaxelID());
				delete cb;
				cb='\0';
			}
			else{
				SkinCalibrationLogger::writeMsg("[CalibrationEngine] Error cb is null \n");
			}
		changealgorithm.post();
	}
}

void CalibrationEngine::onStop(){
	startprocessing.signal();
}

int CalibrationEngine::getAlgorithmsNumber(){
	return nalgorithms;
}

bool CalibrationEngine::changeCalibrationAlgorithm(int num){
	bool resp = false;
	if(num<0 || num>=nalgorithms)
		return false;
	changealgorithm.wait();
		if(algorithms[num] && isInitialized)
			resp = algorithms[num]->init(rf,robotBodyPart, skinPart, &position_estimates);
		else
			resp = true;
		ca = algorithms[num];
	changealgorithm.post();
	return resp;
}
