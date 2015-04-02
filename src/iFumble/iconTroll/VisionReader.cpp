#include "VisionReader.h"


    VisionReader::VisionReader(){
	useCallback();

    }
    
    void VisionReader::onRead(Bottle& b){
      sem.wait();
      double now=Time::now();
      std::string shouldSayObject(b.get(0).asString().c_str());
      if(!shouldSayObject.compare("OBJECT")){
	std::string inputLabel(b.get(1).asString().c_str());
	int obj_ind = getObjectIndex(inputLabel);
	double newX=b.get(2).asDouble();
	double newY=b.get(3).asDouble();
	double newZ=b.get(4).asDouble();
	if(obj_ind<0){
	  labels.push_back(inputLabel);
	  lastSeenTime.push_back(now);
	  latestXs.push_back(newX);
	  latestYs.push_back(newY);
	  latestZs.push_back(newZ);
	  std::cout<<"Adding new object bottle "<<b.toString()<<std::endl;
	}else{
	  labels[obj_ind]=(inputLabel);
	  lastSeenTime[obj_ind]=(now);
	  latestXs[obj_ind]=(newX);
	  latestYs[obj_ind]=(newY);
	  latestZs[obj_ind]=(newZ);	  
	  std::cout<<"Updating object no "<<obj_ind<<" with bottle "<<b.toString()<<std::endl;
	}
      
      }else{
	std::cerr<<"Received unknown vision command "<<b.toString()<<std::endl;
      }
      sem.post();
    }
    
    std::string VisionReader::getObjectLabel(int index){
	#ifdef GEN_TEST_DATA
	return std::string("TESTOBJECTGARBAGE");
	#endif      
      sem.wait();
      std::string toReturn=labels[index];
      sem.post();
      return toReturn;
    }
    
    std::vector<int> VisionReader::getCurrObjectIndices(){
      	#ifdef GEN_TEST_DATA
      	std::vector<int> newvec;
	newvec.push_back(0);
	return newvec;
	#endif      
      sem.wait();
      std::vector<int> toReturn;
      double now = Time::now();
      for(int i=0;i<lastSeenTime.size();i++){
	  if((now-lastSeenTime[i])<=NO_DETECTION_TIMEOUT_SECONDS){
	    toReturn.push_back(i);
	  }
      }
      sem.post();
      return toReturn;
    }
    
    void VisionReader::getCoords(int index,double &x,double &y,double &z){
	#ifdef GEN_TEST_DATA
	x=-0.2+0.1*((double)rand())/RAND_MAX;
	y=0.2*((double)rand())/RAND_MAX-0.1;
	z=0.2;
	return;
	#endif            
      sem.wait();
      x=latestXs[index];
      y=latestYs[index];
      z=latestZs[index];
      sem.post();
    }
    
    int VisionReader::getObjectIndex(std::string label){
	#ifdef GEN_TEST_DATA
	return 0;
	#endif         
      
      sem.wait();
      int toReturn=-1;
      for(int i=0;i<labels.size();i++){
	  if(!labels[i].compare(label)){
	    toReturn=i;
	    break;
	  }
      }
      sem.post();
      return toReturn;
    }