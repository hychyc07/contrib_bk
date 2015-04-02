#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <vector>

#define NO_DETECTION_TIMEOUT_SECONDS 2

#define GEN_TEST_DATA
#undef GEN_TEST_DATA

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

/**
  VisionReader vs;
  vs.open("/mymodle/visonreader");
  
  std::vector<int> inds = vs.getCurrObjectIndices()
  int rand_obj_index=inds[rand()%inds.size()];
  double x,y,z;
  vs.getCoords(rand_obj_index,x,y,z);
*/

class VisionReader : public BufferedPort<Bottle> {

    
public:
  
    VisionReader();
  
    virtual void onRead(Bottle& b) ;
    
    virtual std::string getObjectLabel(int index);
    
    virtual int getObjectIndex(std::string label);
    
    virtual std::vector<int> getCurrObjectIndices();
    
    virtual void getCoords(int index,double &x,double &y,double &z);
    

protected:
  
    Semaphore sem;
    std::vector<double> latestXs;
    std::vector<double> latestYs;
    std::vector<double> latestZs;
    std::vector<std::string> labels;
    std::vector<double> lastSeenTime;
      
    
};