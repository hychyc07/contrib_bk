#include <yarp/dev/Drivers.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include "impController.hpp"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace HandSome;

class StubThd: public RateThread
{
 private:
 	ControllerThd* thd;
 	Vector jnts;
 	
 public:
 	StubThd(const double period, ControllerThd* thd) : RateThread(int(period*1000.0))
 	{
 		this->thd = thd;
 		jnts.resize(5);
 		
 		jnts[0]= 50;
 		jnts[1]= 20;
 		jnts[2]= -10;
 		jnts[3]= 50;
 		
 	}
 	
 	virtual bool threadInit()
 	{
 		cout<<"Stub initialized!"<<endl;
 	}
 	virtual void run()
 	{
 		thd->setJoints(jnts);
 	}
 	virtual void threadRelease()
 	{
 		cout<<"Stub stopped!"<<endl;
 	}
 	 
};

int main()
{
		Network yarp;

        Property option("(device remote_controlboard)");
        option.put("remote","/icubSim/left_arm");
        option.put("local","/left_arm");
        
        PolyDriver* client=new PolyDriver;
        client->open(option);
        
        //Vector stiff;
        Vector joints;
        joints.resize(7);
        joints[0] = 0;
        joints[1] = 0;
        joints[2] = 0;
        joints[3] = 0;
        joints[4] = 0;
        joints[5] = 0;
        joints[6] = 0;
        
		//I need to know the size of the arms
        PositionImpedanceThd* pi = new PositionImpedanceThd(client,(double)10,(double)50,joints,(double)0.2);
        StubThd* stub = new StubThd((double)0.2,pi);
    	
        if(!pi->start()){return 0;}
        if(!stub->start()){return 0;}
        
        Time::delay(1);
        
        stub->stop();
        pi->stop();
        return 0;
}








