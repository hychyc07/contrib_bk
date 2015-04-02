/** Usage:
yarp connect /testgmr /iFumble/learning/givemedata ; yarp connect /testgmrrpc /iFumble/learning/rpc
*/

#include <yarp/os/all.h>

#include <iostream>
#include <iomanip>
#include <string>
//#include <rand>
//#include <MathLib.h>
#include <LasaMatrix.h>


using namespace std;
using namespace yarp;
using namespace yarp::os;

/*
int main2(int argc, char *argv[])
{
    Network yarp;

    BufferedPort<Bottle> myPort;
    myPort.open("/testgmr");

    if (!yarp.checkNetwork())
        return -1;

    int num_data=5;
    int num_dim=8;
    while (true) {
        Time::delay(1);
        for (int i=0;i<num_data;i++) {
            Bottle& b=myPort.prepare();
            b.addString("DATAPOINT");
            b.addInt(num_dim);
            for (int j=0;j<num_dim;j++)
                b.addDouble(((double)rand())/RAND_MAX);
            myPort.write();
        }
    }

}
*/


int main(int argc, char *argv[])
{
    Network yarp;
    ConstString bottlestring;
    BufferedPort<Bottle> myPort;
    //BufferedPort<Bottle> myPort2;
    Port myPort2;
    myPort.open("/testgmr");
    myPort2.open("/testgmrrpc");
    bool connecting=true;
    do{
      Time::delay(1);
      std::cout<<"Trying connect "<<std::endl;
      connecting=true;
      connecting=connecting&&myPort.addOutput("/iFumble/iLearnStuff/givemedata","tcp");
      connecting=connecting&&myPort2.addOutput("/iFumble/iLearnStuff/rpc","tcp");
    }while(!connecting);

    Time::delay(1);

    if (!yarp.checkNetwork())
        return -1;

    int num_data=5;
    int num_dim=10;
    int infer_from_dim=2;
    int num_inferences=10;

    Time::delay(1);
    for (int i=0;i<num_data;i++) {
        Time::delay(0.02);
        Bottle& b=myPort.prepare();b.clear();
        b.addString("DATAPOINT");
	b.addString("LEARNER0");
        b.addInt(num_dim);
        MathLib::Vector learnvec(num_dim);
        //makes stars
        learnvec[0]=(((double)rand())/RAND_MAX);
        for (int dimno=1;dimno<num_dim;dimno++) {
            double randn=((double)rand())/RAND_MAX;
            double randn2=((double)rand())/RAND_MAX;
            if (randn<0.5)
                learnvec[dimno]=(0.5*learnvec[dimno-1]+0.5*randn2);
            else
                learnvec[dimno]=(0.5- 0.5*learnvec[dimno-1]+0.5*randn2);
        }
        for (int dimno=0;dimno<num_dim;dimno++) {
            b.addDouble(learnvec[dimno]);
        }
        bottlestring=b.toString();
        std::cout<<"SENDING BOTTLE TO DATAIN: "<<bottlestring<<std::endl;
        myPort.write();
    }

Time::delay(8);

    for (int i=0;i<num_inferences;i++) {
        //Bottle& b=myPort2.prepare();b.clear();
	Bottle request,response;
	request.clear();response.clear();
        request.addString("INFER");
	request.addString("LEARNER0");
        request.addInt(infer_from_dim);
        MathLib::Vector components(infer_from_dim);
        for (int j=0;j<infer_from_dim;j++) {
            bool found=false;
            int testind=rand()%infer_from_dim;
            for (int jj=0;jj<j;jj++) {
                if (components[jj]==testind)found=true;
            }
            if (!found)components[j]=testind;
            else j--;
        }

        for (int j=0;j<infer_from_dim;j++) {
            request.addInt(components[j]);

        }
        for (int j=0;j<infer_from_dim;j++) {

            request.addDouble(((double)rand())/RAND_MAX);
        }
        //myPort2.write();
	bottlestring=request.toString();
	std::cout<<"Sent inference string "<<bottlestring.c_str()<<std::endl;
	myPort2.write(request,response);
        
        //std::cout<<"Sent inference string "<<bottlestring.c_str()<<std::endl;
      //  Bottle *response=myPort.read();
        //response.read(myPort);
        bottlestring=response.toString();
        std::cout<<"READ BOTTLE BACK: "<<bottlestring<<std::endl;
      //  delete response;		
    }


}


