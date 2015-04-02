#include <stdio.h>
#include "hmaxModel.h"
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <cv.h>
#include <highgui.h>
#include <fstream>
#include "hmaxModel.h"
#include <iostream>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

class hminModule: public RFModule
{
	hmaxModel *model;
	BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePort;
	BufferedPort<yarp::os::Bottle> modelOutput;
	dictionary *dict;
	IplImage *img;

public:
	bool configure(ResourceFinder &rf)
	{
		Bottle &b = rf.findGroup("HMAX");

		dict = (dictionary *) malloc(sizeof(dictionary));
		dict->pfCount = b.find("pfCount").asInt();
		dict->yxCountMax = b.find("yxCountMax").asInt();
		dict->fCount = b.find("fCount").asInt();
		dict->fSizes = (double*)  malloc(dict->fCount * sizeof(double));
		dict->fVals  = (float*) malloc (dict->pfCount * dict->yxCountMax * dict->yxCountMax * dict->fCount * sizeof(float));

		std::cout << "Opening " << rf.getContextPath() << "/" << b.find("fSizes").asString() << std::endl;
		ifstream ifile((rf.getContextPath()+"/"+b.find("fSizes").asString()).c_str());

		for(int s = 0; s < dict->fCount; s++) ifile >> dict->fSizes[s];
		ifile.close();

		std::cout << "Opening " << rf.getContextPath() << "/" << b.find("fSizes").asString()  << std::endl;
		ifile.open((rf.getContextPath()+"/"+b.find("fVals").asString()).c_str());

		for(int s = 0; s < (dict->pfCount * dict->yxCountMax * dict->yxCountMax * dict->fCount); s++) ifile >> dict->fVals[s];
		ifile.close();

		model = new hmaxModel(b.find("imgDepth").asInt(), dict);

		imagePort.open("/hminModule/image/in");
		modelOutput.open("/hminModule/out");

		return true;
	}
	double getPeriod()
	{
		return 0.0;
	}
	bool updateModule()
	{
		ImageOf<PixelRgb> *image = imagePort.read();
		float *out=new float[model->FCount()];
		if ( image != NULL )
		{
			img = (IplImage*) image->getIplImage();
			model->response(img, out);

			Bottle &responseOut = modelOutput.prepare();
			responseOut.clear();

			for(int s = 0; s < model->FCount(); s++) {
				responseOut.addDouble((double) out[s]);
				std::cout << out[s] << ", ";
			}
			std::cout << std::endl;
			modelOutput.write();
		}
        delete [] out;
		return true;
	}
	bool close()
	{
		delete model;
		delete dict->fSizes;
		delete dict->fVals;
		delete dict;

        return true;
	}
	bool interruptModule()
	{
		return true;
	}
};

int main(int argc, char **argv)
{
	Network yarp;

	if ( !yarp.checkNetwork() ) {
		std::cout << "Error. Unable to check yarp network!" << std::endl;
		return -1;
	}

	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultContext("mitcub/conf");
	rf.setDefaultConfigFile("config.ini");
	rf.configure("ICUB_ROOT",argc,argv);
	rf.setDefault("name","mitcub");

	hminModule hm;

	if (!hm.configure(rf)){
		std::cerr << "Error opening the hmin module!" << std::endl;
		return -2;
	}

	return hm.runModule(rf);

}
