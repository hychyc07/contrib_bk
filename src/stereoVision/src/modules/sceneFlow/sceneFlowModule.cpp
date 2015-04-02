#include "sceneFlowModule.h"

bool sceneFlowModule::configure(ResourceFinder &rf)
{
    sceneFlow=new SceneFlow(rf);
    if(sceneFlow->isOpen())
    {
        sceneFlow->start();
        Time::delay(0.5);
        return true;
    }
    else
    {
        delete sceneFlow;
        return false;
    }
    namedWindow("flowField");
    namedWindow("Module");
}

bool sceneFlowModule::close()
{
	delete sceneFlow;	
    return true;
}

bool sceneFlowModule::interruptModule()
{
	fprintf(stdout, "Closing Scene Flow...\n");
	sceneFlow->suspend();
 	sceneFlow->close();   	
    return true;
}
bool sceneFlowModule::updateModule()
{
	if(isStopping())
		return false;

    // Whole Flow Access
    int u=160;
    int v=120;
    Mat flow3D;
    sceneFlow->getSceneFlow(flow3D);
    Point3f Pflow3D;

	if(!flow3D.empty())
	{

		Pflow3D.x=flow3D.ptr<float>(v)[2*u]; // Flow DeltaX
		Pflow3D.y=flow3D.ptr<float>(v)[2*u+1]; // Flow DeltaY
		Pflow3D.z=flow3D.ptr<float>(v)[2*u+2]; // Flow DeltaZ
		
	    fprintf(stdout,"3D Motion of Pixel (%i,%i): (%f, %f, %f) \n",u,v,Pflow3D.x,Pflow3D.y,Pflow3D.z);


		// Compute and show motion field and flow module

		IplImage* module=cvCreateImage(cvSize(sceneFlow->getImgWidth(),sceneFlow->getImgHeight()),8,3);
		IplImage* flowField=sceneFlow->draw2DMotionField();
		sceneFlow->drawFlowModule(module);
		
		if(flowField!=NULL)
			cvShowImage("flowField", flowField);
		cvShowImage("Module", module);
		cvWaitKey(5);

		cvReleaseImage(&flowField);
		cvReleaseImage(&module);
	}


    return true;
}

double sceneFlowModule::getPeriod()
{
    return 0.1;
}





int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("sceneFlowModule/conf");
    rf.configure("ICUB_ROOT",argc,argv);
    sceneFlowModule mod;

    return mod.runModule(rf);
}
