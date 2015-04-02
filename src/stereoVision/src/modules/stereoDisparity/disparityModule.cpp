#include "disparityModule.h"
#include <yarp/os/Stamp.h>



bool stereoModule::configure(yarp::os::ResourceFinder &rf)
{    


    string moduleName = rf.check("name", Value("stereoDisparity"), "module name (string)").asString().c_str();
    setName(moduleName.c_str());


    handlerPortName = "/";
    handlerPortName += getName(rf.check("CommandPort",Value("/rpc"),"Output image port (string)").asString().c_str());



    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << ": unable to open port " << handlerPortName << endl;
        return false;
    }

    attach(handlerPort);


    dispThread = new disparityThread(rf,&handlerPort);

    dispThread->start(); 

    return true ;

}


bool stereoModule::interruptModule()
{
   dispThread->stop();


    return true;
}


bool stereoModule::close()
{

    dispThread->stop();
    delete dispThread;

    return true;
}


bool stereoModule::respond(const Bottle& command, Bottle& reply) 
{
    if(command.size()==0)
        return false;

    if (command.get(0).asString()=="quit") {
        cout << "closing..." << endl;
        return false;
    }

    if(command.get(0).asString()=="set" && command.size()==10)
    {
        bool bestDisp=command.get(1).asInt() ? true : false;
        int uniquenessRatio=command.get(2).asInt();
        int speckleWindowSize=command.get(3).asInt();
        int speckleRange=command.get(4).asInt();
        int numberOfDisparities=command.get(5).asInt();
        int SADWindowSize=command.get(6).asInt();
        int minDisparity=command.get(7).asInt();
        int preFilterCap=command.get(8).asInt();
        int disp12MaxDiff=command.get(9).asInt();

        dispThread->setDispParameters(bestDisp,uniquenessRatio,speckleWindowSize,speckleRange,numberOfDisparities,SADWindowSize,minDisparity,preFilterCap,disp12MaxDiff);
    }
    if (command.get(0).asString()=="disparity") {
        if(command.size()==2  && command.get(1).asString()=="on" )
            dispThread->compute(true);
        else
            dispThread->compute(false);

        if(dispThread->isComputing())
            reply.addString("Disparity Computation ON");
        else
            reply.addString("Disparity Computation OFF");
        return true;
    }
    else if (command.get(0).asString()=="Point" || command.get(0).asString()=="Left" ) {
        int u = command.get(1).asInt();
        int v = command.get(2).asInt(); 
        Point3f point = dispThread->get3DPoints(u,v);
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
    }
    else if (command.size()==2) {
        int u = command.get(0).asInt();
        int v = command.get(1).asInt(); 
        Point3f point = dispThread->get3DPoints(u,v,"ROOT");
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
    }
    else if (command.get(0).asString()=="Right") {
        int u = command.get(1).asInt();
        int v = command.get(2).asInt();
        Point3f point = dispThread->get3DPoints(u,v,"RIGHT");
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
    }

    else if (command.get(0).asString()=="Root") {
        int u = command.get(1).asInt();
        int v = command.get(2).asInt();
        Point3f point = dispThread->get3DPoints(u,v,"ROOT");
        reply.addDouble(point.x);
        reply.addDouble(point.y);
        reply.addDouble(point.z);
    }

    else if (command.get(0).asString()=="cart2stereo") {
        double x = command.get(1).asDouble();
        double y = command.get(2).asDouble();
        double z = command.get(3).asDouble();

        Point2f pointL = dispThread->projectPoint("left",x,y,z);
        Point2f pointR = dispThread->projectPoint("right",x,y,z);

        reply.addDouble(pointL.x);
        reply.addDouble(pointL.y);
        reply.addDouble(pointR.x);
        reply.addDouble(pointR.y);
    }

    else if(command.size()>0 && command.size()%4==0)
    {
        for(int i=0; i<command.size(); i+=4)
        {
            double ul = command.get(i).asDouble();
            double vl = command.get(i+1).asDouble();
            double ur = command.get(i+2).asDouble();
            double vr = command.get(i+3).asDouble();

            Point3f point= dispThread->get3DPointMatch(ul,vl,ur,vr,"ROOT");
            reply.addDouble(point.x);
            reply.addDouble(point.y);
            reply.addDouble(point.z);
        }

    }
    else
        reply.addString("NACK");
    return true;
}


bool stereoModule::updateModule()
{
    return true;
}



double stereoModule::getPeriod()
{
    return 0.1;
}
