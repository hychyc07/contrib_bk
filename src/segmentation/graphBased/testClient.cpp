// Author: Elena Ceseracciu, RBCS-Istituto Italiano di Tecnologia - 2012

#include <string>
#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>

#include "SegmModule.h"

using namespace yarp::os;
using namespace yarp::sig;

int main(int argc, char *argv[]) {
   
    Network yarp;   

    if (!yarp.checkNetwork())
        return -1;


    Port clientPort;
    Port fixationPort;
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > maskPort;


    ResourceFinder rf;
    rf.configure("ICUB_ROOT.ini", argc, argv);
    std::string name="testGbsClient";
    if (rf.check("name"))
    {
        name=rf.find("name").asString();
    }
    
    std::string slash="/";
    clientPort.open((slash+name+slash +"rpc:o").c_str());
    fixationPort.open((slash+name+slash +"interestPoint:i").c_str()); 
    maskPort.open((slash+name+slash +"mask:o").c_str());
    
    
    SegmentationModuleInterface segmIf;
    segmIf.yarp().attachAsClient(clientPort);
    
    
    while(true)
    {
        Bottle fixIn;
        fixIn.clear();
        fixationPort.read(fixIn);

        if (fixIn!=NULL)
        {
            double fix_x = fixIn.get(0).asDouble();
            double fix_y = fixIn.get(1).asDouble();
//             double cropSizeWidth = fixIn.get(2).asInt();
//             double cropSizeHeight = fixIn.get(3).asInt();
            std::vector<Pixel> pixelList=segmIf.get_component_around(Pixel(fix_x, fix_y));
            std::cout<<"asked server"<<std::endl;
            yarp::sig::ImageOf< yarp::sig::PixelRgb > &img=maskPort.prepare();
            img.resize(320, 240);
            img.zero();
            for(std::vector<Pixel>::const_iterator pixIt=pixelList.begin(); pixIt!=pixelList.end(); ++pixIt)
            {
                img.pixel(pixIt->x, pixIt->y) = yarp::sig::PixelRgb(255, 255, 255);
            }
            maskPort.write();
           
        }
    }
    return 0;
}