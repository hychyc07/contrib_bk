/*
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @ingroup icub_contrib_modules
 *
 * \defgroup cameraAlign cameraAlign
 *
 * This module is an extention to the previous cameraAlign module developped by Francesco Rea. It has been developed as a tool for cameras alignment along the vertical axis
 * It uses the feedback coming from the user whom is asked to click on an object present on both images
 * In both the images a horizontal line is drawn and that help in the alignment process
 
 * \author ReaFrancesco
 */

#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


/************************************************************************/
class CameraAlign: public RFModule, public PortReader
{
protected:

    string                              moduleName;     //string containing the module name
    string                              inPortName;     //string containing the name of the input port
    string                              outPortName;    //string containing the name of the output port
    string                              coordPortName;  //string containing the name of the coord port
    int                                 x;              //integer containing the x coord
    int                                 y;              //integer containing the x coord
    bool                                draw;           //boolean to draw the line
    Semaphore                           mutex;

    BufferedPort<Bottle>                coordPort;      //Port for incoming x and y coordinate
    Port                                imageInPort;    //input port for connection with the robot's camera
    Port                                imageOutPort;   //output port for connection with the robot's camera

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        x = 0;
        y = 0;
        draw = false;

        moduleName = rf.check("name",Value("cameraAlign"), "module name (string)").asString();

        setName(moduleName.c_str());

        coordPortName = "/" + moduleName + "/coord:i";
        coordPort.open( coordPortName.c_str() );
        
        inPortName = "/" + moduleName + "/image:i";
        imageInPort.open( inPortName.c_str() );

        outPortName = "/" + moduleName + "/image:o";
        imageOutPort.open( outPortName.c_str() );

        imageInPort.setReader(*this);

        return true;
    }

    /************************************************************************/
    bool interruptModule()
    {
        // order does matter!
        imageInPort.interrupt();
        coordPort.interrupt();        
        imageOutPort.interrupt();
        return true;
    }

    /************************************************************************/
    bool close()
    {
        // order does matter!
        imageInPort.close();
        coordPort.close();        
        imageOutPort.close();
        return true;
    }
    /************************************************************************/
    double getPeriod()
    {
        return 0.5;
    }
    /************************************************************************/
    virtual bool read(ConnectionReader& connection)
    {
        ImageOf<PixelRgb> img;
        img.read(connection);
                
        Bottle *fixIn = coordPort.read(false);

        if (fixIn != NULL)
        {
            x=fixIn->get(0).asInt();
            y=fixIn->get(1).asInt();
            fprintf(stdout, "\nWill now draw line at x= %d and y=%d \n", x, y);
            draw = true;
        }

        if (draw)
            draw::addSegment(img, PixelRgb(255,0,0), 0, y, img.width(), y);

        imageOutPort.write(img);
        return true;
    }
    /************************************************************************/
    bool updateModule()
    {        
        return true;
    }
};

/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("cameraAlign/conf");
    //rf.setDefaultConfigFile("cameraAlign.ini"); 
    rf.configure("ICUB_ROOT",argc,argv);

    CameraAlign cameraAlign;
    return cameraAlign.runModule(rf);
}
