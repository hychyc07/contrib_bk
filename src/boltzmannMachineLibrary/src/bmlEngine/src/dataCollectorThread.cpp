// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
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
 * @file dataCollectorThread.cpp
 * @brief main code for bulding and sending images ( see dataCollectorThread.h )
 */

//opencv includes
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

//yarp includes
#include <yarp/math/Math.h>

//within project includes
#include <iCub/dataCollectorThread.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

/**
* initialise the thread
*/
bool dataCollectorThread::threadInit(){
    inhibit = false;
    sampleGeneralPort = new BufferedPort<Bottle>;
    printf("Image Thread initialising: \n");
    string name = getName("/bmlEngine/sample:i");
    printf("name of the port %s \n", name.c_str());
    sampleGeneralPort->open(name.c_str());
    /*
    printf("opening image port..... \n");
    string str=getName();
    str.append("/sample:i");
    samplePort.open((const char*)str.c_str());
    */
    /*
    str=getName();
    str.append("/weight:o");
    weightPort.open((const char*)str.c_str());*/
    printf("successfuly  allocated a port for samples insertion \n");
    return true;
}

/**
* code that is executed after the thread starts
* @param s is true if the thread started
*/
void dataCollectorThread::afterStart(bool s){
    if(s){
        printf("Image Thread after start.....\n");
    }
}

/**
* running code of the thread
*/
void dataCollectorThread::run(){
    printf("collecting Thread \n");
    int totRows  = mb->getLayer(0)->getRow();
    int totUnits = mb->getLayer(0)->getCol();
    printf("extracting the dimension \n");
    const int dim = totRows * totUnits;
    Vector sample(dim);
    while (isStopping() != true) {
        if(!inhibit) {
            //sample.clear();
            if(sampleImagePort->getInputCount()) {
                ImageOf<PixelMono>* ptr_inputImageMono = sampleImagePort->read(true);
                if(ptr_inputImageMono!=0) {
                    bool inputImage_flag=true;
                    bool inputColour=false;
                    int width,height;
                    IplImage* im_tmp_ipl;
                    if(inputImage_flag){
                        if(inputColour) {
                            //1.extracts 3 planes
                            //width=ptr_inputImage->width();
                            //height=ptr_inputImage->height();
                            //im_tmp_ipl = cvCreateImage( cvSize(width,height), 8, 1 );
                            //cvCvtColor(ptr_inputImage->getIplImage(),im_tmp_ipl,CV_RGB2GRAY);
                        }
                        else {
                            //2.copying one channel planes
                            width=ptr_inputImageMono->width();
                            height=ptr_inputImageMono->height();
                            im_tmp_ipl = cvCreateImage( cvSize(width,height), 8, 1 );
                            cvCopy(ptr_inputImageMono->getIplImage(),im_tmp_ipl);
                        }
                        
                        //2.Extract the necessary information
                        int rectDimX = width / totUnits;
                        int rectDimY = height / totRows;
                        //3.maps the intensity on to the layer
                        int sum=0;
                        uchar* data = (uchar*)(im_tmp_ipl->imageData);
                        int step       = im_tmp_ipl->widthStep/sizeof(uchar);
                        //printf("step of the gray image as input %d",step);
                        for(int r = 0 ; r < totRows ; r++){
                            for(int c = 0 ; c < totUnits ; c++){
                                sum=0;
                                for(int y = 0 ; y < rectDimY ; y++){
                                    for(int x = 0 ; x < rectDimX ; x++){
                                        sum += data[r * rectDimY * width + c * rectDimX + width * y + x];
                                        //sum+=im_tmp_ipl[boltzmannMachineRow*rectDimY*320+boltzmannMachineCol*rectDimX+320*y+x];
                                    }
                                }
                                double mean = sum / (rectDimX * rectDimY);
                                sample[r * totUnits + c] = mean / 255;
                            }
                        }
                        mb->addSample(sample);
                    }
                }
                
            }
            else if(sampleGeneralPort->getInputCount()) {
                printf("sampling from general port \n");
                Bottle* b = sampleGeneralPort->read(true);
                printf("read from the sample port %d,  %d \n", b->size(), dim);
                //int dimList = b->size();
                printf("dimension of the list dimension of the layer ");
                sample[0] = 0.5;
                for(int i=0; i < 625; i++) {
                    sample[i] = b->get(i).asDouble();
                    printf("%f ",b->get(i).asDouble());
                }
                mb->addSample(sample);
            }
        }//end of inhibit
    } // end of while
    printf("exiting from the isRunning cycle \n");
}

void dataCollectorThread::onStop() {
    sampleImagePort->interrupt();
    sampleGeneralPort->interrupt();
    sampleImagePort->close();
    sampleGeneralPort->close();
}

/**
* code executed when the thread is released
*/
void dataCollectorThread::threadRelease(){
    printf("Image Thread releasing..... \n");
}

void dataCollectorThread::setName(string n){
    name=n.substr(0,n.size());
}

std::string dataCollectorThread::getName(const char* p){
    string str(name);
    str.append(p);
    //printf("name: %s", name.c_str());
    return str;
}

std::string dataCollectorThread::getName() {
    return name;
}

void dataCollectorThread::setInputData(BufferedPort<ImageOf<PixelMono> >* port) {
    printf("setting input data \n");
    sampleImagePort = port;
}

void dataCollectorThread::setMachineBoltzmann(MachineBoltzmann* machine) {
    printf("setting machine boltzmann reference \n");
    mb = machine;
}
