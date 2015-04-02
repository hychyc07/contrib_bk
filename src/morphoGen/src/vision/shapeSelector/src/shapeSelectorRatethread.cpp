// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Ajaz Ahmad Bhat
  * email: ajaz.bhat@iit.it
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
 * @file shapeSelectorRatethread.cpp
 * @brief Implementation of the eventDriven thread (see shapeSelectorRatethread.h).
 */

#include <shapeSelectorRatethread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace cv;

#define THRATE 33 //ms

shapeSelectorRatethread::shapeSelectorRatethread():RateThread(THRATE) {
    robot = "icub";        
}

shapeSelectorRatethread::shapeSelectorRatethread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
}

shapeSelectorRatethread::~shapeSelectorRatethread() {
    // do nothing
}

bool shapeSelectorRatethread::threadInit() {

    
        

    if (!inputBottlePort[0].open(getName("/leftBottle:i").c_str())) {
        cout << ": unable to open port to receive coordinates for left camera image\n"  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    
    if (!inputBottlePort[1].open(getName("/rightBottle:i").c_str())) {
        cout << ": unable to open port to receive coordinates for right camera image\n"  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   
    
    if (!inputImagePort[0].open(getName("/leftImage:i").c_str())) {
        cout << ": unable to open port to receive left camera input image\n"  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    
    if (!inputImagePort[1].open(getName("/rightImage:i").c_str())) {
        cout << ": unable to open port to receive right camera input image\n"  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
     
    
    if (!outputImagePort[0].open(getName("/leftImage:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   
    
    if (!outputImagePort[1].open(getName("/rightImage:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }


    
    /*if (!Network::connect("/MyRemembered:o","/shapeSelector/rememberedX:i"))
        return false;    
   //printf("Connection to rememberedX");
    if (!Network::connect("/Useful/PastXperiences:o","/shapeSelector/cue:i"))
        return false;
    //if (!Network::connect("/Hub:o","/shapeSelector/hub:i"))
    //   return false;
    */
    //Network::connect("/PlanXplore:o","/shapeSelector/plan:i");
        //printf("connection NOT successful\n");
       
    
    scaleX  =   dimX / 320;
    scaleY  =   dimY / 240; 

    for (int j = 0; j < 2; j++)
        for(int i = 0; i < 3; i++)  {        // initialize the coordinates
            xLeft[j][i]     =   -1;
            yTop[j][i]      =   -1;
            xWidth[j][i]    =   0;
            yHeight[j][i]   =   0;
            objectID[j][i]  =   -1;
        }   
    return true;
}

void shapeSelectorRatethread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string shapeSelectorRatethread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void shapeSelectorRatethread::setInputPortName(string InpPort) {
        
}

void shapeSelectorRatethread::setSharingBottle(Bottle *lBot, Bottle *rBot) {
 
       leftBot  =   lBot;
       rightBot =   rBot;
}


/*
void shapeSelectorRatethread::updateObjects(ImageOf<PixelMono>* inputImage, ImageOf<PixelMono>* outputImage, int k, int x) {
    
     
    for (int i = 0; i < 3; i++) {   // for each bouning rectangle
        oproc       =   outputImage->getRawImage();   
        if (x) {
            inproc      =   inputImage->getRawImage();
            for (int r = 0; r < height; r++) {
                    for (int c = 0; c < width; c++) {
                        if ( ( r >= yTop[k][i]) && (r <= yHeight[k][i]) && (c >= xLeft[k][i]) && (c <= xWidth[k][i] ) ) {
                            *oproc = *inproc;       // copy the data from input image     
                        }    
                        oproc++;
                        inproc++;                             
                    }
                    oproc+=padding;
                    inproc+=padding;                                  
            }
        }
        else {
            for (int r = 0; r < height; r++) {
                    for (int c = 0; c < width; c++) {
                        if ( ( r >= yTop[k][i]) && (r <= yHeight[k][i]) && (c >= xLeft[k][i]) && (c <= xWidth[k][i] ) ) {
                            *oproc = 255;           // set the pixels in the rectangle to white
                        }    
                        oproc++;                 
                    }
                    oproc+=padding;                                
            }
        }
    }
}
*/

void shapeSelectorRatethread::run() {


    /*
    endTime = Time::now();
    double interval = endTime - startTime;
    printf("interval %f \n", interval);
    startTime = Time::now();
    */ 
    //printf("scaleX %d scaleY %d\n", scaleX,scaleY);       
    for (int j = 0 ; j < 2; j++)  {                 // left case (0) and right case (1) 
        
        // Bottle handling
        if( inputBottlePort[j].getInputCount()){
            incomingBottle    =   inputBottlePort[j].read(false);
            if (incomingBottle  !=  NULL)   {
                for(int i = 0; i < 3; i++)  {        // initialize the coordinates for every new bottle
                    xLeft[j][i]     =   -1;
                    yTop[j][i]      =   -1;
                    xWidth[j][i]    =   0;
                    yHeight[j][i]   =   0;
                    objectID[j][i]  =   -1;                                      
                }
                objectCount = incomingBottle->get(0).asInt();
                if (incomingBottle->size() > 0) {
                    
                    for (int i = 0; i < 3; i++) {   // we have limited the number of objects to 3. Please use "objectCount" to manage dynamically
                        Bottle * tempBottle =   incomingBottle->get(i+1).asList();
                        // The below code line bounds the vision area to be processed
                        //if((tempBottle != NULL) && ( (abs(tempBottle->get(0).asInt()) > (0.2*width)) && (abs(tempBottle->get(1).asInt()) > (0.2*height)) && ((abs(tempBottle->get(0).asInt()) + abs(tempBottle->get(2).asInt())) < (0.80*width)) && ((abs(tempBottle->get(1).asInt()) + abs(tempBottle->get(3).asInt())) < (0.8*height))  )){
                        if (tempBottle != NULL){
                            xLeft[j][i]         =   abs(tempBottle->get(0).asInt())/scaleX; // Divison by scale signifies an external constraint
                            yTop[j][i]          =   abs(tempBottle->get(1).asInt())/scaleY; // as in our case we have images and dimensions coming from
                            xWidth[j][i]        =   abs(tempBottle->get(2).asInt())/scaleX; // different modules.
                            yHeight[j][i]       =   abs(tempBottle->get(3).asInt())/scaleY;
                            objectID[j][i]      =   tempBottle->get(4).asInt();
                            
                        }
                        printf("handling object %d: %d %d %d %d with ID %d \n", i, xLeft[j][i], yTop[j][i], xWidth[j][i], yHeight[j][i], objectID[j][i]); 
                                   
                    }
                    incomingBottle->clear();
                    if(incomingBottle->size() != 0) {
                        printf("Error\n");   
                    }
                }  
            }
        }
        else {
            for(int i = 0; i < 3; i++)  {        // initialize in case of no input bottle ports
                xLeft[j][i]     =   -1;
                yTop[j][i]      =   -1;
                xWidth[j][i]    =   0;
                yHeight[j][i]   =   0;
                objectID[j][i]  =   -1;
                }
        } 

        
    }
    
          
    for (int j = 0 ; j < 2; j++)  {                 // left case (0) and right case (1)    

        // image handling
        if (outputImagePort[j].getOutputCount()) {
            //printf("%d\n",foreground.at<int>(0, 0));                           
            ImageOf<PixelRgb>& outputImage =  outputImagePort[j].prepare();
            outputImage.resize(width, height);
            padding = outputImage.getPadding();
            
            outputImage.zero();
            if (inputImagePort[j].getInputCount())  {   // if any input ports are connected
                tempImage  =   inputImagePort[j].read(false);
                
                if (tempImage  !=  NULL)   {
                    inputImage[j]   =   tempImage; // to save data from previous run tempImage is introduced
                }
                
                //outputImage.resize(width, height);
                //
                if (inputImage[j]  !=  NULL)   {
                    
                    width           =   inputImage[j]->width();
                    height          =   inputImage[j]->height();
                    printf("input width height %d %d \n",width,height);
                    outputImage.resize(width, height);
                    outputImage.zero();
                    foreground      =   Mat(height,width,CV_8UC3,cv::Scalar(0,0,0));
                    mask            =   Mat(height,width,CV_8UC3,cv::Scalar(0,0,0));
                    inputIplImage   =   *((IplImage*) inputImage[j]->getIplImage());   
                    temp1            =   &inputIplImage;
                    cv::cvtColor(temp1, temp, CV_BGR2RGB);
                    //cv::Mat in[]    =   {temp, temp, temp};
                    //cv::merge(in, 3, img0);
                    int i = 0;
                    cv::Mat bgdModel, fgdModel;
                    for (int i = 0; i < 3; i++) {
                        rect = Rect (xLeft[j][i], yTop[j][i], xWidth[j][i], yHeight[j][i]);
                        printf("rect values %d: %d %d %d %d rect area: %d \n", i, xLeft[j][i], yTop[j][i], xWidth[j][i], yHeight[j][i], rect.area());
                        if (rect.area() > 0)    {
                            grabCut( temp, mask, rect, bgdModel, fgdModel, 1 ,cv::GC_INIT_WITH_RECT);
                            // Get the pixels marked as likely foreground
                            //cv::compare(mask,cv::GC_PR_FGD,mask,cv::CMP_EQ);
                            // Generate output image                        
                            temp.copyTo(foreground,mask);
                            //printf("this is the first character %c",foreground.at<unsigned char>(0,0));
                            
                            
                        }                      
                    }
                    outputIplImage = foreground;
                    outputImage.wrapIplImage(&outputIplImage);              
                }
                outputImagePort[j].write();                 
                
            }

            else{   // set white patches into the rectangles
                outputImage.zero();
                for (int i = 0; i < 3; i++) {
                    oproc       =   outputImage.getRawImage();
                    for (int r = 0; r < height; r++) {
                        for (int c = 0; c < width; c++) {
                            if ( ( r >= yTop[j][i]) && (r <= (yTop[j][i] + yHeight[j][i])) && (c >= xLeft[j][i]) && (c <= (xLeft[j][i] + xWidth[j][i] )) ) {
                                
                                *oproc++ = 255;
                                *oproc++ = 255;
                                *oproc++ = 255;
                            }
                            else    
                            oproc+=3;                 
                        }
                        oproc+=padding;                                
                    }
                }
                outputImagePort[j].write();
                //oproc = NULL;
            }
                  
        }
          
    }// for loop for left and right cases ends here                   
    //printf("Finishing the rate thread run \n");
    //watershed();          
}

void shapeSelectorRatethread::threadRelease() {

    for (int i = 0; i < 2; i++) {
        outputImagePort[i].interrupt();
        outputImagePort[i].close();
        inputImagePort[i].interrupt();
        inputImagePort[i].close();
        inputBottlePort[i].interrupt();
        inputBottlePort[i].close();
    }
    
}


