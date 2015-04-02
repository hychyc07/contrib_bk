// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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
 * @file detectorThread.cpp
 * @brief Implementation of the eventDriven thread (see detectorThread.h).
 */

#include <detectorThread.h>


using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 100 //ms

detectorThread::detectorThread():Thread() {
    robot = "icub"; 
    firstRun = true;
}

detectorThread::detectorThread(string _robot, string _configFile):Thread(){
    robot = _robot;
    configFile = _configFile;
    firstRun = true;
    
}

detectorThread::~detectorThread() {
    // do nothing
}

bool detectorThread::threadInit() {

    //para_yml_file = "data_blocks/para_blocks.yml";
    
    printf("\n trying to open ports from %s \n", getName("/img:i").c_str());
    imageInputPort = new BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >;
    
    
    if (!imageInputPort->open(getName("/img:i").c_str())) {
        cout << ": unable to open input image port  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    

    imageOutputPort = new BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >;
    if (!imageOutputPort->open(getName("/img:o").c_str())) {
        cout << ": unable to open port output image port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    
    if (!dataPortMec.open(getName("/shapeData:o").c_str())) {
        cout << ": unable to open port for the stream data out "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    /////////////////////////////////////////////////
	// STEP1: initiate
	/////////////////////////////////////////////////
    bool flag;
	detector = new CTLdetector();
    printf("trying to upload paraFile from %s \n", para_yml_file.c_str());
	flag = detector->initiate(para_yml_file);
	if (!flag){		
        cout<< "detector initialization failed "<<endl;
        return false;
    }
    else {
        cout<<" detector initialization success"<<endl;
    }

    /////////////////////////////////////////////////
	// STEP2: train
	/////////////////////////////////////////////////
    printf("trying to upload training file from %s \n", tr_bin_file.c_str());
    detector->setTraining(tr_bin_file);
	flag = detector->train();
	if (!flag) {
        cout<< "detector training failed "<<endl;
		return false;
    }
    else {
        cout<< "detector training success"<<endl;
    }

    return true;
    

}

void detectorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string detectorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void detectorThread::setInputPortName(string InpPort) {
    
}

void detectorThread::run() {
    cout<< "Detecting mode active.... \n"<<endl;
    cv::Mat cvMatImage;
    int key = 0;
    

    while (isStopping() != true) {
        
        if (imageInputPort->getInputCount()) {
            ImageOf<PixelRgb> *img = imageInputPort->read(true);
            
            if(firstRun) {
                cvMatImage.create(img->height(), img->width(), CV_8UC3);
                firstRun = false;
            }
            
            double start = Time::now();
            unsigned char* ptr = img->getRawImage();
            memcpy(cvMatImage.data, ptr, cvMatImage.cols * cvMatImage.rows * 3);
            cv::cvtColor(cvMatImage, cvMatImage, CV_RGB2BGR);
            
            detector->detect(cvMatImage);
            //detector->showDetObjs(cvMatImage,Scalar(0,255,0),Scalar(255,255,255),480);  //chaged 3.2.2013 Rea changed to orinal size
            detector->showDetObjs(cvMatImage,Scalar(0,255,0),Scalar(255,255,255),0);
            //detector->dispDetObjs();
            std::vector<DetObj> objects;
            objects = detector->getDetObjs();

            double end = Time::now();
            
            double interval = end - start;
            printf("processing time interval %f sec \n", interval);

            //key = cv::waitKey(30);

            //============================================
            //sending out through image port the image out    
            //============================================
            
            if(imageOutputPort->getOutputCount()) {
                cv::Mat ppIm = detector->getPostProcessIm();            
                cv::cvtColor(ppIm, ppIm, CV_BGR2RGB);
                
                // printf("image %d %d \n", cvMatImage.cols, cvMatImage.rows);
                ImageOf<PixelRgb>& tmpImage = imageOutputPort->prepare();  
                tmpImage.resize(img->width(),img->height());
                tmpImage.zero();
                unsigned char* ptrd = tmpImage.getRawImage();
                unsigned char* ptrs = ppIm.data;
                int padding         = tmpImage.getPadding();
                
                for (int row = 0; row <  img->height(); row++) {
                    for(int col = 0; col < img->width(); col++) {
                        *ptrd = *ptrs;
                        ptrd++; 
                        ptrs++;
                        *ptrd = *ptrs;
                        ptrd++; 
                        ptrs++;
                        *ptrd = *ptrs;
                        ptrd++; 
                        ptrs++;
                            
                    }
                    ptrd += padding;
                    
                }
                
                    //memcpy(tmpImage.getRawImage(), ppIm.data, cvMatImage.cols * cvMatImage.rows * 3);
                //memcpy(ptrd,ptrs, cvMatImage.cols * cvMatImage.rows * 3 );
                imageOutputPort->write();
            }
            
            //====================================================
            //sending featureas of detected objects on the bottle    
            //====================================================
            
            if(dataPortMec.getOutputCount()) {
                Bottle& ShapOp = dataPortMec.prepare();
                ShapOp.clear();
                ShapOp.addInt(objects.size());
                
                Bottle objectBottle;

                for (int i = 0; i < objects.size(); i++){
                    Bottle& objectBottle = ShapOp.addList();
                    objectBottle.clear();
                    objectBottle.addInt(objects[i].box_tight.x);
                    objectBottle.addInt(objects[i].box_tight.y);
                    objectBottle.addInt(objects[i].box_tight.width);
                    objectBottle.addInt(objects[i].box_tight.height);
                    objectBottle.addInt(objects[i].id_label);
                    
                    //ShapOp.addInt(10);
                    //if want to know the object name: detector->all_obj_cls[objects[i].id_label]
                }
                
                // free the memory of the objects.
                for (int i=0;i<objects.size();i++){
                    objects[i].mat_edge_NN_tr.release();
                }
                objects.clear();
                //bbOutputPort->write();
                dataPortMec.write();
            }            
        }
    }

    cvMatImage.release();
	std::cout<<"***Done."<<std::endl;
                  
}

void detectorThread::onStop(){
    // code for the stopping session in detector Thread
    printf("detectorThread::onStop() \n");
    imageInputPort->close();
    imageOutputPort->close();    
    dataPortMec.close();
    printf("detectorThread::onStop() end! \n");
      
}

void detectorThread::threadRelease() {
    // nothing
     
}


