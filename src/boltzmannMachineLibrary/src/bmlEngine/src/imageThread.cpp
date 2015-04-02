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
 * @file imageThread.cpp
 * @brief main code for bulding and sending images ( see imageThread.h )
 */


#include <iCub/imageThread.h>
#include <yarp/math/Math.h>

/**
* initialise the thread
*/
bool imageThread::threadInit(){
    scaleFactorX=4;
    scaleFactorY=4;
    
    //cvImage= cvCreateImage(cvSize(sizeX,sizeY), IPL_DEPTH_8U, 3 );
    //image2=new ImageOf<PixelRgb>;
    //image2->resize(sizeX,sizeY);
    //sizeX=plottedLayer->getColWeights()*scaleFactorX;
    //sizeY=plottedLayer->getRowWeights()*scaleFactorX;
    //imageWeights=new ImageOf<PixelRgb>;
    //imageWeights->resize(sizeX,sizeY);
    imageWeights=0;
    /*
    for (int x=0; x<sizeX; x++){
        for(int y=0;y<sizeY;y++){
            PixelRgb &pix=image2->pixel(x,y);
            //PixelRgb &pixW=imageWeights->pixel(x,y);
            pix.r=0;
            pix.b=0;
            pix.g=0;
        }
    }
    */
    printf("Image Thread initialising..... \n");
    printf("opening image port..... \n");
    string str=getName();
    str.append("/state:o");
    statePort.open((const char*)str.c_str());
    str=getName();
    str.append("/weight:o");
    weightPort.open((const char*)str.c_str());
    return true;
}

/**
* code that is executed after the thread starts
* @param s is true if the thread started
*/
void imageThread::afterStart(bool s){
    if(s){
        printf("Image Thread after start.....\n");
    }

}

/**
* running code of the thread
*/
void imageThread::run(){
    //1. produces the image of the state
    if(statePort.getOutputCount()) {
        ImageOf<PixelRgb>& image2=statePort.prepare();
        int sizeX=plottedLayer->getCol()*scaleFactorX;
        int sizeY=plottedLayer->getRow()*scaleFactorY;
        image2.resize(sizeX,sizeY);
        Vector v=*(plottedLayer->stateVector);
        for(int i=0;i<plottedLayer->getCol();i++) {
            for(int j=0;j<plottedLayer->getRow();j++) {
                int pos=j*plottedLayer->getCol()+i;
                //printf("%d %f \n",pos,v(pos));
                for (int scaleX=0; scaleX < scaleFactorX; scaleX++) {
                    for(int scaleY=0;scaleY < scaleFactorY;scaleY++) {
                        PixelRgb &pix=image2.pixel(i*scaleFactorX+scaleX,j*scaleFactorY+scaleY);
                        pix.r=v(pos)*255;
                        pix.b=v(pos)*255;
                        pix.g=v(pos)*255;
                    }
                }
            }
        }
        /*ImageOf<PixelRgb>& imageWeights=weightPort.prepare();
        imageWeights.resize(sizeX,sizeY);
        cvCopy(image2.getIplImage(),imageWeights.getIplImage());
        weightPort.write();
        */
        statePort.write();
    }
    //2. preduces the image of the weights to the upper level
    /*
    if(weightPort.getOutputCount()) {
        ImageOf<PixelRgb>& imageWeights=weightPort.prepare();
        int sizeXweight=plottedLayer->getColWeights()*scaleFactorX;
        int sizeYweight=plottedLayer->getRowWeights()*scaleFactorY;
        if((sizeXweight!=0)&&(sizeYweight!=0)) {
            imageWeights.resize(sizeXweight,sizeYweight);
            if(0!=plottedLayer->vishid){
                Matrix matVisHid=*(plottedLayer->vishid);
                double* w= matVisHid.data();
                for(int i=0;i<plottedLayer->getColWeights();i++) {
                    for(int j=0;j<plottedLayer->getRowWeights();j++) {
                        int pos=j*plottedLayer->getColWeights()+i;
                        for (int scaleX=0; scaleX < scaleFactorX; scaleX++) {
                            for(int scaleY=0; scaleY < scaleFactorY; scaleY++) {
                                PixelRgb &pix=imageWeights.pixel(i*scaleFactorX+scaleX,j*scaleFactorY+scaleY);
                                pix.r=matVisHid(j,i)*255;
                                pix.b=matVisHid(j,i)*255;
                                pix.g=matVisHid(j,i)*255;
                            }
                        }
                    }
                }
             }
            weightPort.write();
        }
    }
    */

    //2. produces images as collection of features
    int nFeaturesRow = 2;
    int nFeaturesCol = 2;
    if((weightPort.getOutputCount())&&(0!=plottedLayer->vishid)) {
        Matrix matVisHid=*(plottedLayer->vishid);
        int featureDimX = (int)sqrt((double)matVisHid.cols());
        int featureDimY = (int)sqrt((double)matVisHid.cols());
        ImageOf<PixelRgb>& imageWeights=weightPort.prepare();
        int sizeXweight=featureDimX * nFeaturesCol;
        int sizeYweight=featureDimY * nFeaturesRow;
        imageWeights.resize(sizeXweight,sizeYweight);
        for(int ir=0;ir<nFeaturesRow;ir++) {
            for(int ic=0;ic<nFeaturesCol;ic++) {
                int index= yarp::math::Rand::scalar() * matVisHid.cols();
                Vector column=matVisHid.getCol(index);
                double* w= column.data();
                unsigned char* pimage=imageWeights.getRawImage();
                int rowSize = imageWeights.getRowSize();
                int padding = imageWeights.getPadding();
                pimage += ir * featureDimY * rowSize + ic * featureDimX * 3;
                int maxkr=(int)sqrt((double)column.length());
                int maxkc=(int)sqrt((double)column.length());
                double ww;
                for (int kr=0; kr < maxkr; kr++) {
                    for(int kc=0; kc < maxkc; kc++) {
                        if(*w < 0) {
                            ww = *w * -1;
                        }
                        else {
                            ww = *w;
                        }
                        *pimage =(unsigned char) (ww * 255 * 10);
                        pimage++;
                        *pimage =(unsigned char) (ww * 255 * 10);
                        pimage++;
                        *pimage =(unsigned char) (ww * 255 * 10);
                        pimage++;
                        w++;
                    }
                    pimage += rowSize - maxkc * 3;
                }
            }
        }
        weightPort.write();
    }
}

void imageThread::setLayer(Layer* layer){
    this->plottedLayer=layer;
}

/**
* code executed when the thread is released
*/
void imageThread::threadRelease(){
    printf("Image Thread releasing..... \n");
    statePort.close();
    weightPort.close();
}

ImageOf<PixelRgb>* imageThread::getYarpImage(){
    return image2;
}

void imageThread::setName(string n){
    name=n.substr(0,n.size());
}

string imageThread::getName() {
    return name;
}
