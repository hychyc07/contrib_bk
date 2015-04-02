// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -

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
 * @file main.h
 * @brief definition of the module that creates a structure from scratch
 */

#include <iCub/MachineBoltzmann.h>

#define SAVE



/**
* The main is called just in the testing phase of the library
* An application can be created in this project in order to test the library
*/


int main(int argc, char *argv[]) {
#ifdef LOAD 	
    MachineBoltzmann *mb=new MachineBoltzmann();
    cout<<"Loading configuration from file"<<endl;
    mb->loadConfiguration();		
#else
    MachineBoltzmann *mb=new MachineBoltzmann(2);
    Layer *layerA=new Layer("LA",10,10);
    printf("created Layer0 \n");
    mb->addLayer(layerA);
    printf("creating Layer1 \n");
    Layer *layerB=new Layer("LB",10,10);
    mb->addLayer(layerB);
    mb->interconnectLayers(layerA,layerB);

    int numhid=4*12;
    Matrix data(1,12);
    data(0,0)=0;
    data(0,1)=1;
    data(0,2)=10;
    data(0,3)=11;
    data(0,4)=20;
    data(0,5)=21;
    data(0,6)=30;
    data(0,7)=31;
    data(0,8)=40;
    data(0,9)=41;
    data(0,10)=50;
    data(0,11)=51;
    Vector v(100);
    for (int i=0;i<100;i++){
        v[i]=0.1;
    }
    mb->addSample(v);
    mb->addSample(v);
    mb->addSample(v);
    mb->addSample(v);
    mb->addSample(v);
    mb->addSample(v);
    mb->addSample(v);
    mb->addSample(v);
    mb->addSample(v);
    mb->addSample(v);


    //mb->rbm((Matrix)data,layerA,numhid);
    //mb->migrateLayer(*layer);
    //mb->interconnectLayer(2);
    //mb->evolveClamped(2,1);
    //mb->saveConfiguration();

    /*mb->addClampedUnit(153,1);
    mb->addClampedUnit(13,1);
    mb->addClampedUnit(14,1);
    for(int i=0;i<150;i++)
        mb->evolveFreely(2,1);
    for(int i=0;i<150;i++)
        mb->evolveClamped(2,1);
    for(int i=0;i<1000;i++){
        mb->setProbabilityFreely();
        mb->setProbabilityClamped();
    }
    mb->learn();*/
    //getch();
#endif
    
    return 0;
}
