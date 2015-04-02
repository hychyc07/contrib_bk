// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Katrin Lohan
  * email: katrin.lohan@iit.it
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


#include "iCub/InterfaceThread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;


//#define Bucketx 174.7
//#define Buckety 102.8
//40 364 245 500
//40 349 245 485

#define Bucketx 140.0
#define Buckety 432.0

#define Bodyx 525.0
#define Bodyy 350.0

anticipation* anti;
detectcollision* detcoll;
objecthandler* objecth;

InterfaceThread::InterfaceThread(BufferedPort<Bottle> *port1, BufferedPort<Bottle> *port2, BufferedPort<Bottle> *port3, BufferedPort<Bottle> *port4, BufferedPort<Bottle> *outport, BufferedPort<Bottle> *outport2)
{
    this->port1 = port1;    
    this->port2 = port2;
    this->port3 = port3;
    this->port4 = port4;
    this->outport = outport;
    this->outport2 = outport2;
}

bool InterfaceThread::threadInit() 
{
    oA =true;
    oB =false;
    oC =false;   
    anti = new anticipation();
    anti->init();
    detcoll = new detectcollision();
    detcoll->init();
    objecth = new objecthandler();
    objecth->init();
    endoftrail=true;
   return true;
}

void InterfaceThread::bodyBottle(Bottle *output, Bottle *output2){

    output->addString("fix");
    output->addDouble(Bodyx);
    output->addDouble(Bodyy);  
    output2->addString("fix");
    output2->addString("Body");
    output2->addDouble(Bodyx);
    output2->addDouble(Bodyy);
}

void InterfaceThread::bucketBottle(Bottle *output, Bottle *output2){

    output->addString("fix");
    output->addDouble(detcoll->getobjectBX5());
    output->addDouble(detcoll->getobjectBY5());
        //output->addString("Track Hand no collision");    
        output2->addString("fix");
        output2->addString("Bucket");
        output2->addDouble(detcoll->getobjectBX5());
        output2->addDouble(detcoll->getobjectBY5());

    
}

void InterfaceThread::handBottle(Bottle *output, Bottle *output2){

    output->addString("fix");
    output->addDouble(detcoll->getobjectBX4());
    output->addDouble(detcoll->getobjectBY4()); 
    output2->addString("fix");
    output2->addString("Hand");
    output2->addDouble(detcoll->getobjectBX4());
    output2->addDouble(detcoll->getobjectBY4());
}

void InterfaceThread::trackBallOneBottle(Bottle *output, Bottle *output2){
    output->addString("track");
    output->addString("red");
    cout<< "Bottle:" << endl;
    cout<< output->toString().c_str() << endl;
    output2->addString("track");
    output2->addString("red");
    output2->addString("Ball");
    output2->addDouble(detcoll->getobjectAX1());
    output2->addDouble(detcoll->getobjectAY1());
    output2->addString("Hand");
    output2->addDouble(detcoll->getobjectBX4());
    output2->addDouble(detcoll->getobjectBY4());
    cout<< "Bottle:" << endl;
    cout<< output2->toString().c_str() << endl;
}

void InterfaceThread::trackBallTwoBottle(Bottle *output, Bottle *output2){
     output->addString("track");
    output->addString("green");
    cout<< "Bottle:" << endl;
    cout<< output->toString().c_str() << endl;
    output2->addString("track");
    output2->addString("green");
    output2->addString("Ball");
    output2->addDouble(detcoll->getobjectAX2());
    output2->addDouble(detcoll->getobjectAY2());
    output2->addString("Hand");
    output2->addDouble(detcoll->getobjectBX4());
    output2->addDouble(detcoll->getobjectBY4());
    cout<< "Bottle:" << endl;
    cout<< output2->toString().c_str() << endl;
}

void InterfaceThread::trackBallThreeBottle(Bottle *output, Bottle *output2){
     output->addString("track");
    output->addString("purple");
    cout<< "Bottle:" << endl;
    cout<< output->toString().c_str() << endl;
    output2->addString("track");
    output2->addString("purple");
    output2->addString("Ball");
    output2->addDouble(detcoll->getobjectAX3());
    output2->addDouble(detcoll->getobjectAY3());
    output2->addString("Hand");
    output2->addDouble(detcoll->getobjectBX4());
    output2->addDouble(detcoll->getobjectBY4());
    cout<< "Bottle:" << endl;
    cout<< output2->toString().c_str() << endl;
}

void InterfaceThread::unfixedBottle(Bottle *output, Bottle *output2){

        output->addString("unfixed");
        output2->addString("unfixed");
}

void InterfaceThread::startBottle(Bottle *output, Bottle *output2){

        output->addString("Start");
        output2->addString("Start");
}

void InterfaceThread::run(){
  

while(endoftrail) {
    
    int j,l,k; 
      
            
	while(oA){
    //sleep(3);
    bool first = false;       
    //ObjectA
    Bottle* posTrack = port1->read(true);
    if(strcmp (posTrack->get(0).asString(),"Start") != 0){ 
    detcoll->setobjectAX1(posTrack->get(0).asDouble());
    detcoll->setobjectAY1(posTrack->get(1).asDouble());
    }else{
    first = true;
    }
    //Hand    
    posTrack = port4->read(true);
    if(strcmp (posTrack->get(0).asString(),"Start") != 0){ 
    detcoll->setobjectBX4(posTrack->get(0).asDouble());
    detcoll->setobjectBY4(posTrack->get(1).asDouble());
    }else{
    first = true;
    }
    //Bucket
    detcoll->setobjectBX5(Bucketx);
    detcoll->setobjectBY5(Buckety);
    //Body
    detcoll->setobjectBX6(Bodyx);
    detcoll->setobjectBY6(Bodyy);
    //
     //Output
    Bottle *output = &outport->prepare();
    //Output
    Bottle *output2 = &outport2->prepare();
    output->clear();
        output2->clear();
	j = objecth->objectA(anti, detcoll);
    cout<< j << endl;
        if(!first){
        if(j==0){
        printf ("No Collision ObjectA.\n");        
        bodyBottle(output,output2);
        output2->addInt(j);
        oA=true;
        oB=false;
        oC=false;
        }
    	if( j==1 ) {
        printf ("reaching ObjectA\n");
        handBottle(output,output2);
        output2->addInt(j);
        oA=true;
        oB=false;
        oC=false;
        }
        if(j==2 ) {
        printf ("Transport ObjectA\n");
         trackBallOneBottle(output,output2); 
        output2->addInt(j);
         oA=true;
        oB=false;
        oC=false;
        }
    	if( j==3) {
        printf ("Ende ObjectA next Object\n");
        bucketBottle(output,output2);
        output2->addInt(j);
        oA=false;
        oB=true;
        oC=false;
        }
        if(j==4) {
        printf ("in Transport ObjectA\n");
        bodyBottle(output,output2);
        output2->addInt(j);
        oA=true;
        oB=false;
        oC=false;
        }
        if(j==5) {
        printf ("in reaching ObjectA\n");
        trackBallOneBottle(output,output2);
        output2->addInt(j);
        oA=true;
        oB=false;
        oC=false;
        }
        if(j==6){
       unfixedBottle(output,output2);
       output2->addInt(j);
         oA=true;
        oB=false;
        oC=false;
        }
        }else{
        startBottle(output,output2);
        output2->addInt(j);
        /*oA=true;
        oB=false;
        oC=false;*/
        }
        outport2->writeStrict();
        outport->writeStrict();
	}

    
	while(oB){
    //sleep(3);
    bool first = false;       
    //ObjectA
    Bottle* posTrack = port2->read(true);
    if(strcmp (posTrack->get(0).asString(),"Start") != 0){ 
    detcoll->setobjectAX2(posTrack->get(0).asDouble());
    detcoll->setobjectAY2(posTrack->get(1).asDouble());
    }else{
    first = true;
    }
    //Hand    
    posTrack = port4->read(true);
    if(strcmp (posTrack->get(0).asString(),"Start") != 0){ 
    detcoll->setobjectBX4(posTrack->get(0).asDouble());
    detcoll->setobjectBY4(posTrack->get(1).asDouble());
    }else{
    first = true;
    }
    //Bucket
    detcoll->setobjectBX5(Bucketx);
    detcoll->setobjectBY5(Buckety);
    //Body
    detcoll->setobjectBX6(Bodyx);
    detcoll->setobjectBY6(Bodyy);
    //
     //Output
    Bottle *output = &outport->prepare();
    //Output
    Bottle *output2 = &outport2->prepare();
    output->clear();
        output2->clear();
	k = objecth->objectB(anti, detcoll);
    cout<< k << endl;
        if(!first){
        if(k==0){
        printf ("No Collision ObjectA.\n");        
        handBottle(output,output2);
        output2->addInt(k);
        oA=false;
        oB=true;
        oC=false;
        }
    	if( k==1 ) {
        printf ("reaching ObjectA\n");
        handBottle(output,output2);
        output2->addInt(k);
        oA=false;
        oB=true;
        oC=false;
        }
        if(k==2 ) {
        printf ("Transport ObjectA\n");
         trackBallTwoBottle(output,output2); 
        output2->addInt(k);
         oA=false;
        oB=true;
        oC=false;
        }
    	if( k==3) {
        printf ("Ende ObjectA next Object\n");
        bucketBottle(output,output2);
        output2->addInt(k);
        oA=false;
        oB=false;
        oC=true;
        }
        if(k==4) {
        printf ("in Transport ObjectA\n");
        bodyBottle(output,output2);
        output2->addInt(k);
        oA=false;
        oB=true;
        oC=false;
        }
        if(k==5) {
        printf ("in reaching ObjectA\n");
        trackBallTwoBottle(output,output2);
        output2->addInt(k);
        oA=false;
        oB=true;
        oC=false;
        }
        if(k==6){
       unfixedBottle(output,output2);
       output2->addInt(k);
         oA=false;
        oB=true;
        oC=false;
        }
        }else{
        startBottle(output,output2);
        output2->addInt(k);
        /*oA=true;
        oB=false;
        oC=false;*/
        }
        outport2->writeStrict();
        outport->writeStrict();
        }


	while(oC){
    //sleep(3);
    bool first = false;       
    //ObjectA
    Bottle* posTrack = port3->read(true);
    if(strcmp (posTrack->get(0).asString(),"Start") != 0){ 
    detcoll->setobjectAX3(posTrack->get(0).asDouble());
    detcoll->setobjectAY3(posTrack->get(1).asDouble());
    }else{
    first = true;
    }
    //Hand    
    posTrack = port4->read(true);
    if(strcmp (posTrack->get(0).asString(),"Start") != 0){ 
    detcoll->setobjectBX4(posTrack->get(0).asDouble());
    detcoll->setobjectBY4(posTrack->get(1).asDouble());
    }else{
    first = true;
    }
    //Bucket
    detcoll->setobjectBX5(Bucketx);
    detcoll->setobjectBY5(Buckety);
    //Body
    detcoll->setobjectBX6(Bodyx);
    detcoll->setobjectBY6(Bodyy);
    //
     //Output
    Bottle *output = &outport->prepare();
    //Output
    Bottle *output2 = &outport2->prepare();
    output->clear();
        output2->clear();
	l = objecth->objectC(anti, detcoll);
    cout<< l << endl;
        if(!first){
        if(l==0){
        printf ("No Collision ObjectA.\n");        
        handBottle(output,output2);
        output2->addInt(l);
        oA=false;
        oB=false;
        oC=true;
        }
    	if( l==1 ) {
        printf ("reaching ObjectA\n");
        handBottle(output,output2);
        output2->addInt(l);
        oA=false;
        oB=false;
        oC=true;
        }
        if(l==2 ) {
        printf ("Transport ObjectA\n");
         trackBallThreeBottle(output,output2); 
        output2->addInt(l);
         oA=false;
        oB=false;
        oC=true;
        }
    	if( l==3) {
        printf ("Ende ObjectA next Object\n");
        bucketBottle(output,output2);
        output2->addInt(l);
        oA=false;
        oB=false;
        oC=false;
        endoftrail=false;
        }
        if(l==4) {
        printf ("in Transport ObjectA\n");
        bodyBottle(output,output2);
        output2->addInt(l);
        oA=false;
        oB=false;
        oC=true;
        }
        if(l==5) {
        printf ("in reaching ObjectA\n");
        trackBallThreeBottle(output,output2);
        output2->addInt(l);
        oA=false;
        oB=false;
        oC=true;
        }
        if(l==6){
       unfixedBottle(output,output2);
       output2->addInt(l);
         oA=false;
        oB=false;
        oC=true;
        }
        }else{
        startBottle(output,output2);
        output2->addInt(l);
        /*oA=true;
        oB=false;
        oC=false;*/
        }
        outport2->writeStrict();
        outport->writeStrict();
   }            
    }
}
                
            


void InterfaceThread::threadRelease() 
{
   /* for example, delete dynamically created data-structures */
}


