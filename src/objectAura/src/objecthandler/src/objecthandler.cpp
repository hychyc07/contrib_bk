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


#include "iCub/objecthandler.h"


using namespace std; 

//*********************//
//For the collision detection ObjectA=1; ObjectB=2; ObjectC=3; Hand =4; Bucket=5; Body = 6
//*********************//



void objecthandler::init(){}

objecthandler::objecthandler(){
oAtransport = false;
oAreaching = false;
oBtransport = false;
oBreaching = false;
oCtransport = false;
oCreaching = false;
}

int objecthandler::objectA (anticipation* anti, detectcollision* detcoll) {
	
	int i = -1;
    int count =0;
    bool stateipointAH =false;
    bool stateipointABu =false;
	bool stateipointABo =false;
    bool stateipointHBo =false;



	bool ipointAH = detcoll->collision(1,4);
	if (ipointAH){
        stateipointAH = true;//"ObjectA"
    }else{
        stateipointAH = false;//"Hand"
	}
    
	bool ipointABu = detcoll->collision(1,5);
	if (ipointABu) {	
        stateipointABu = true;//"Bucket";	
    }else{
        stateipointABu = false;//"ObjectA";
    }

    bool ipointABo = detcoll->collision(1,6);
	if (ipointABo) {
        stateipointABo = true;//"Body";
    }else{
        stateipointABo = false;//"ObjectA";
    }
    
    bool ipointHBo = detcoll->collision(4,6);
	if (ipointHBo) {
        stateipointHBo=true;//"Body";
    }else{
        stateipointHBo=false;//"Hand";
    }

    if (!stateipointAH && !stateipointABu && !stateipointABo && !stateipointHBo){
    i =0;
    anti->anticipate(detcoll->getobjectBX6(), detcoll->getobjectBY6());
    }//no Collision Start

    if (!stateipointAH && !stateipointABu && !stateipointABo && stateipointHBo ){
    i= 1;
    anti->anticipate(detcoll->getobjectBX4(), detcoll->getobjectBY4());
        oAreaching = true;
    }//reaching

    if (stateipointAH && !stateipointABo && !stateipointHBo && !stateipointABu){
    i=2;
    anti->anticipate(detcoll->getobjectAX1(), detcoll->getobjectAY1());
        oAtransport=true;
        oAreaching = false;
    }//Transport

     if (stateipointABo && stateipointAH && stateipointHBo && !stateipointABu  && oAtransport){
    i=4;
    anti->anticipate(detcoll->getobjectBX6(), detcoll->getobjectBY6());
    }//in Transport
    if (stateipointABu){
    i=3;
    anti->anticipate(detcoll->getobjectBX5(), detcoll->getobjectBY5());
         
    }//End next Object
    if (!stateipointABo && !stateipointAH && !stateipointABu && !stateipointHBo && oAreaching){
    i=5;
    anti->anticipate(detcoll->getobjectAX1(), detcoll->getobjectAY1());
    }//in reaching
    if (i == -1){
    i=6;
    }
	return i;	
}


int objecthandler::objectB (anticipation* anti, detectcollision* detcoll) {

	int i = -1;
    int count =0;
    bool stateipointAH =false;
    bool stateipointABu =false;
	bool stateipointABo =false;
    bool stateipointHBo =false;



	bool ipointAH = detcoll->collision(2,4);
	if (ipointAH){
        stateipointAH = true;//"ObjectA"
    }else{
        stateipointAH = false;//"Hand"
	}
    
	bool ipointABu = detcoll->collision(2,5);
	if (ipointABu) {	
        stateipointABu = true;//"Bucket";	
    }else{
        stateipointABu = false;//"ObjectA";
    }

    bool ipointABo = detcoll->collision(2,6);
	if (ipointABo) {
        stateipointABo = true;//"Body";
    }else{
        stateipointABo = false;//"ObjectA";
    }
    
    bool ipointHBo = detcoll->collision(4,6);
	if (ipointHBo) {
        stateipointHBo=true;//"Body";
    }else{
        stateipointHBo=false;//"Hand";
    }

    if (!stateipointAH && !stateipointABu && !stateipointABo && !stateipointHBo){
    i =0;
    anti->anticipate(detcoll->getobjectBX4(), detcoll->getobjectBY4());
    }//no Collision Start

    if (!stateipointAH && !stateipointABu && !stateipointABo && stateipointHBo ){
    i= 1;
    anti->anticipate(detcoll->getobjectBX4(), detcoll->getobjectBY4());
        oBreaching = true;
    }//reaching

    if (stateipointAH && !stateipointABo && !stateipointHBo && !stateipointABu){
    i=2;
    anti->anticipate(detcoll->getobjectAX1(), detcoll->getobjectAY1());
        oBtransport=true;
        oBreaching = false;
    }//Transport

     if (stateipointABo && stateipointAH && stateipointHBo && !stateipointABu  && oAtransport){
    i=4;
    anti->anticipate(detcoll->getobjectBX6(), detcoll->getobjectBY6());
    }//in Transport
    if (stateipointABu){
    i=3;
    anti->anticipate(detcoll->getobjectBX5(), detcoll->getobjectBY5());
         
    }//End next Object
    if (!stateipointABo && !stateipointAH && !stateipointABu && !stateipointHBo && oAreaching){
    i=5;
    anti->anticipate(detcoll->getobjectAX1(), detcoll->getobjectAY1());
    }//in reaching
    if (i == -1){
    i=6;
    }
	return i;	
}
int objecthandler::objectC (anticipation* anti, detectcollision* detcoll) {
	
	int i = -1;
    int count =0;
    bool stateipointAH =false;
    bool stateipointABu =false;
	bool stateipointABo =false;
    bool stateipointHBo =false;



	bool ipointAH = detcoll->collision(3,4);
	if (ipointAH){
        stateipointAH = true;//"ObjectA"
    }else{
        stateipointAH = false;//"Hand"
	}
    
	bool ipointABu = detcoll->collision(3,5);
	if (ipointABu) {	
        stateipointABu = true;//"Bucket";	
    }else{
        stateipointABu = false;//"ObjectA";
    }

    bool ipointABo = detcoll->collision(3,6);
	if (ipointABo) {
        stateipointABo = true;//"Body";
    }else{
        stateipointABo = false;//"ObjectA";
    }
    
    bool ipointHBo = detcoll->collision(4,6);
	if (ipointHBo) {
        stateipointHBo=true;//"Body";
    }else{
        stateipointHBo=false;//"Hand";
    }

    if (!stateipointAH && !stateipointABu && !stateipointABo && !stateipointHBo){
    i =0;
    anti->anticipate(detcoll->getobjectBX4(), detcoll->getobjectBY4());
    }//no Collision Start

    if (!stateipointAH && !stateipointABu && !stateipointABo && stateipointHBo ){
    i= 1;
    anti->anticipate(detcoll->getobjectBX4(), detcoll->getobjectBY4());
        oCreaching = true;
    }//reaching

    if (stateipointAH && !stateipointABo && !stateipointHBo && !stateipointABu){
    i=2;
    anti->anticipate(detcoll->getobjectAX1(), detcoll->getobjectAY1());
        oCtransport=true;
        oCreaching = false;
    }//Transport

     if (stateipointABo && stateipointAH && stateipointHBo && !stateipointABu  && oAtransport){
    i=4;
    anti->anticipate(detcoll->getobjectBX6(), detcoll->getobjectBY6());
    }//in Transport
    if (stateipointABu){
    i=3;
    anti->anticipate(detcoll->getobjectBX5(), detcoll->getobjectBY5());
         
    }//End next Object
    if (!stateipointABo && !stateipointAH && !stateipointABu && !stateipointHBo && oAreaching){
    i=5;
    anti->anticipate(detcoll->getobjectAX1(), detcoll->getobjectAY1());
    }//in reaching
    if (i == -1){
    i=6;
    }
	return i;	
}



