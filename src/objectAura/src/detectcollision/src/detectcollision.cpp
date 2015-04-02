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


#include <iostream>
#include "iCub/detectcollision.h"
#include <stdio.h>
#include <math.h>


using namespace std;

//*********************//
//For the collision detection ObjectA=1; ObjectB=2; ObjectC=3; Hand =4; Bucked=5; Body = 6
//*********************//


	double AX1;
	double AY1;
	

	double AX2;
	double AY2;
	

	double AX3;
	double AY3;
	

	double BX4;
	double BY4;

	double BX5;
	double BY5;
	    
    double BX6;
	double BY6;

    double AX1t;
	double AY1t;
	

	double AX2t;
	double AY2t;
	

	double AX3t;
	double AY3t;
	

	double BX4t;
	double BY4t;

	double BX5t;
	double BY5t;

    double BX6t;
	double BY6t;

	double objectAX, objectAY;
	double objectBX; 
	double objectBY; 
	double d1;
	double d2;
	double d; 
	double objectauradistance;
	

	double detectcollision::getobjectAX1(){
	return AX1t;
	}
	void detectcollision::setobjectAX1(double AX1){
    AX1t=AX1;
    }
	

	double detectcollision::getobjectAY1(){
	return AY1t;
	}
	void detectcollision::setobjectAY1(double AY1){
    AY1t=AY1;
    }



	//************************************************************	

	double detectcollision::getobjectAX2(){
	return AX2t;
	}
	void detectcollision::setobjectAX2(double AX2){
    AX2t=AX2;    
    }
	

	double detectcollision::getobjectAY2(){
	return AY2t;
	}
	void detectcollision::setobjectAY2(double AY2){
    AY2t=AY2;
    }




	
//**********************************************************

	double detectcollision::getobjectAX3(){
	return AX3t;
	}
	void detectcollision::setobjectAX3(double AX3){
    AX3t=AX3;
    }
	

	double detectcollision::getobjectAY3(){
	return AY3t;
	}
	void detectcollision::setobjectAY3(double AY3){
    AY3t=AY3;    
    }




	//******************************************************


	double detectcollision::getobjectBX4(){
	return BX4t;
	}
	void detectcollision::setobjectBX4(double BX4){
    BX4t=BX4;
    }


	double detectcollision::getobjectBY4(){
	return BY4t;
	}
	void	detectcollision::setobjectBY4(double BY4){
    BY4t=BY4;
    }



	
	//*********************************************************

	double detectcollision::getobjectBX5(){
	return BX5t;
	}
	void detectcollision::setobjectBX5(double BX5){
    BX5t=BX5;
    }


	double detectcollision::getobjectBY5(){
	return BY5t;
	}
	void	detectcollision::setobjectBY5(double BY5){
    BY5t=BY5;
    }


    //*********************************************************

	double detectcollision::getobjectBX6(){
	return BX6t;
	}
	void detectcollision::setobjectBX6(double BX6){
    BX6t=BX6;
    }


	double detectcollision::getobjectBY6(){
	return BY6t;
	}
	void	detectcollision::setobjectBY6(double BY6){
    BY6t=BY6;
    }//=0;
	

	//*******************************************************


	bool detectcollision::collision(int ObjectA, int ObjectB){
			
			

			//detectcollision detcoll;
			if(ObjectA==1){
			objectAX = getobjectAX1();
			objectAY = getobjectAY1();
			objectauradistance=-1;				
			}
			if(ObjectA==2){
			objectAX = getobjectAX2();
			objectAY = getobjectAY2();
			objectauradistance=-1;
			}	
			if(ObjectA==3){
			objectAX = getobjectAX3();
			objectAY = getobjectAY3();
			objectauradistance=-1;
			}		
            if(ObjectA==4){
			objectAX = getobjectBX4();
			objectAY = getobjectBY4();
			objectauradistance=-1;
			}	            
            if(ObjectA==5){
			objectAX = getobjectBX5();
			objectAY = getobjectBY5();
			objectauradistance=-1;
			}	
            if(ObjectA==6){
			objectAX = getobjectBX6();
			objectAY = getobjectBY6();
			objectauradistance=-1;
			}	
			if(ObjectB==4){
			objectBX = getobjectBX4();
			objectBY = getobjectBY4();
			objectauradistance=-1;

			}
			if(ObjectB==5){
			objectBX = getobjectBX5();
			objectBY = getobjectBY5();
			objectauradistance=-1;

			}
            if(ObjectB==6){
			objectBX = getobjectBX6();
			objectBY = getobjectBY6();
			objectauradistance=-1;

			}
			if(ObjectA==1 && ObjectB==4){
			objectauradistance = 50;
			} 
			if(ObjectA==1 && ObjectB==5){
			objectauradistance = 150;
			} 
			if(ObjectA==2 && ObjectB==4){
			objectauradistance = 50;
			} 
			if(ObjectA==2 && ObjectB==5){
			objectauradistance = 150;
			} 
			if(ObjectA==3 && ObjectB==4){
			objectauradistance = 50;
			} 
			if(ObjectA==3 && ObjectB==5){
			objectauradistance = 150;
			} 
			if(ObjectA==4 && ObjectB==6){
			objectauradistance = 230;
			} 
			if(ObjectA==1 && ObjectB==6){
			objectauradistance = 230;
			} 
			if(ObjectA==2 && ObjectB==6){
			objectauradistance = 230;
			} 
            if(ObjectA==3 && ObjectB==6){
			objectauradistance = 230;
			} 
		    if(objectBX != -1.0 && objectAX != -1.0 && objectBY != -1.0 && objectAY != -1.0){
	        d1 = (objectBX-objectAX)*(objectBX-objectAX);
	        d2 = (objectBY-objectAY)*(objectBY-objectAY);
	        d = sqrt(d1+d2);
            objectBX=0;
            objectBY=0;
            objectAX=0;
            objectAY=0;
            }
            if((objectBX == -1.0 && objectBY == -1.0 ) || (objectAX == -1.0 && objectAY == -1.0)){   
            d=1000.0;
            objectauradistance = -1;
            objectBX=0;
            objectBY=0;
            objectAX=0;
            objectAY=0;
            //return false;
            }
			
			//printf ("Collision detection is running.\n");
            //printf ("Distance of ObjectA:  %i ObjectB: %i \n", ObjectA, ObjectB);
			//printf ("Distance XY:  %4.2f  %4.2f \n", d1, d2);
			//printf ("Distance:  %4.2f \n", d);

			if(d< objectauradistance && d >0){
            printf ("Collision detectio ObjectA:  %i ObjectB: %i \n", ObjectA, ObjectB);
            objectBX=0;
            objectBY=0;
            objectAX=0;
            objectAY=0;
			return true;
			}else{
            objectBX=0;
            objectBY=0;
            objectAX=0;
            objectAY=0;
			return false;
			}

			

	}

detectcollision::detectcollision(){

}

void detectcollision::init( ){
//collision(1, 4, detcoll);
    }

/*int main() 
{

		detectcollision detcoll;
		detcoll.collision(1,4);
			
			
		return 0;
    }*/


