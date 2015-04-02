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
#include "iCub/detectcollision3D.h"
#include <stdio.h>


using namespace std;

//*********************//
//For the collision detection ObjectA=1; ObjectB=2; ObjectC=3; Hand =4; Bucked=5; Body = 6
//*********************//


	double AX1 =1.0;
	double AY1 =1.0;
	double AZ1 =2.0;
	double ANormX1 =1.0;
	double ANormY1 =2.0;
	double ANormZ1 =1.0;

	double AX2;
	double AY2;
	double AZ2;
	double ANormX2;
	double ANormY2;
	double ANormZ2;

	double AX3;
	double AY3;
	double AZ3;
	double ANormX3;
	double ANormY3;
	double ANormZ3;

	double BX4 =1.0;
	double BY4 =2.0;
	double BZ4 =2.0;

	double BX5;
	double BY5;
	double BZ5;
	




	double intersectionPointX, intersectionPointY, intersectionPointZ;
	double lambda;
	double objectAX, objectAY, objectAZ;
	double objectBX;
	double objectBY;
	double objectBZ;
	double objectANormX, objectANormY, objectANormZ;
	double d1;
	double d2;
    double d3;
	double d[3];
	

	double detectcollision3D::getobjectAX1(){
	return AX1;
	}
	void detectcollision3D::setobjectAX1(double AX1){}
	

	double detectcollision3D::getobjectAY1(){
	return AY1;
	}
	void detectcollision3D::setobjectAY1(double AY1){}


	double detectcollision3D::getobjectAZ1(){
	return AZ1;
	}
	void detectcollision3D::setobjectAZ1(double AZ1){}


	double detectcollision3D::getobjectANormX1(){
	return ANormX1;
	}
	void	detectcollision3D::setobjectANormX1(double ANormX1){}

	double detectcollision3D::getobjectANormY1(){
	return ANormY1;
	}
	void 	detectcollision3D::setobjectANormY1(double ANormY1){}

	double detectcollision3D::getobjectANormZ1(){
	return ANormZ1;
	}
	void 	detectcollision3D::setobjectANormZ1(double ANormZ1){}

	//************************************************************	

	double detectcollision3D::getobjectAX2(){
	return AX2;
	}
	void detectcollision3D::setobjectAX2(double AX2){}
	

	double detectcollision3D::getobjectAY2(){
	return AY2;
	}
	void detectcollision3D::setobjectAY2(double AY2){}


	double detectcollision3D::getobjectAZ2(){
	return AZ2;
	}
	void detectcollision3D::setobjectAZ2(double AZ2){}


	double detectcollision3D::getobjectANormX2(){
	return ANormX2;
	}
	void	detectcollision3D::setobjectANormX2(double ANormX2){}

	double detectcollision3D::getobjectANormY2(){
	return ANormY2;
	}
	void 	detectcollision3D::setobjectANormY2(double ANormY2){}

	double detectcollision3D::getobjectANormZ2(){
	return ANormZ2;
	}
	void 	detectcollision3D::setobjectANormZ2(double ANormZ2){}
//**********************************************************

	double detectcollision3D::getobjectAX3(){
	return AX3;
	}
	void detectcollision3D::setobjectAX3(double AX3){}
	

	double detectcollision3D::getobjectAY3(){
	return AY3;
	}
	void detectcollision3D::setobjectAY3(double AY3){}


	double detectcollision3D::getobjectAZ3(){
	return AZ3;
	}
	void detectcollision3D::setobjectAZ3(double AZ3){}


	double detectcollision3D::getobjectANormX3(){
	return ANormX3;
	}
	void	detectcollision3D::setobjectANormX3(double ANormX3){}

	double detectcollision3D::getobjectANormY3(){
	return ANormY3;
	}
	void 	detectcollision3D::setobjectANormY3(double ANormY3){}

	double detectcollision3D::getobjectANormZ3(){
	return ANormZ3;
	}
	void 	detectcollision3D::setobjectANormZ3(double ANormZ3){}

	//******************************************************


	double detectcollision3D::getobjectBX4(){
	return BX4;
	}
	void detectcollision3D::setobjectBX4(double BX4){}


	double detectcollision3D::getobjectBY4(){
	return BY4;
	}
	void	detectcollision3D::setobjectBY4(double BY4){}


	double detectcollision3D::getobjectBZ4(){
	return BZ4;
	}
	void 	detectcollision3D::setobjectBZ4(double BZ4){}

	
	//*********************************************************

	double detectcollision3D::getobjectBX5(){
	return BX5;
	}
	void detectcollision3D::setobjectBX5(double BX5){}


	double detectcollision3D::getobjectBY5(){
	return BY5;
	}
	void	detectcollision3D::setobjectBY5(double BY5){}


	double detectcollision3D::getobjectBZ5(){
	return BZ5;
	}
	void 	detectcollision3D::setobjectBZ5(double BZ5){}

	//*******************************************************


	bool detectcollision3D::collision(int ObjectA, int ObjectB){
			
			

			detectcollision3D detcoll;
			if(ObjectA==1){
			objectAX = detcoll.getobjectAX1();
			objectAY = detcoll.getobjectAY1();
			objectAZ = detcoll.getobjectAZ1();
			
			objectANormX = detcoll.getobjectANormX1();
			objectANormY = detcoll.getobjectANormY1();
			objectANormZ = detcoll.getobjectANormZ1();	
			}
			if(ObjectA==2){
			objectAX = detcoll.getobjectAX2();
			objectAY = detcoll.getobjectAY2();
			objectAZ = detcoll.getobjectAZ2();

			objectANormX = detcoll.getobjectANormX2();
			objectANormY = detcoll.getobjectANormY2();
			objectANormZ = detcoll.getobjectANormZ2();
			}	
			if(ObjectA==3){
			objectAX = detcoll.getobjectAX3();
			objectAY = detcoll.getobjectAY3();
			objectAZ = detcoll.getobjectAZ3();

			objectANormX = detcoll.getobjectANormX3();
			objectANormY = detcoll.getobjectANormY3();
			objectANormZ = detcoll.getobjectANormZ3();
			}		

			if(ObjectB==4){
			objectBX = detcoll.getobjectBX4();
			objectBY = detcoll.getobjectBY4();
			objectBZ = detcoll.getobjectBZ4();
			}
			if(ObjectB==5){
			objectBX = detcoll.getobjectBX5();
			objectBY = detcoll.getobjectBY5();
			objectBZ = detcoll.getobjectBZ5();
			}
			

			intersectionPointZ = objectBZ;
			
			if (objectANormZ != 0.0) {
			lambda = (objectBZ-objectAZ)/objectANormZ;
			}	else {
			lambda = (objectBZ-objectAZ)/0.000000000001;
			}
			intersectionPointX = objectAX + lambda * objectANormX;
	        intersectionPointY = objectAY + lambda * objectANormY;
	
		
	        d1 = objectBX-intersectionPointX;
	        d2 = objectBY-intersectionPointY;
	        d3 = objectBZ-intersectionPointZ;
	        d = {d1,d2,d3};


			

			printf ("Intersection Point:  %4.2f  %4.2f  %4.2f \n", intersectionPointX, intersectionPointY, intersectionPointZ);
			printf ("Distance:  %4.2f  %4.2f  %4.2f \n", d1, d2, d3);

			if(intersectionPointX != 0.0 || intersectionPointY != 0.0 || intersectionPointX != 0.0){
			return true;
			}else{
			return false;}

	}




/*int main() 
{

		detectcollision detcoll;
		detcoll.collision(1,4);
			
			
		return 0;
    }*/


