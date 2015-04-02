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

#ifndef detectcollision3D_H
#define detectcollision3D_H


#include <iostream>
//#include <sstream>

using namespace std;


class detectcollision3D {
public:
	
	double getobjectAX1();
	void setobjectAX1(double AX1);
	
	double getobjectAY1();
	void	setobjectAY1(double AY1);

	double getobjectAZ1();
	void	setobjectAZ1(double AZ1);

	double getobjectANormX1();
	void	setobjectANormX1(double ANormX1);

	double getobjectANormY1();
	void 	setobjectANormY1(double ANormY1);

	double getobjectANormZ1();
	void 	setobjectANormZ1(double ANormZ1);


	double getobjectAX2();
	void setobjectAX2(double AX2);
	
	double getobjectAY2();
	void	setobjectAY2(double AY2);

	double getobjectAZ2();
	void	setobjectAZ2(double AZ2);

	double getobjectANormX2();
	void	setobjectANormX2(double ANormX2);

	double getobjectANormY2();
	void 	setobjectANormY2(double ANormY2);

	double getobjectANormZ2();
	void 	setobjectANormZ2(double ANormZ2);


	double getobjectAX3();
	void setobjectAX3(double AX3);
	
	double getobjectAY3();
	void	setobjectAY3(double AY3);

	double getobjectAZ3();
	void	setobjectAZ3(double AZ3);

	double getobjectANormX3();
	void	setobjectANormX3(double ANormX3);

	double getobjectANormY3();
	void 	setobjectANormY3(double ANormY3);

	double getobjectANormZ3();
	void 	setobjectANormZ3(double ANormZ3);


	double getobjectBX4();
	void	setobjectBX4(double BX4);

	double getobjectBY4();
	void	setobjectBY4(double BY4);

	double getobjectBZ4();
	void 	setobjectBZ4(double BZ4);

	
	double getobjectBX5();
	void	setobjectBX5(double BX5);

	double getobjectBY5();
	void	setobjectBY5(double BY5);

	double getobjectBZ5();
	void 	setobjectBZ5(double BZ5);

	bool collision(int ObjectA, int ObjectB);

};


#endif // detectcollision3D_H
