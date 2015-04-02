// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Vishwanathan Mohan
  * email: vishwanathan.mohank@iit.it
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


#pragma once

class Smallworld
{
	private:
    double Col[30],Word[30],Shape[30],ProvHub[42],OCHub[42],LocalMapAct[90],ipCol[3],ipShap[3],RdisActW[2][30];
	int MapstoProvHub[36][90],ProvHubtoMaps[90][36],ColW[30][3], WorW[30][120];
	int WordIn[2][120], inhib[30][30],NumWords;


	
	public:
		Smallworld(void);
		~Smallworld(void);
		void InitializeSW();
		void WordEncode(int numu);
		void GetLocalAct(int numW);
		void Retroactivate(int PropWC);
		void HubRep();

};
