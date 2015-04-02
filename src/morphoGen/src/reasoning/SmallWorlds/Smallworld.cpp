
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

#include <string>
#include<time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <windows.h>
#include <iostream>
#include <fstream>
#include "Smallworld.h"

using namespace std;

Smallworld::Smallworld(void)
{
}

Smallworld::~Smallworld(void)
{
}

void Smallworld::HubRep()
	{
      InitializeSW();
	  GetLocalAct(NumWords);
      int PropWeightC=1; // this can came as a reulst of the elimination rule
	  Retroactivate(PropWeightC);
	};


void Smallworld:: Retroactivate(int PropWC)
	{
      int iCo, jCo, mCo, sumWHM;
	   ofstream HubA("HubA.txt");
	   ofstream MapA("MapA.txt");

      if(PropWC==0)
		  {
		     for(iCo=0; iCo<36; iCo++)
				 {
					sumWHM=0;
					for(jCo=0; jCo<60; jCo++)
									 {
                                        sumWHM=sumWHM+MapstoProvHub[iCo][jCo];
									 }
                     if(sumWHM==2)
						 {
						    for(mCo=0; mCo<60; mCo++)
								 {
                                   MapstoProvHub[iCo][mCo]=0;
							     }
						 }
			     }
		  
		  }

      for(iCo=0; iCo<500; iCo++)
		 {
			//WProvColShapRev*LocalAct': MapstoProvHub[36][90] Bottom up====================================
			 for(jCo=0; jCo<36; jCo++)
					 {
                       double pr_Co=0;  
						 for(mCo=0; mCo<90; mCo++)
							{
                              pr_Co = pr_Co + MapstoProvHub[jCo][mCo]*LocalMapAct[mCo];
							}

						 ProvHub[jCo]=ProvHub[jCo]+0.015*((-0.01*ProvHub[jCo])+pr_Co);
						 if(ProvHub[jCo]<0.0001)
							 {
								 ProvHub[jCo]=0;
							 }


    				 }

             //==================================== WProvColShapRev'*u ===Top Down==============;

             for(jCo=0; jCo<90; jCo++)
					 {
                       double pr_CoT=0;  
						 for(mCo=0; mCo<36; mCo++)
							{
                              pr_CoT = pr_CoT + ProvHubtoMaps[jCo][mCo]*ProvHub[mCo];
							}
						 LocalMapAct[jCo]=LocalMapAct[jCo]+1.3*((-0.01*LocalMapAct[jCo])+pr_CoT);
						 if(LocalMapAct[jCo]<0.0001)
							 {
								 LocalMapAct[jCo]=0;
							 }
					}  
    	 }

	  for(iCo=0; iCo<42; iCo++)
		 {
          HubA << ProvHub[iCo]<< endl;
	     }
	   for(iCo=0; iCo<90; iCo++)
		 {
          MapA << LocalMapAct[iCo]<< endl;
	     }

	};


void Smallworld:: GetLocalAct(int numW)
{
 int iCo, jCo, m, clocal=0; 
  ofstream MapAini("MapI.txt"); 
 double Rdis,RdisAct[90],MaxiAct=0.0001, pr_Co,RdisS,RdisActS[90],MaxiActS=0.0001, pr_CoS;
 double WorActiv[2][30],MaxiActW=0.0001,pr_CoW,RdisW,RdistempW;

 for(iCo=0; iCo<30; iCo++)
	 {
       Rdis=sqrt(pow(ColW[iCo][0]-ipCol[0],2)+pow(ColW[iCo][1]-ipCol[1],2)+pow(ColW[iCo][2]-ipCol[2],2));
	   RdisS=sqrt(pow(ColW[iCo][0]-ipShap[0],2)+pow(ColW[iCo][1]-ipShap[1],2)+pow(ColW[iCo][2]-ipShap[2],2));
       RdisAct[iCo]=1*exp(-(Rdis)/40);
	   RdisActS[iCo]=1*exp(-(RdisS)/40);
       if(RdisAct[iCo]>MaxiAct)
		   {
			 MaxiAct=RdisAct[iCo];
			 // u can also have a indicatior here of which neuron is most active in col som
		   }
	   if(RdisActS[iCo]>MaxiActS)
		   {
			 MaxiActS=RdisActS[iCo];
			 // u can also have a indicatior here of which neuron is most active in col som
		   }
	   }

 for(iCo=0; iCo<30; iCo++)
				{
				     pr_Co=0;
					 pr_CoS=0;
					 for(jCo=0; jCo<30; jCo++)
						{
							pr_Co = pr_Co + inhib[iCo][jCo]*RdisAct[jCo];
							pr_CoS = pr_CoS + inhib[iCo][jCo]*RdisActS[jCo];
						}

                    Col[iCo]=(RdisAct[iCo]-(0.035*pr_Co))/MaxiAct; 
					Shape[iCo]=(RdisActS[iCo]-(0.035*pr_CoS))/MaxiActS; 
                    LocalMapAct[clocal]=Col[iCo];
					LocalMapAct[60+clocal]=Shape[iCo];
                    clocal=clocal+1; 
				}

 //========================================================
//Words act
for(m=0; m<numW; m++)
	     {
			//RdistempW=0;
			MaxiActW=0.0001;
			for(iCo=0; iCo<30; iCo++)
				 {
                     RdistempW=0;
					 RdisW=0;					
					 for(jCo=0; jCo<120; jCo++)
						{
							RdistempW = RdistempW + ((WorW[iCo][jCo]-WordIn[m][jCo])*(WorW[iCo][jCo]-WordIn[m][jCo])); //pow(WorW[iCo][jCo]-WordIn[m][jCo],2)
						}
				   RdisW=sqrt(RdistempW);
				   RdisActW[m][iCo]=5.64*exp(-1*(RdisW/0.25));
				   if(RdisActW[m][iCo]>MaxiActW)
					   {
						 MaxiActW=RdisActW[m][iCo];
					   }
				 }
			cout << MaxiActW << endl;
			//Sleep(20000);
			//===============================================================
       for(iCo=0; iCo<30; iCo++)
				{
				     pr_CoW=0;
					for(jCo=0; jCo<30; jCo++)
						{
							pr_CoW = pr_CoW + inhib[iCo][jCo]*RdisActW[m][jCo];
						}
                    WorActiv[m][iCo]=(RdisActW[m][iCo]-(0.03*pr_CoW))/MaxiActW; 
					//WorActiv[m][iCo]=RdisActW[m][iCo]*10; 
                 }
         
       } // m loop of words to be projected to the word SOM

   for(iCo=0; iCo<30; iCo++)
				{
					if(numW==1)
					{
					 LocalMapAct[30+iCo]=WorActiv[0][iCo];
					}
					if(numW==2)
					{
					 LocalMapAct[30+iCo]=WorActiv[0][iCo]+WorActiv[1][iCo];
					}
                 }
 
 //===== here u get the net initial bottom up neural activity: col, shap: by vision and word through keyboard ========
   for(iCo=0; iCo<90; iCo++)
		 {
          //MapAini <<WorActiv[0][iCo]<<endl;
		  MapAini << LocalMapAct[iCo]<< endl;
	     }
};

void Smallworld::InitializeSW()
 {
    int m=0,n=0;
	//================== Initialize connectivity ==========================
    ifstream WCSW("WProvColShapRev.txt");
	ifstream WorN("wordW.txt");
	ifstream ColN("colNeuronsW.txt");
	ifstream WHMW("WHubToMaps.txt");

    for (m =0; m<36; m++)
				{
					for (n=0; n<90; n++)
					{
						 WCSW >> MapstoProvHub[m][n];
					}
				}

	for (m =0; m<90; m++)
				{
					for (n=0; n<36; n++)
					{
						 WHMW >> ProvHubtoMaps[m][n];
					}
				}

	for (m =0; m<30; m++)
				{
					for (n=0; n<120; n++)
					{
						 WorN >> WorW[m][n];
					}
				}
cout << WorW[0][8]<< endl;

for (m =0; m<30; m++)
				{
					for (n=0; n<3; n++)
					{
						 ColN >> ColW[m][n];
					}
				}

	for (m =0; m<2; m++)
				{
					for (n=0; n<120; n++)
					{
						 WordIn[m][n]=0;
					}
				}

	for (m =0; m<30; m++)
				{
					for (n=0; n<30; n++)
					{
						 inhib[m][n]=1;
						 if(m==n)
						 {
						   inhib[m][n]=0;
						 }
					}
				}

	  for (n=0; n<30; n++)
					{
						Col[n]=0;
						Word[n]=0;
						Shape[n]=0;
					}
	      ipCol[0]=0;
	      ipCol[1]=0;
	      ipCol[2]=0;
		  ipShap[0]=0;
		  ipShap[1]=0;
		  ipShap[2]=0;

	  for (n=0; n<90; n++)
					{
					 LocalMapAct[n]=0;
					}
	  for (n=0; n<42; n++)
					{
						ProvHub[n]=0;
						OCHub[n]=0;
					}


//================== Updating objects: This must come from perceptual processing, here through user i/p ==========================
   int sizz;
   ofstream WorBin("WorBin.txt");
   cout << "Input number of words " << endl;
   cin >> sizz;
   NumWords=sizz;
   	for (n=0; n<sizz; n++)
		{
		  WordEncode(n);
		}

	for (m =0; m<sizz; m++)
				{
					for (n=0; n<120; n++)
					{
						 WorBin << WordIn[m][n] << "    ";
					}
					WorBin << "    " << endl;
				}

  };

void Smallworld::WordEncode(int numu)
{
     
	string mycol;
    int sizz,n;
 
    cout << "Input word " << endl;
    cin >> mycol;

   sizz=mycol.size();
   cout << sizz << endl;

    for (n=0; n<sizz; n++)
		{
		 cout << mycol[n] << endl;
		 if (mycol[n] ==  'r') { WordIn[numu][(8+(n*20))]=1;} 
		 if (mycol[n] ==  'e') { WordIn[numu][(0+(n*20))]=1;} 
		 if (mycol[n] ==  'd') { WordIn[numu][(9+(n*20))]=1;} 
		 if (mycol[n] ==  't') { WordIn[numu][(1+(n*20))]=1;} 
		 if (mycol[n] ==  'a') { WordIn[numu][(2+(n*20))]=1;} 
		 if (mycol[n] ==  'o') { WordIn[numu][(3+(n*20))]=1;} 
		 if (mycol[n] ==  'i') { WordIn[numu][(4+(n*20))]=1;} 
		 if (mycol[n] ==  'n') { WordIn[numu][(5+(n*20))]=1;} 
		 if (mycol[n] ==  's') { WordIn[numu][(6+(n*20))]=1;} 
		 if (mycol[n] ==  'h') { WordIn[numu][(7+(n*20))]=1;} 
		 if (mycol[n] ==  'l') { WordIn[numu][(10+(n*20))]=1;} 
		 if (mycol[n] ==  'c') { WordIn[numu][(11+(n*20))]=1;} 
		 if (mycol[n] ==  'u') { WordIn[numu][(12+(n*20))]=1;} 
		 if (mycol[n] ==  'm') { WordIn[numu][(13+(n*20))]=1;} 
		 if (mycol[n] ==  'w') { WordIn[numu][(14+(n*20))]=1;} 
		 if (mycol[n] ==  'f') { WordIn[numu][(15+(n*20))]=1;} 
		 if (mycol[n] ==  'g') { WordIn[numu][(16+(n*20))]=1;} 
		 if (mycol[n] ==  'y') { WordIn[numu][(17+(n*20))]=1;} 
		 if (mycol[n] ==  'p') { WordIn[numu][(18+(n*20))]=1;} 
		 if (mycol[n] ==  'j') { WordIn[numu][(19+(n*20))]=1;} 

		 
		 }
   };