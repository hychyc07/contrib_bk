#include <string.h>
#include<time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
//#include <windows.h>
#include <iostream>
#include <fstream>
#include <AMem.h>
using namespace std;


AMem::AMem(void)
{
   N=1000;
}

AMem::~AMem(void)
{
}

void AMem:: MemControl()
	{
     InitializeAM();
     int Nrelevent=RememberPast();  //remember my past episodic experiences based on the present context
     int nwinner=TDMemCompHub(Nrelevent); // find which memories out of all those rememebred are msot valuable	 
	 int noverl=FindOverlap(nwinner); // find how various rememebred expereinces overlap in the knowledge they encode
     RipRealSequence(nwinner); // Take out useful object action sequnces that can be enacted in the present from these memories
	 PlanFromPastXP(nwinner,noverl);   // synthesize a novel behaviour or combine past experience with exploration
    };

void AMem:: PlanFromPastXP(int NWine, int Noverlapp)
	{
      int iCo,jCo,Olala=0,Pctrr,composi=0,Pctrr2;
	    	if(Noverlapp==0)
					{
						if(NWine==1)
							{
							  Pctrr=0;
						      cout << "Past Exp sequnce is directly available without  recombination/chunking" << endl;
							   for(iCo=0; iCo<IndexM[0]; iCo++)
										 {
										 for(jCo=0; jCo<50; jCo++)
											 {
							  					  PlanPastExp[Pctrr]=NowObAcSeq[0][Pctrr];
							    				  Pctrr=Pctrr+1;
											 }
										 }
							   PEnd=Pctrr;
								   if((SumVSSP-SumTopDown)<0.5)
									  {
										  cout << "Full knowledge exists in past experiences: Plan Synthesized! " << endl;
									  } 
								   if((SumVSSP-SumTopDown)>0.5)
									  {
										  cout << "Combining past experience with exploration: On novel objects " << endl;
										  Olala=NovelObID[0];
										  XploreCombact(Olala); 
									  } 
						      }

						if(NWine==2)
							{
						      cout << "need to combine nonoverlapping chunks of knowledge by exploration " << endl;
								// here u have to copy paste first action seq in usefulacseq as the plan
                            //=================================================================
				for(composi=0; composi<2; composi++)
						{
							Pctrr=0;
							Pctrr2=0;
							if(composi==0)
							{
								for(iCo=0; iCo<IndexM[0]+IndexM[1]; iCo++)
									{
										  if(iCo<IndexM[0])
										   {
											 for(jCo=0; jCo<50; jCo++)
												 {
							  					  PlanPastExp[Pctrr]=NowObAcSeq[0][Pctrr];
							    				  Pctrr=Pctrr+1;
												 }
										  }
										  if(iCo>=IndexM[0])
										   {
											 for(jCo=0; jCo<50; jCo++)
												 {
							  					  PlanPastExp[Pctrr]=NowObAcSeq[1][Pctrr2];
							    				  Pctrr=Pctrr+1;
												  Pctrr2=Pctrr2+1;
												 }
										  }
									  }
								PEnd=Pctrr;
                                if((SumVSSP-SumTopDown)<0.5)
									  {
										  cout << "Full knowledge exists in past experiences: Plan Synthesized! " << endl;
										  cout << "Trigger explorative action sequence 1 ? " << endl;
									  }

								   if((SumVSSP-SumTopDown)>0.5)
									  {
										  cout << "Combining past experience with exploration: On novel objects " << endl;
										  Olala=NovelObID[0];
										  XploreCombact(Olala); 
								      } 
                            					
                            }

							//---------------------------------------------------------------------------
							if(composi==1)
							{
								for(iCo=0; iCo<IndexM[0]+IndexM[1]; iCo++)
									{
										  if(iCo<IndexM[1])
										   {
											 for(jCo=0; jCo<50; jCo++)
												 {
							  					  PlanPastExp[Pctrr]=NowObAcSeq[1][Pctrr];
							    				  Pctrr=Pctrr+1;
												 }
										  }
										  if(iCo>=IndexM[1])
										   {
											 for(jCo=0; jCo<50; jCo++)
												 {
							  					  PlanPastExp[Pctrr]=NowObAcSeq[0][Pctrr2];
							    				  Pctrr=Pctrr+1;
												  Pctrr2=Pctrr2+1;
												 }
										  }
									  }
                             PEnd=Pctrr;
                                if((SumVSSP-SumTopDown)<0.5)
									  {
										  cout << "Full knowledge exists in past experiences: Plan Synthesized! " << endl;
										  cout << "Trigger explorative action sequence 2 ? " << endl;
									  } 
								   if((SumVSSP-SumTopDown)>0.5)
									  {
										  cout << "Combining past experience with exploration: On novel objects " << endl;
										  Olala=NovelObID[0];
										  XploreCombact(Olala); 
								      }     
			
							}//composi=1 loop
						}

//=======================================================================
						    }
					}

				if(Noverlapp > 0)
					{
					  MemComb(NWine);	
					   if((SumVSSP-SumTopDown)<0.5)
						  {
							  cout << "Full knowledge exists the novel action sequence synthesized by recombinign memories! " << endl;
						  } 
					  if((SumVSSP-SumTopDown)>0.5)
						  {
							  cout << "Combining past experience with exploration: On novel objects " << endl;
							  Olala=NovelObID[0];
							  XploreCombact(Olala); 
						  }     

					} 	   
		
	};

	void AMem::XploreCombact(int Ola)
		{
		  int NovObID, iCo,jCo;
		  NovObID=Ola;
         

		  for(iCo=0; iCo<2; iCo++)
			{ 
		      for(jCo=0; jCo<50; jCo++)
				{

					}
		  }


		  // take the past plan and append the novelty before or after: there will be 2 explorative plans
		};


void AMem::MemComb(int NWiner)
	{
		int iCo,jCo,dis[2],disDiff[2],ConcMem[3][1000],conccnt,coun2,compo;
		ofstream PXper("PXper.txt");
//===========================================================================================================
      cout << "Making a new plan combining multiple past Xpereinces" << endl;
                if(NWiner==2) // if there are many u need to remove this loop and iterate more
					{
						//put M1 before M2
						for (compo=0;compo<2; compo++)
						{
						 conccnt=0;
                         coun2=0;
                         for(iCo=0; iCo<IndexM[0]+IndexM[1]; iCo++)
						 {
							
							 for(jCo=0; jCo<50; jCo++)
							 {
								 if(compo==0)
								 {
									   if(iCo<IndexM[0])
									   {
										   ConcMem[compo][conccnt]=NowObAcSeq[0][conccnt];
										   if((jCo==OLap[0])&&(ConcMem[compo][conccnt]==1))
										   {
											dis[0]=iCo;
											cout << "Olap Element Mem1" << dis[0]<< endl;
										   }
									   }
										if(iCo >=IndexM[0])
										   {
											   ConcMem[compo][conccnt]=NowObAcSeq[1][coun2];
											   if((jCo==OLap[0])&&(ConcMem[compo][conccnt]==1))
											   {
												dis[1]=iCo;
												cout << "Olap Element Mem2" << dis[1]<< endl;
											   }
											   coun2=coun2+1;
											}
								 } // if compo =0 

								 if(compo==1)
								 {
									   if(iCo<IndexM[1])
									   {
										   ConcMem[compo][conccnt]=NowObAcSeq[1][conccnt];
										   if((jCo==OLap[0])&&(ConcMem[compo][conccnt]==1))
										   {
											dis[0]=iCo;
											cout << "Olap Element Mem1" << dis[0]<< endl;
										   }
									   }
										if(iCo >=IndexM[1])
										   {
											   ConcMem[compo][conccnt]=NowObAcSeq[0][coun2];
											   if((jCo==OLap[0])&&(ConcMem[compo][conccnt]==1))
											   {
												dis[1]=iCo;
												cout << "Olap Element Mem2" << dis[1]<< endl;
											   }
											   coun2=coun2+1;
											}
								 } // if compo =1

									conccnt=conccnt+1;
							 }// end of jCo loop

						 }// end of iCo loop

						 disDiff[compo]=dis[1]-dis[0];
						 cout << "Compo diss diff" << compo << "   " << disDiff[compo]<< endl;

						}// end of compo loop 
							
				    }// end of if Winer loop
    
//============ Removing redundant overlapps and synthesizing novel action sequnce ====================
  int Pctr;
	if(disDiff[0]<disDiff[1])
	  {
          conccnt=0;
		  Pctr=0;
		  for(iCo=0; iCo<IndexM[0]+IndexM[1]; iCo++)
				 {
					 if(iCo==dis[1])
						 {
						  iCo=iCo+2;
                          conccnt=conccnt+100; 
						 } 
					 if((iCo !=dis[1])||(iCo !=dis[1]+1))
						 {
						 for(jCo=0; jCo<50; jCo++)
							 {
							   PlanPastExp[Pctr]=ConcMem[0][conccnt];
							   conccnt=conccnt+1;
							   Pctr=Pctr+1;
							 }
					     }
		         }//iCo loop ends here
		  PEnd=Pctr;
	    }
 
  if(disDiff[0]>disDiff[1])
	  {
	  conccnt=0;
		  Pctr=0;
		  for(iCo=0; iCo<IndexM[0]+IndexM[1]; iCo++)
				 {
					 if(iCo==dis[0])
						 {
						  iCo=iCo+2;
                          conccnt=conccnt+100; 
						 } 
					 if((iCo !=dis[0])||(iCo !=dis[0]+1))
						 {
						 for(jCo=0; jCo<50; jCo++)
							 {
							   PlanPastExp[Pctr]=ConcMem[0][conccnt];
							   conccnt=conccnt+1;
							   Pctr=Pctr+1;
							  }
					     }
		         }//iCo loop ends here
	    PEnd=Pctr;
	  }
  if(disDiff[0]==disDiff[1])
	  {
	   //Need to explore further both action sequences in ConcMem....
	  
	  }

	for(iCo=0; iCo<1000; iCo++)
		{
		 PXper<< PlanPastExp[iCo] <<endl;
		}

	};


void AMem:: RipRealSequence(int nme)
	{
	 int iCo,jCo,jCoo,MCnt,UnfM[20][50], UseSeq[20][50];
	 ofstream UsePCue("UsefulAcSeqs.txt");
     for(MCnt=0; MCnt<nme; MCnt++)
	 {
//====================================================================================
		 int counUnf=0;
		 for(iCo=0; iCo<20; iCo++)
				{
				  for(jCo=0; jCo<50; jCo++)
						{
	      					UseSeq[iCo][jCo]=0;
							UnfM[iCo][jCo]=NRelPast[MemID[MCnt]][counUnf];
							if(UnfM[iCo][jCo]>0.75)
									{
									  UnfM[iCo][jCo]=1;
									}
							counUnf=counUnf+1;
							if((jCo==44)&&(UnfM[iCo][jCo]==1))
								{
								 IndexM[MCnt]=iCo;
								}
						}
				}  
                // Here u have the index of the useful part of the useful memories
                int ICT=IndexM[MCnt];
				int CntUse=0;
                for(iCo=0; iCo<ICT; iCo=iCo+2)
				  {
                    int SuVssp=0; 
                    for(jCo=0; jCo<36; jCo++)
						{
                         SuVssp=SuVssp+UnfM[iCo][jCo]*VSSP[jCo];
					    }
					if(SuVssp != 0)
						{
						 for(jCoo=0; jCoo<50; jCoo=jCoo+1)
							  {
                               UseSeq[CntUse][jCoo]=UnfM[iCo][jCoo];
							   UseSeq[CntUse+1][jCoo]=UnfM[iCo+1][jCoo];
                               }
						 CntUse=CntUse+2;
						} //ripped the object action sequnce
					if(SuVssp == 0)
						{
                         IndexM[MCnt]=IndexM[MCnt]-2; //because one object remembered of the past is not there in the now
						 // hence cannot be acted upon
					    }
				  }
  /////// Reunfold the object-action sequnce relevant in the now emerging from this past experience
     int countRel=0;
				for(iCo=0; iCo<20; iCo++)
					{
					  for(jCo=0; jCo<50; jCo++)
							{
								NowObAcSeq[MCnt][countRel]=UseSeq[iCo][jCo];
								UsePCue << NowObAcSeq[MCnt][countRel]<< "    ";
								countRel=countRel+1;
							}
				}
            UsePCue << "    " << endl;
  // ========================================================================================================        
  //for MCnt ends below
	 }
   	};

int AMem::FindOverlap(int NW)
	{
      int iCo,  Overl=0;
	  double memul, vssptddiff=0;
	  NNovobs=0;
	  if(NW==1)
		  {
		  cout << "There is a single winner" << endl;
           for(iCo=0; iCo<36; iCo++)
				{
					vssptddiff=VSSP[iCo]-HubTopDown[MemID[0]][iCo];
					if(vssptddiff>0.2)
						{
							cout << "There is a novel object in VSSP that is not there in TD Hub activity" << endl;
							NovelObID[NNovobs]=iCo;
							NNovobs=NNovobs+1; //size issue is here : to solve it make VSSP neural activity 
							//a unique ID that codes for both size and shape, only large objects need to be reassigned a new neuron
						}
				}
		   cout << "Novel obejcts" << NNovobs <<endl;
		  }

      if(NW==2)
		  {
		    for(iCo=0; iCo<36; iCo++)
				{
                  memul=0;
			      memul=HubTopDown[MemID[0]][iCo]*HubTopDown[MemID[1]][iCo];
			      if(memul > 0.1) 
					  {
					   Overl=Overl+1;
					   OLap[Overl-1]=iCo;
					  }
                   vssptddiff=VSSP[iCo]-(HubTopDown[MemID[0]][iCo]+HubTopDown[MemID[1]][iCo]);
					if(vssptddiff>0.2)
						{
							cout << "There is a novel object in VSSP that is not there in TD Hub activity" << endl;
							NovelObID[NNovobs]=iCo;
							NNovobs=NNovobs+1; //size issue is here : to solve it make VSSP neural activity 
							//a unique ID that codes for both size and shape, only large objects need to be reassigned a new neuron
						} 
			    }
			cout << "No of overlaps" << "   " <<  Overl  << "Element in Hub" << "   " << OLap[0] <<endl;
			cout << "Novel obejcts" << NNovobs <<endl;
		   }
return Overl;
 };


int AMem:: TDMemCompHub(int Nrelev)
	{
		int InstMk=0,iCo,jCo,Nmemos=0,Hubu[42],nom,comprel=0,UnfR[20][50],rewno;
		double SunHI[6];
		SumTopDown=0;
		ofstream HeeHu("HubTDownCont.txt");
		ofstream HeeHoo("HubTDCom.txt");
       
				   for(jCo=0; jCo<6; jCo++)
						{
							SunHI[jCo]=0;
							MemID[jCo]=0;
						}
                
		for(iCo=0; iCo<5; iCo++)
				{
				   for(jCo=0; jCo<42; jCo++)
						{
							HubTopDown[iCo][jCo]=0;
						}
                }  
	
        for(jCo=0; jCo<42; jCo++)
						{
						Hubu[jCo]=OHub[jCo];
    					}
         Hubu[36]=0; Hubu[37]=0; 

//=================== Init done, Now find the anticipated reward fetched by each remembered memory and objects they know about=============
		for(comprel=0; comprel<Nrelev; comprel++)
				{ 
					int counUnf=0;
					for(iCo=0; iCo<20; iCo++)
							{
							  for(jCo=0; jCo<50; jCo++)
									{
										UnfR[iCo][jCo]=NRelPast[comprel][counUnf];
										counUnf=counUnf+1;
									}
							}
                     for(iCo=0; iCo<20; iCo++)
							{
								if(UnfR[iCo][44]>0.8)
									{
									  InstMk=iCo;
									}
					        }
	                 for(jCo=0; jCo<36; jCo++)
									{
										if(UnfR[InstMk][jCo]>0.8)
											{
											    rewno=jCo;
												nom=(jCo+1)/(0.5*InstMk);
											}
									}  
        
                   cout << "Anticipated reward for memory"  << "\t" << comprel+1 << "is" << "\t" << nom << "\t"  << InstMk << "\t" << rewno  << endl;
				//   HubActNote[comprel][6]=nom;
				  				   
	//====================== here u extract the top down influence of this experience =============================
          	for(iCo=0; iCo<42; iCo++)
				{
				    int pr_CooC=0;
					 for(jCo=0; jCo<1000; jCo++)
						{
							pr_CooC = pr_CooC + WHub2EpimT[iCo][jCo]*NRelPast[comprel][jCo];
						}
                   HubTopDown[comprel][iCo]= pr_CooC*Hubu[iCo]*nom;
				}
            
			SunHI[comprel]=0; 
			for(iCo=0; iCo<42; iCo++)
				 {
				 HeeHu<< HubTopDown[comprel][iCo]<< "    ";
                 SunHI[comprel]=SunHI[comprel]+HubTopDown[comprel][iCo];  //Sum HubTD
				 }
			 HeeHu << "    " << endl;

				// loop ends here
			}

if(Nrelev>1)
		{
			 int iterComp, iterHub,iterHubM;
			 for(iterComp=0; iterComp<5; iterComp++)
					 {
						 for(iCo=0; iCo<Nrelev; iCo++)
							 {
								 double MaxHu=0.0001;
								 for(jCo=0; jCo<Nrelev; jCo++)
									 {
										if(iCo != jCo)
											{
											  for(iterHub=0; iterHub<42; iterHub++)
												 {
													HubTopDown[iCo][iterHub]=HubTopDown[iCo][iterHub]-(0.1*(HubTopDown[iCo][iterHub]*HubTopDown[jCo][iterHub]*SunHI[jCo]));
													if(HubTopDown[iCo][iterHub] < 0.25)
													{
                                                    HubTopDown[iCo][iterHub]=0;
													}
													if(HubTopDown[iCo][iterHub]>MaxHu)
														{
														MaxHu=HubTopDown[iCo][iterHub];
														}
											    }
											  
											//here u get all 42 numbers, u need to find max, normalize and go to next iteration
										}
										for(iterHubM=0; iterHubM<42; iterHubM++)
												{
													HubTopDown[iCo][iterHubM]=HubTopDown[iCo][iterHubM]/MaxHu;										
												}
                                        //jCo loop of reciveing inhibition from all memories competing is over
                                        
									  }
							 }
					  }
		//compet if loop ends below 
		}
for(iCo=0; iCo<Nrelev; iCo++)
				 {
                    SunHI[Nrelev]=0;
					for(jCo=0; jCo<42; jCo++)
					 {
					   HeeHoo<< HubTopDown[iCo][jCo] << "    ";
					   SunHI[Nrelev]=SunHI[Nrelev]+HubTopDown[iCo][jCo];
					 }
					 HeeHoo << "    " << endl;
					 if(SunHI[Nrelev]<0.4) // was 0.9
					 {
					  SunHI[Nrelev]=0;
					 }
                     if(SunHI[Nrelev]>0.4)
					 {
					  Nmemos=Nmemos+1;
					  MemID[Nmemos-1]=iCo;
					 }
					 cout << "Winning MemID"  << "  " << MemID[Nmemos-1] << " power " << SunHI[Nrelev] << " No of winners so far " << Nmemos  <<endl ;
				     SumTopDown=SumTopDown+SunHI[Nrelev];
    }
//here we get the final result of competing memories, know how many survived Nmemos, who all are they...
cout << "Top Down Sum"  << "  " << SumTopDown << endl ;

return Nmemos;
	};



int AMem::RememberPast() // loops from the present to partial cue to remembered past
{
	int NRelEp=0,icn,HE[1000],AE[1000],sumhe,tempHE,iCo,jCo,VunF[20][50],PartCue[5][1000];  //OHub WHub2Epim Episodes
	ofstream PCue("PCue.txt");
	ofstream Hee("HeEpMult.txt");
	for(icn=0; icn<NumEpi; icn++)
	{
		sumhe=0;
        tempHE=0;
	   	//==================== Instantaneous Hub (1x42)* WHubEpim (42x1000)========================================
           	for(iCo=0; iCo<1000; iCo++)
				{
				    int pr_Coo=0;
					 for(jCo=0; jCo<42; jCo++)
						{
							pr_Coo = pr_Coo + WHub2Epim[iCo][jCo]*OHub[jCo];
						}

                    HE[iCo]=pr_Coo*Episodes[icn][iCo]; //bloody
					sumhe=tempHE+ HE[iCo];
                    tempHE=sumhe;
				}
	//==========There is potential for remembering here ========================
    if(tempHE>0)
		{
			NRelEp=NRelEp+1;
		//=====================Add Acn==============================================
			for(iCo=0; iCo<1000; iCo++)
					{
						int pr_CooA=0;
						 for(jCo=0; jCo<9; jCo++)
							{
								pr_CooA = pr_CooA + WAct2Epim[iCo][jCo]*Action[jCo];
							}

						AE[iCo]=(pr_CooA+HE[iCo])*Episodes[icn][iCo]; //bloody
					}
		// Increment no of past experiences, Add tags
        // Retrieve full memo
        // Store experience in WM
        //====================================Unfold2by2==================================
			int counUnf=0;
            for(iCo=0; iCo<20; iCo++)
					{
					  for(jCo=0; jCo<50; jCo++)
							{
								VunF[iCo][jCo]=AE[counUnf];
								counUnf=counUnf+1;
							}
						}
       //==================================Add Hub Tags===================             
        for(iCo=0; iCo<20; iCo=iCo+2)
			{
              int sumUnf=0;
			  for(jCo=0; jCo<50; jCo++)
				  {
				   sumUnf=sumUnf+VunF[iCo][jCo];
				  }
              if(sumUnf>0)
			  {
			  VunF[iCo][42]=1;
			  }
			}
		for(iCo=1; iCo<20; iCo=iCo+2)
			{
              int sumUnfA=0;
			  for(jCo=0; jCo<50; jCo++)
				  {
				   sumUnfA=sumUnfA+VunF[iCo][jCo];
				  }
              if((sumUnfA>0)&&(VunF[iCo-1][42]==1))
			  {
			  VunF[iCo][43]=1;
			  }
			}
//=============================================
     int counti=0;
				for(iCo=0; iCo<20; iCo++)
					{
					  for(jCo=0; jCo<50; jCo++)
							{
								PartCue[NRelEp-1][counti]=VunF[iCo][jCo];
								Uini[counti]=VunF[iCo][jCo];
								PCue << PartCue[NRelEp-1][counti]<< "    ";
								counti=counti+1;
							}
				}
            PCue << "    " << endl;
//========================Partical cue is generated and written==========================================
// Retrieve full experience from partial cue in the present context, store it in appropriate location in NrELeP
        RetrievalFromCue(NRelEp-1);
	}
			
	//====================================================================
  	
     for(iCo=0; iCo<1000; iCo++)
	 {
	     Hee<< NRelPast[NRelEp-1][iCo]<< "    ";
	 }
     Hee << "    " << endl;
    // cout << "sum potential" <<sumhe << endl ;

	}
	
	return NRelEp; //here u get all remembered experiences from the past, related to the current situation at hand 
};


void AMem::RetrievalFromCue(int Nrelpos)
{
double delt=0.0015,inhibi=0,sumV=0;
int j=0, k=0;
double VinstT[1000],TempAct[1000],Vsig[1000];

		for (j=0; j<1000; j++) 
				 {
					Vsig[j]=Uini[j];	//Uini stores the partial cue, Vnet is the final activity			
				 }
		           					
for (k=1; k<500; k++)
	 {
		    sumV=0;
			for (j=0; j<1000; j++) //summation of init cond
				 {
					sumV=sumV+Vsig[j];				
				 }
			inhibi=-30+(3.5*sumV); //net inhibitory network current
			if(inhibi<=0) 
			{
			 inhibi=0;
			}
			//cout << inhibi << endl ;
			//==================== Instantaneous VT 1000*1000 x 1000*1========================================
            int iCo,jCo;
			for(iCo=0; iCo<N; iCo++)
				{
				    double pr_Co=0;
					 for(jCo=0; jCo<N; jCo++)
						{
							pr_Co = pr_Co + data[iCo][jCo]*Vsig[jCo];
						}

                    VinstT[iCo]=pr_Co; 
				}
			//====================================================================
          for(iCo=0; iCo<N; iCo++)
			   {
               TempAct[iCo]= delt*((-0.01*Uini[iCo])+VinstT[iCo]-inhibi);
			   Uini[iCo]=Uini[iCo]+TempAct[iCo];
			   if(Uini[iCo]<=0) 
				   {
				     Vsig[iCo]=0;
				   }
				  if(Uini[iCo]>0) 
				   {
				      Vsig[iCo]= Uini[iCo];
				   }
			   } 
   
	  }
               				
//=====================================================================
      int iCo; 
      ofstream RecMem("Remembered.txt");
    			for (iCo=0; iCo<1000; iCo++)
				    	{
                         if(Vsig[iCo]>0) 
						   {
							  VnetAct[iCo]=1;
						   }
						 if(Vsig[iCo]<=0) 
						   {
							  VnetAct[iCo]=0;
						   }
						 RecMem << Vsig[iCo]<< endl;
						 NRelPast[Nrelpos][iCo]=VnetAct[iCo];
						}
};


void AMem::MemMaint()
{
	int T[1000][1000],m,n,j;
    for (m =0; m<1000; m++)
				{
					for (n=0; n<1000; n++)
					{
						T[m][n]=0;
					}
				}

	for (m =0; m<1000; m++)
				{
					for (n=0; n<1000; n++)
					{
						for (j=0; j<NumEpi; j++)
						{
						T[m][n]=T[m][n]+ (Episodes[j][m]*Episodes[j][n]);
						}
						if(m==n)
						{
						T[m][n]=0;
						}
					}
				}

	for (m =0; m<1000; m++)
				{
					for (n=0; n<1000; n++)
					{
						if(T[m][n]>0)
						{
						T[m][n]=1;
						}
						else
						{
						T[m][n]=0;
						}
                     data[m][n]=T[m][n];
					}
	}
		
};

void AMem::InitializeAM()
 {
	int m=0,n=0;
    ifstream NEp("Numepi.txt");
    NEp>> NumEpi;
	cout << "Number of episodes experiences in memory" << "\t" <<NumEpi << endl ;
   //===================================================================================

	ifstream EpiW("EpisodesNew.txt");
    if(!EpiW)
           { cout << "Error opening Network weight matrix" << endl; }
    else
           { cout << "Loading Existing expereince" << endl;}
		for (m =0; m<25; m++)
				{
					for (n=0; n<1000; n++)
					{
						 EpiW >> Episodes[m][n];
						//  cout << Episodes[m][n] << endl ;
					}
				}
//===================================================================================

ifstream TCo("WeightN.txt");

data = (int **) malloc(1000 * sizeof(int)); 
for (int h = 0; h < 1000; h++)  
         data[h] = (int*) malloc(1000*sizeof(int));
   
 if(!TCo)
           { cout << "Error opening Network weight matrix" << endl; }
    else
           { cout << "Loading Neural connectivity of AssocMem" << endl;}

 for (m =0; m<1000; m++){
	 for (n =0; n<1000; n++)
				{
						 TCo >> data[m][n];
					   //  cout << data[m][n]<< endl ;
					}
 }
	//DumpAM();
    //===================================================================================

	ifstream Cu("cue.txt");
    { cout << "Loading partial cue" << endl;}
					for (n=0; n<1000; n++)
					{
						 Cu >> Uini[n];
				    }
                    cout << Uini[42] << endl ;
					cout << Uini[55] << endl ;
//===================================================================================
					// RetrievalFromCue();
//===================================================================================
ifstream Whub("whepis.txt");
  for (m =0; m<1000; m++)
				{
					for (n=0; n<42; n++)
					{
						 Whub >> WHub2Epim[m][n];
					}
				}
cout << "Loading Hub Epim Net" <<WHub2Epim[18][18] << endl ;

//===================================================================================
ifstream WhubTt("WHubToEpis.txt");
  for (m =0; m<42; m++)
				{
					for (n=0; n<1000; n++)
					{
						 WhubTt >> WHub2EpimT[m][n];
					}
				}
cout << "Loading Hub Epim NetT" <<WHub2EpimT[18][18] << endl ;

//===================================================================================
   ifstream WAcE("WActToEpis.txt");
    if(!WAcE)
           { cout << "Error opening Hub Epim Net" << endl; }
    else
           { cout << "Loading Act Epim Net" << endl;}
		for (m =0; m<1000; m++)
				{
					for (n=0; n<9; n++)
					{
						 WAcE >> WAct2Epim[m][n];
					}
				}
					cout << WAct2Epim[55][5] << endl ;
					
//===================================================================================
   for (m =0; m<42; m++)
	   {
		OHub[m]=0;
		VSSP[m]=0;
	   }
OHub[20]=1;  // This initialization can from from bottom up perception
OHub[13]=1;
OHub[16]=1;
OHub[18]=1;
OHub[37]=1;
SumVSSP=0;
for (m =0; m<42; m++)
	   {
		VSSP[m]=OHub[m];
		if(m<36)  // note that u only take shape into account assuming 2 shapes of different sizes do not come into picture
			{
			   SumVSSP=SumVSSP+OHub[m];
			}
	   }
cout << "Visuo Spatial sketch pad cumulative activity" << "\t" << SumVSSP << endl ;

//===================================================================================
for (m =0; m<9; m++)
	   {
		Action[m]=0;
	    }
	  Action[5]=1;   //this initialization can come through user goal, all set

//===================================================================================

for (m =0; m<5; m++)
				{
					for (n=0; n<1000; n++)
					{
						NRelPast[m][n]=0;
						NowObAcSeq[m][n]=0;
						PlanPastExp[n]=0;
					}
				}
for (n=0; n<1000; n++)
	{
	 PlanPastExp[n]=0;
	}

//===================================================================================
};


void AMem::DumpAM()
{
	int m=0,n=0;
	ofstream NetWO("WeightN.txt");
    		for (m =0; m<1000; m++)
				{
					for (n=0; n<1000; n++)
					{
						 NetWO << data[m][n] << "    ";
					}
					NetWO << "    " << endl;
				}
			free(data); 

//====================Updating new episodes learnt in the now ===============================
			ofstream EpiWO("EpisodesNew.txt");
    		for (m =0; m<25; m++)
				{
					for (n=0; n<1000; n++)
				    	{
						 EpiWO << Episodes[m][n]<< "    ";
						}
					EpiWO << "    " << endl;
				}
//======================== Updating number of episdoes of experiences in the memory at present =========
    ofstream NEpo("Numepi.txt");
    NEpo<< NumEpi;
	cout << "Number of episodes experiences in memory" << "\t" <<NumEpi << endl ;

}

int AMem::Random_Zel(int lim)
{
   srand( (unsigned)time( NULL ) );
   int addn=rand();
   static long a = 100001+addn; 
   int p;
   a = (a * 125) % 2796203;   
   p= ((a % lim));
   cout << "\n\n Random Select %d" << "\t" << p << endl;
   return p;
};

