
#include <ObserverThread.h>
#include <cstring>
#include <string>
#include<time.h>
#include <math.h>
#include <iostream>
#include <fstream>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

ofstream Report("Report.txt");

ObserverThread::ObserverThread() {
    robot = "icub";        
}

ObserverThread::ObserverThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

ObserverThread::~ObserverThread() {
    // do nothing
}

bool ObserverThread::threadInit() {
   
   
	if (!QueryEPIM.open(getName("/what-to-do:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   
	
	if (!PlanEx.open(getName("/Strategy:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }  

	if (!EpimCtrlPort.open(getName("/EpimCtrl:io").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }  

	if (!UserServices.open(getName("/UserServer:io").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }  

	if (!OPCCtrlPort.open(getName("/SmallWorldsOPC:io").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }  

	if (!BodySchemaCtrlPort.open(getName("/BodySchemaSim:io").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }  

	if (!GraspPort.open(getName("/GraspCtrl:io").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }  
	
	 return true;
}

void ObserverThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string ObserverThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void ObserverThread::setInputPortName(string InpPort) {
    
}

void ObserverThread::run()
{   
	KeySend=0;
	int AckMesEp=0;
	//state = 1; //VM made state as 2 for checking with TX190L 14 feb 2013
   state = 0;
	while (isStopping() != true) {

		switch(state) {
			case 0 : {
				    int iCo;
					cout<<" Waiting for user goal " <<endl;	 
					Report<<" Waiting for user goal " <<endl;	
					GiD=0;
					cin >> GiD;
					//if goal is reach or grasp:go to 2, if stack go to 1 to find a plan and then go to 2
					Network::connect("/SmallWorldsOPC:io", "/OPCServer:io");  // connect to server OPC 
					Network::connect("/BodySchemaSim:io", "/PMPreply:io");  
					Network::connect("/GraspCtrl:io", "/Graspreply:io");
					if(GiD==14){
					Replan=0;
					RefreshPlacemap(); // commented for testing 10-03-2014 ********************************
					state=1;
					} 

					if(GiD==32){
							NPiCs=1;
							Replan=0;
							for(iCo=0; iCo<10; iCo++){
							  SeqAcP[iCo]=50;
							}
							RefreshPlacemap(); // commented for testing 10-03-2014 ********************************
							for(iCo=0; iCo<NumberofObs; iCo++){
							  SeqAcP[iCo]=ObjIDEE[iCo];
							  cout<<"Oject id is "<<SeqAcP[iCo]<<endl;
							  NPiCs=NPiCs+1;
							}

							state = 5;
							//state=0;
					} 
			}
			break;

			case 1: {
                   Network::connect("/EpimCtrl:io", "/strategy:io");  
				   for(int i=0;i<1000;i++)
					  {
						 Strata[i]= 0;
					  }
					cout << "Sending snapshot to EPIM "<< endl;	
					Report << "Sending snapshot to EPIM "<< endl;	
                      
			// for tests with Xlate and Inv-Xlate// this should come from Refresh Place Map that issues OPC the microgoal to FORTH
				    for(int i=0;i<10;i++)
							{
								ObjIDEE[i]=50; //null object
								XlatTrackP[i]=50;
								XlatTrackPl[i]=50;
							}
						if(Replan==0){
					    NumberofObs=4;
						NumberofObsE=4;
                        largeness=1;
                        ObjIDEE[0]=7; //mush
	     				ObjIDEE[1]=6; 
						ObjIDEE[2]=2; //cyli
					 	ObjIDEE[3]=0;
						//ObjIDEE[3]=2;
						//ObjIDEE[3]=0; //
						} 
						if(Replan==1){
					    NumberofObs=2;
						NumberofObsE=1;
                        largeness=0;
                        ObjIDEE[0]=4; //mush
	     				ObjIDEE[1]=0; 
						//ObjIDEE[2]=4; //cyli
						} 
						for(int i=0;i<NumberofObs;i++)
							{
								XlatTrackP[i]=ObjIDEE[i];
								XlatTrackPl[i]=ObjIDEE[i]; 
						    } 
						//=====================================================
						Bottle cmd, response;
						cmd.addVocab(COMMAND_VOCAB_REQ);
						Xlator(); //*******************************************************************************
                        cmd.addDouble(NumberofObsE);
						cmd.addInt(largeness);
						for(int i=0;i<NumberofObsE;i++)
							{
								cmd.addDouble(ObjIDEEEpim[i]); //this should be replaced with ObjIDEE[i] coming from OPC server
							}
	                    
						EpimCtrlPort.write(cmd,response);
						printf("%s \n",response.toString().c_str());

					int responsecode = response.get(0).asVocab();
					cout<<responsecode<<endl;

					if(responsecode == 123 /*COMMAND_VOCAB_ACK*/) {
						ofstream PXper("PXper.txt");
						 cout << "Receiving Plan from server" << endl;
						 Report <<"Receiving Plan from server" << endl;
						 for(int i=0;i<1000;i++)
							{
							 Strata[i]= response.get(i+1).asInt();
							 PXper << Strata[i] <<endl;
							}
						 cout << "Plan recd sucessfully: Requesting EPIM to wait for next event" << endl;
						 Report << "Plan recd sucessfully: Requesting EPIM to wait for next event" << endl;

						 state=2;
					}
			}
			break;
			
			case 2: 
				{
					int iCo,jCo;
					int Pctrr=0;
					int AcSeq[20][50];
					for(iCo=0; iCo<6; iCo++){
					  SeqAc[iCo]=50;
					  }
					for(iCo=0; iCo<10; iCo++){
					  SeqAcP[iCo]=50;
					//  PastPlan[iCo]=50;
					  }
					//===================================================
					ifstream PlnW("PXper.txt");
					if(!PlnW)
						   { cout << "Error opening plan" << endl; }
					else
						   { cout << "Loading plan" << endl;
					         Report<< "Loading plan" << endl;}
					for (iCo=0; iCo<1000; iCo++)
						   {
							PlnW >> Strata[iCo];
						   }
					//==============Just for testing, this will be usally fed from EPIM=======			
					for(iCo=0; iCo<20; iCo++)
							 {
								for(jCo=0; jCo<50; jCo++)
										{
							  				 AcSeq[iCo][jCo]=Strata[Pctrr];
											 Pctrr=Pctrr+1;
										 }
							 }

					int NObjs=0;
                    int ctr=1;
					for(iCo=0; iCo<20; iCo++)
							 {
                                if(AcSeq[iCo][42]==1)
                                  {
									    NObjs=NObjs+1;
										for(jCo=0; jCo<36; jCo++)
												{
							  						if(AcSeq[iCo][jCo]==1)
														{
														 SeqAc[ctr]=jCo;
														 cout << "found object" << SeqAc[ctr] << endl;  
														 ctr=ctr+1;
														}
												}
									}
							 }
					  SeqAc[0]=NObjs;
					  cout<< "there are " << SeqAc[0] << "object shapes to stack: Interpreting.." << endl;
                      Interpret();  // Interprets plan  in shapes to plan in FORTH representations
					   if(Replan==0){
							 state = 5;
						 }
					    if(Replan==1){
							 state = 4;
						 }
				}
			break;

			case 4 : {
			  cout<<"Merging the incompletely executed past plan with the new one from EPIM..."<<endl;
			  Report<<"Merging the incompletely executed past plan with the new one from EPIM..."<<endl;
              Mergence();
			  cout<<"Mergence synthesized a posisble plan"<<endl;
			  Report<<"Mergence synthesized a posisble plan"<<endl;
			  state=5;
			}
			break;

			case 5: 
				  {	  
					  int iCo,jCo;
					  int StakSuc=0; 
					  if(Replan==1){
					  Replan=0; //when ever it enters pick and place loop there are chances of failing and replanning
					  //so this is implemented such that the system calls itself recurrsively ..... VM1203
					  cout<<"Building up on a past plan that was not sucessfull: Trying Again...."<<endl; 
					  Report<<"Building up on a past plan that was not sucessfull: Trying Again...."<<endl; 
				     }
					  PtOfReplan=0;//***************************************************


					 // for(iCo=0; iCo<NPiCs-1; iCo++)
						//{ 
						//	cout<< "Proposed Plan is as follows:" << endl;         
						//	cout << "Picking : " << SeqAcP[iCo+1]  << " and placing it on : " << SeqAcP[iCo] << endl;
						//}
					 // 
					 // Time::delay(20);


					  for(iCo=0; iCo<NPiCs-1; iCo++)
							 { 
							   int pickk=50;
							   int Placc=50;
							   if(GiD==14){
							       cout << "Picking" << SeqAcP[iCo+1]  << "and placing it on" << SeqAcP[iCo] << endl;
                                   Report<< "Picking" << SeqAcP[iCo+1]  << "and placing it on" << SeqAcP[iCo] << endl;								 
								   pickk=SeqAcP[iCo+1];
								   Placc=SeqAcP[iCo];
							   }
							   if(GiD==32){
							       cout << "Transporting Object " << SeqAcP[iCo] << "in the box" << endl;
								   Report << "Transporting Object " << SeqAcP[iCo] << "in the box" << endl;
								    pickk=SeqAcP[iCo];
								   Placc=SeqAcP[iCo+1];
							   }
								  
								  int picks = PickandPlace(pickk,Placc,iCo); //**************************************
							//   int picks=1; // loop break just for testing //***********************************
                            cout<<"PICKS Result"<< picks<<endl;
							if(picks==4) //VM Made change here 23/03
							   {
							      cout << "PMP target Unreachable or Object slippage: Attention:" << endl;
								  Report << "PMP target Unreachable or Object slippage: Attention:" << endl;
								  SeqAcP[iCo+1]=SeqAcP[iCo];  
							   } 

								 if(picks==0)
								   {
									   cout << "Sense failiures: Attention" << endl; 
									   Report << "Sense failiures: Attention" << endl; 
									   PtOfReplan=iCo;
                                       cout << "Failure sensed when picking" <<SeqAcP[PtOfReplan+1] << "and placing it on"<< SeqAcP[PtOfReplan]<< endl; 
									   Report << "Failure sensed when picking" <<SeqAcP[PtOfReplan+1] << "and placing it on"<< SeqAcP[PtOfReplan]<< endl; 
									   Replan=1;; //to exit loop
                                       iCo=NPiCs;
									   //state=3; // go back to refresh and contact reasoning
								    }
							   if(picks==1){ 
                               StakSuc=StakSuc+picks; 
							   cout << "Seems fine: Going ahead with the next micro sequence !!" << endl;  
							   Report << "Seems fine: Going ahead with the next micro sequence !!" << endl;  
							   }
							 }
					  //MAY BE WE MUST TAKE A SNAPSHOT AND MEASURE THE TOP MOST POINT: IN CASE OF STACK DESTRUCTION
					   cout << "Finished the Goal!! Anticipated reward from past experience is" << StakSuc+1 <<endl; 
					   Report << "Finished the Goal!! Anticipated reward from past experience is" << StakSuc+1 <<endl; 
					  //have to read the right value from Strata...
					  //Time::delay(60);
					   if(Replan==1){
					   state =3;
					   }
						if(Replan==0){
					   state = 0;
					   }

			    }

			break;

			case 3 : {
				cout<<" Sensed Unexpected hurdles: Monitoring where the past plan failed and contacting EPIM " <<endl;
				Report<<" Sensed Unexpected hurdles: Monitoring where the past plan failed and contacting EPIM " <<endl;
				    int iCo,jCo;
					for(iCo=0; iCo<10; iCo++){
					//  SeqAcP[iCo]=50;
					  PastPlan[iCo]=50;
					  }
					numberpast=0;
				//	findsuccess=0; //********************Need to uncomment this********VM1203
				    if((findsuccess==0)||(CannotFind==1)) //or grasp slippage or PMP fialiure
						{
							numberpast=PtOfReplan;
							for(iCo=0; iCo<PtOfReplan+1; iCo++){
							PastPlan[iCo]=SeqAcP[iCo];
							cout<< "Transfering half finished plan to Working memory...."<<PastPlan[iCo]<<endl;
							Report<< "Transfering half finished plan to Working memory...."<<PastPlan[iCo]<<endl;
							PPiCo=PastPlan[iCo];
							}
						}
					if((findsuccess2==0)&&(findsuccess!=0)) //failed during palceing operations
						{
							numberpast=PtOfReplan-1;
							for(iCo=0; iCo<PtOfReplan; iCo++){
							PastPlan[iCo]=SeqAcP[iCo];
							cout<< "Transfering half finished plan to Working memory...."<<PastPlan[iCo]<<endl;
							Report<< "Transfering half finished plan to Working memory...."<<PastPlan[iCo]<<endl;
						   }
                        }
					cout<< "Peceiving the messed up situation and contacting Reasoning" <<PastPlan[numberpast] << endl;
				   
					// A time delay may be needed here
					Time::delay(12);
				  	RefreshPlacemap(); //****** Need to Uncomment this for real testing: VM1203
					//***************************************************************************
						
					//***************************************************************************
					state=1;
					}
			break;

		}

		//Time::delay(5);          	     
	} 
}

void ObserverThread::threadRelease() {
    // nothing
	EpimCtrlPort.close();
     
}

void ObserverThread::Mergence(){
    int iCo, jCo, NoBReplan=0;
	int TempSeqAcP[10];
//	if(CannotFind==1){
//		NPiCs=NPiCs-1;
//		}
	for(iCo=0; iCo<NPiCs; iCo++)
			{ 
               TempSeqAcP[iCo]=SeqAcP[iCo];
			   cout<< "Interpreting New plan"<< TempSeqAcP[iCo] <<endl;
			   Report<< "Mergence Interpreting New plan"<< TempSeqAcP[iCo] <<endl;
			   SeqAcP[iCo]=50;
       	 }
        SeqAcP[0]=PPiCo;
		NoBReplan=NoBReplan+1;
		cout<< "Connecting element of past plan"<< SeqAcP[0] <<endl;
		Report<< "Connecting element of past plan"<< SeqAcP[0] <<endl;
		int present;
        for(iCo=0; iCo<NPiCs; iCo++)
			{
				present=0;
				for(jCo=0; jCo<numberpast+1; jCo++){
					if(PastPlan[jCo]==TempSeqAcP[iCo])
					{
					 cout<<"Object" << TempSeqAcP[iCo] << "is common"<< endl;
					 Report<<"Object" << TempSeqAcP[iCo] << "is common"<< endl;
					 present=1;
					} 
				}
				if((present==0)&&(TempSeqAcP[iCo]!=SeqAcP[0])){
                    cout<<"Using Object"<<TempSeqAcP[iCo]<<"in the new plan" <<endl;
					SeqAcP[NoBReplan]=TempSeqAcP[iCo];
					NoBReplan=NoBReplan+1;
				}
		    }
		NPiCs=NoBReplan;
		
}

int ObserverThread::PickandPlace(int pick, int place, int seqNumber) {

	int resPandP=0;
	double PMPRepl=0, PMPReplPlace=0,GraspO=-1, GraspC=-1, GraspR=-1,PMPReplInit=0,PMPReplMicro=0;
	CannotFind=-1, Cumulate=0;
	findsuccess=0;
	findsuccess2=0;
	if(GiD==14){
	/*findsuccess=PrimSearch(pick,place,5);
		if(findsuccess == 0)
		{
			findsuccess=PrimSearch(pick,place,5);
			if(findsuccess == 0)
			{
				findsuccess=PrimSearch(pick,place,5);
			}
		}*/ 
        cout<<" Waiting for findsuccess " <<endl;	 
		cin >> findsuccess;  //VM extended test 0104

	}
	if(GiD==32){
	findsuccess=PrimSearch(pick,place,68);
	}
	cout<<"FIND SUCCESS IS : "<<findsuccess<<endl;
	Report<<"FIND SUCCESS IS : "<<findsuccess<<endl;
	
//    int findsuccess=1;
	if (findsuccess==1)
		{
		  // 
			if(GiD==14){
			cout<< "Both Objects involved in the present micro sequnce are there " << endl;
			Report<< "Both Objects involved in the present micro sequnce are there "<<pick << place << endl;
			}
			cout<< "Contacting Grasp server to open gripper " << endl; //open
			Report<< "Contacting Grasp server to open gripper " << endl;
			Cumulate=Cumulate+1	;

    /*        GraspO=PrimGrasp(2); //commented 0603 // opening of gripper */
                            
			GraspO = 5;//commented 0603 *************************VM01/04
			if(GraspO==5)
				{
					cout<< "Contacting PMP server ........VOCAB REA " << endl; 
					Report<< "Contacting PMP server ........VOCAB REA " << endl; 
                    Cumulate=Cumulate+1	;   
					cout<<"THE OBJECT ID:  "<<GetObjIDs[0]<<std::endl;

				//	PMPRepl=PrimBodySchema(1,GetObjIDs[0],1,pick);// contains arguements for reach object pick;
                    cout<<" Waiting for PMPRepl " <<endl;	                   
					cin >>PMPRepl;

					cout<<"PMPRepl: "<<PMPRepl<<std::endl;
					Report<<"PMPRepl received from Prim Body schema is: "<<PMPRepl<<std::endl;
					//if this is sucessful, grasp the object, init the arm, find if the object is still there in the scene
					//if all these actions are sucessful
				}
			if(PMPRepl==1)
				{
                 cout<< "Contacting Grasp server to Grip the object " << endl; //open
				 Report<< "Contacting Grasp server to Grip the object " << endl; //open
				 Cumulate=Cumulate+1;
            /*     GraspC=PrimGrasp(1); //commented 0603 */ 
                  cout<< "Waiting for GraspC " << endl;
                  cin>>GraspC;
			    }
		////commented 0603
			//GraspC = 5;
			if(GraspC==5)
				{
                 GraspC=4;// added VM 23/03 
                 cout<< "Contacting PMP server to init and Check sucess of Pick " << endl; //open
				 Report<< "Contacting PMP server to init and Check sucess of Pick " << endl; //open
				 Cumulate=Cumulate+1;
				 //Time::delay(10);

             /*    PMPReplInit=PrimBodySchema(19,GetObjIDs[0],1,pick); */ //**********VM01/04

				 //Time::delay(10);
//*****************************23/03/: Modified this part so as not not route through vision..but Gripper instead.......
				 //**My also have to add one line in PMP Module..if necessary...
	/*			 PrimSearch(pick,place,68);
				 int checkDisp=GetObjIDs[0];
                 if ((checkDisp != 50) && (PlaceMap[checkDisp][cannotfindXLoc]!=cannotfindX)&&(GraspC==5))
				 {
				  CannotFind=0;
				 }
                 if(checkDisp==50)
                 {
				  CannotFind=0;
				 } */ 
                 
			//	 GraspC=PrimGrasp(1);  //checking closing again after arm has initialized..VM 23/03

				 GraspC=5; //VM01/04
				 if(GraspC==5){
                 CannotFind=0;
				 cout<<"Object is coupled to the gripper : "<<CannotFind<<endl;
				 Report<<"Object is coupled to the gripper : "<<CannotFind<<endl;
				 } 
				 
				 //you need to refresh placemap here and check if the picked object ID is no longer there
			    }

			//============================bug correct
			if(GraspC==4){
                 CannotFind=1;
				 cout<<"Object has slipped from the fingers : "<<CannotFind<<endl;
                 Report<<"Object has slipped from the fingers : "<<CannotFind<<endl;
				 } 
			//===========================================================

			//CannotFind = 0;
			if (CannotFind==0){
			//RefreshPlacemap();
            Cumulate=Cumulate+1	;
			if(GiD==14){
			//Time::delay(10);
		/*	findsuccess2=PrimSearch(place,pick,68);
			if(findsuccess2 == 0)
			{
				findsuccess2=PrimSearch(place,pick,68);
				if(findsuccess2 == 0)
				{
					findsuccess2=PrimSearch(place,pick,68);
				}
			}*/ 
            cout<<" Waiting for findsucess2 " <<endl;	  
            cin>>findsuccess2; //VM04/01
			cout<< "reaching object " << GetObjIDs[1]<<" , " << GetObjIDs[0] << endl;
			Report<< "reaching object to place " << GetObjIDs[1]<<" , " << GetObjIDs[0] << endl;
			}
			if(GiD==32){
			findsuccess2=1;
			cout<< "Placing object "<< endl;
			Report<< "Placing object "<< endl;
			}
				if(findsuccess2==1){
						 if(GiD==14){

					//	PMPReplPlace=PrimBodySchema(1,GetObjIDs[0],3,pick);// contains arguements for reach object Place;	//was place before
					    cout<<" Waiting for PMPReplPlace " <<endl;
						cin>>PMPReplPlace;
						//VM01/04
							 //if this is sucessful, Release the object, init the arm, find if the object with ID pick is now there in the scene
						//and approximately allined in the z-dimension with object place 
						 }
						 if(GiD==32){

							StaticLoc[0]=550;  //Check the start position to place
							StaticLoc[1]= 450;
							if(pick == 2)
							{
								StaticLoc[2]=125+90;
							}
							else
							{
								StaticLoc[2]=95+90;
							}

						  PMPReplPlace=PrimBodySchema(21,GetObjIDs[0],1,GetObjIDs[0]);// contains arguements for reach object Place;	
						//if this is sucessful, Release the object, init the arm, find if the object with ID pick is now there in the scene
						//and approximately allined in the z-dimension with object place 
						   

							//cout<<"LOCATION where I am placing object "<<pick<<
								//"is"<<StaticLoc[0]<<StaticLoc[1]<<StaticLoc[2]<<endl;
						 }
				   }
			} 

            if(PMPReplPlace==1)
			    {
					cout<< "Contacting Grasp server to stack/release " << endl; //open
					Report<< "Contacting Grasp server to stack/release " << endl; //open
					Cumulate=Cumulate+1	;
                   // GraspR=PrimGrasp(2); // releasing opening gripper 
                   GraspR=5;// 
			    }
			//GraspR = 1;
            if(GraspR==5)
			    {
				  
                  Cumulate=Cumulate+1;
				  Report<<"Entered initializing loop : "<<GraspR<<endl;
				   cout<<"Entered initializing loop : "<<GraspR<<endl;
			//	  PMPReplMicro=PrimBodySchema(19,GetObjIDs[1],1,GetObjIDs[1]); //init the arm and go back to the master to see what next
				  //VM01/04
				   
				   Report<<"out of PMPREPLMICRO : "<<PMPReplMicro<<endl;
				  cout<<"out of PMPREPLMICRO : "<<PMPReplMicro<<endl;
				  //Time::delay(10);
			    } 
        // here you get a cumulative score of sucess of vision, PMP and Grasp, 
		//this has to be fed back to the Observer to execute the next Usequnce
		}

	if (findsuccess==0)
		{
		cout<< "Sense failure in finding object to pick up" << endl; 
		Report<< "Sense failure in finding object to pick up" << endl; 
		Cumulate=0;
		}

	if ((findsuccess2==0)&&(findsuccess!=0))
		{
		 cout<<"Sense failure in finding object on which I need to place the picked object"<<endl; 
		 Report<<"Sense failure in finding object on which I need to place the picked object"<<endl; 
		Cumulate=0;
		}

	if ((PMPRepl==2)&&(findsuccess==1))
		{
		cout<<"Sense failure in reaching the object I need to pick up"<<endl; 
		Report<<"Sense failure in reaching the object I need to pick up"<<endl; 
		Cumulate=4;  // object to be picked is not reachable: Comment VM 23/03^
		}

	if ((PMPReplPlace==2)&&(PMPRepl==1)&&((CannotFind!=1)))
		{
		cout<<"Sense failure in Placing"<<endl;   
		Report<<"Sense failure in Placing"<<endl;   
		Cumulate=0;
		}

	if ((CannotFind==1)&&(PMPRepl==1))
		{
			cout<<"Sense failure: The scene is not refreshed/Object has slipped our: CannotFind"<<CannotFind<<endl;   
		Report<<"Sense failure: The scene is not refreshed/Object has slipped our"<<endl;   
		Cumulate=0; // object picked up has slipped  Comment VM/23/03
		}

	if ((PMPRepl==1)&&(PMPReplPlace==1)&&(CannotFind==0)&&(findsuccess==1)&&(findsuccess2==1)){
		Cumulate=1;
	}
    cout<<"CUMULATE SCORE"<<Cumulate<<endl;

	if(GiD==32){
 Cumulate=findsuccess;
 }

return Cumulate; // VM transformed the findsucess score into a cumualtive score that relates to all micro events
}

double ObserverThread::PrimBodySchema(int PMPGoalCode, int OIDinPM,int PIdentifier,int ObjectIDPMP){

        				
						 cout << "Inside the PrimBodySchema : " << endl;
						 Report << "Inside the PrimBodySchema : "  << endl;
                        int iCo=0, iCocord=-1,cannotfindXLoc=100;
						cannotfindX=0;
                        for(int i=2;i<18;i=i+3)
							{
							  if((PlaceMap[OIDinPM][i])>iCocord) //checks 2-5-8-11-14..
								  {
								     iCocord=PlaceMap[OIDinPM][i]; //z coordinate
                                     iCo=i;
								  }
							}  

						Bottle BodySchema_cmd, BodySchema_response;
						BodySchema_cmd.addVocab(COMMAND_VOCAB_REACH);
						BodySchema_cmd.addInt(PMPGoalCode);
						//the rest of information will be replaced by correct numbers coming from Place Map (that is a event driven 
						//working memory keeping track of what thngs are there and where they are in the world)
						if(PMPGoalCode==1){
							BodySchema_cmd.addDouble(PlaceMap[OIDinPM][iCo-2]); 
							cannotfindX=PlaceMap[OIDinPM][iCo-2]; 
							cannotfindXLoc=iCo-2;
							BodySchema_cmd.addDouble(PlaceMap[OIDinPM][iCo-1]);
							if(PIdentifier==1)
							{
								BodySchema_cmd.addDouble(PlaceMap[OIDinPM][iCo]);
							}
							if(PIdentifier==3)
							{
								if(ObjectIDPMP == 2)
								{
									BodySchema_cmd.addDouble(PlaceMap[OIDinPM][iCo]+123);
								}
								else
								{
									BodySchema_cmd.addDouble(PlaceMap[OIDinPM][iCo]+93);
								}
							}
						}
						if(PMPGoalCode==19){

						cout<<"Entered Goal of Initialization " <<endl;
						Report<<"Entered Goal of Initialization " <<endl;
						BodySchema_cmd.addDouble(XPosition[0]); 
						BodySchema_cmd.addDouble(XPosition[1]);
						BodySchema_cmd.addDouble(XPosition[2]);
						cout<<"Entered Goal of Initialization X1,X2,X3 : "<<XPosition[0]<<" , "<<XPosition[1]<<" , "<<XPosition[2] <<endl;
						Report<<"Entered Goal of Initialization X1,X2,X3 : "<<XPosition[0]<<" , "<<XPosition[1]<<" , "<<XPosition[2] <<endl;
						}
						if(PMPGoalCode==21){
						BodySchema_cmd.addDouble(StaticLoc[0]); 
						BodySchema_cmd.addDouble(StaticLoc[1]);
						BodySchema_cmd.addDouble(StaticLoc[2]);
						}
						BodySchema_cmd.addDouble(0);
						BodySchema_cmd.addDouble(0);
						BodySchema_cmd.addDouble(0);
						// for ComputeTheta
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][0]); 
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][1]); 
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][3]);
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][4]);
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][6]); 
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][7]); 
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][9]);
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][10]);
						BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);

						cout<<"Sent a request to PMP Server with PMPGoalCode : "<<PMPGoalCode<<endl;
						Report<<"Sent a request to PMP Server with PMPGoal Code : "<<PMPGoalCode<<endl;
						printf("%s \n",BodySchema_response.toString().c_str());
	
						double Bresponsecode = BodySchema_response.get(0).asDouble();
					    cout<<Bresponsecode<<endl;

					if(Bresponsecode == 221 /*COMMAND_VOCAB_REACH*/) {
						 cout << "Receiving status from Body Schema PMP server" << endl;
						 Report << "Receiving status from Body Schema PMP server" << endl;
						 for(int i=0;i<10;i++)
							{
							 PMPresp[i]= BodySchema_response.get(i+1).asDouble();
							 cout << "Resp from PMP server" << PMPresp[i]<< endl;
							}
						 XPosition[0]=PMPresp[1];
						 XPosition[1]=PMPresp[2];
						 XPosition[2]=PMPresp[3];
						 cout<<" Forward model output of arm position"<< endl;
						 cout<< XPosition[0] << XPosition[1] << XPosition[2] << endl;
						 if(PMPresp[0]==1){
						 cout << "Reached goal object sucessfully: wait for next goal from client" << endl;
						 Report << "Reached goal object sucessfully: wait for next goal from client" << endl;
						 }

						 if(PMPresp[0]==0){
							 cout << "Goal is not doable: need to form a updated plan with help of EPIM" << endl;
						 }
					}
return PMPresp[0];
}


double ObserverThread::PrimSearch(int obj1, int obj2, int goalidentity){

	RefreshPlacemap(); // connect to the bottle that brings in visual information and refresh the place map
	int iCo, jCo, niterFind=1, goalFind[2];
	double FindResp=0;
	if(goalidentity==68){
	  goalFind[0]=obj1;
	}
	if (goalidentity==5){
		niterFind=2;
		goalFind[0]=obj1;
		goalFind[1]=obj2;
	}
    for(iCo=0; iCo<niterFind; iCo++)
			{
				GetObjIDs[iCo]=50;
				for(jCo=0; jCo<NumObjectsinScene; jCo++)
						 {
						   if(ObjIDEE[jCo]==goalFind[iCo])    
						   GetObjIDs[iCo]=jCo;
						 }
				if(niterFind==1)
				{
				  if(GetObjIDs[iCo]!= 50)
					  {
					   FindResp=1;
					   cout <<"object is there" << endl;
					  }
				}

				if(niterFind==2)
				{
				  if(GetObjIDs[iCo]!= 50)
					  {
					   FindResp=FindResp+0.5;
					   cout <<"object" << iCo << "is there" << endl;
					  }
				}

			}
   return FindResp;
}

int ObserverThread::PrimGrasp(int GraspReq){
                         
						Bottle Grasp_cmd, Grasp_response;
						Grasp_cmd.addInt(GraspReq);
						GraspPort.write(Grasp_cmd,Grasp_response);
						printf("%s \n",Grasp_response.toString().c_str());
	
						int Gresponsecode = Grasp_response.get(0).asInt();
					    cout<< "Reply recd from Grasp server" << Gresponsecode<<endl;
						Report<< "Reply recd from Grasp server" << Gresponsecode<<endl;
						return Gresponsecode;
}

double ObserverThread::RefreshPlacemap(){

    
  Bottle OPC_cmd, OPC_Response;

			 OPC_cmd.addVocab(COMMAND_VOCAB_FIND);
			 OPCCtrlPort.write(OPC_cmd,OPC_Response);
			 printf("%s \n",OPC_Response.toString().c_str());

			 for(int i=0;i<10;i++)
					{
						for(int j=0;j<18;j++)
							{
							  PlaceMap[i][j]=0;
							}
					  }
				 for(int i=0;i<10;i++)
					{
						ObjIDEE[i]=50; //null object
						XlatTrackP[i]=50;
						XlatTrackPl[i]=50;
					}
				 NumberofObs=0;
				 int ctrr=0;
	//==========================================================================================
              int OPCresponsecode = OPC_Response.get(0).asVocab();
			   ctrr=ctrr+1;
			  cout<<OPCresponsecode<<endl;

					if(OPCresponsecode == 428 /*COMMAND_VOCAB_FIND*/) {
						 cout << "Receiving status from OPC server" << endl;
						 Report << "Receiving status from OPC server" << endl;

//=============================================================================================
						 NumberofObs = OPC_Response.get(ctrr).asInt();
						 NumObjectsinScene=NumberofObs;
							   ctrr=ctrr+1;
							   for(int i=0;i<NumberofObs;i++)
									{
										ObjIDEE[i]=OPC_Response.get(ctrr).asInt();
										XlatTrackP[i]=ObjIDEE[i];
										XlatTrackPl[i]=ObjIDEE[i]; //buggy things are there
										cout << "Object ID received from OPC server" << ObjIDEE[i] << endl;
										Report << "Object ID received from OPC server" << ObjIDEE[i] << endl;
                                        ctrr=ctrr+1; 
									}	 

								for(int i=0;i<NumberofObs;i++)
									{
       									for(int j=0;j<18;j++)
											{
												PlaceMap[i][j]=OPC_Response.get(ctrr).asDouble();
												ctrr=ctrr+1; 
											}	
								     }
//==============================================================================================

					}
	return NumberofObs;
}

void ObserverThread::Interpret(){
    
	NPiCs=0;
	for(int iCo=1; iCo<SeqAc[0]+1; iCo++)	{ 

				if((SeqAc[iCo]==20))
							   {
								   for(int jCo=0; jCo<numcu; jCo++){
									SeqAcP[NPiCs]=NumCubID[jCo];
									cout<< "found cube" <<SeqAcP[NPiCs] <<endl;
									NPiCs=NPiCs+1;
				                     }
							   }
				if((SeqAc[iCo]==13))
							   {
								   for(int jCo=0; jCo<numcy; jCo++){
									SeqAcP[NPiCs]=NumCylID[jCo];
									cout<< "found cyli" <<SeqAcP[NPiCs] <<endl;
									NPiCs=NPiCs+1;
				                     }
							   }
				if((SeqAc[iCo]==18))
								{
								 SeqAcP[NPiCs]=7;
								 NPiCs=NPiCs+1;
								}
				if((SeqAc[iCo]==35))
								{
								 SeqAcP[NPiCs]=NumMushID[0];
								 NPiCs=NPiCs+1;
								}
	
	}
	cout<<"NPiCs Interpreter"<<NPiCs<<endl;

}



void ObserverThread::Xlator(){
            largeness=0;
			numcu=0;
			numcy=0;
			nummu=0;
			NumCylID[0]=50;
			NumCylID[1]=50;
			NumCubID[0]=50;
			NumCubID[1]=50;
			NumCubID[2]=50;
			NumberofObsE=0;
			int delnumcy=0,delnumcu=0;
			int CountEP=0;
			for(int i=0;i<NumberofObs;i++)
				{
					if ((ObjIDEE[i]==2)||(ObjIDEE[i]==5))
					   {
						   if(nummu==0){
							ObjIDEEEpim[CountEP]=35; //Mushroom
							CountEP=CountEP+1;
							}
						   XlatTrackP[i]=ObjIDEE[i];
						   XlatTrackPl[i]=ObjIDEE[i];
						   NumMushID[nummu]=ObjIDEE[i];
                           nummu=nummu+1;
							if(nummu==1)
							{
								NumberofObsE=NumberofObsE+1;
							}
					   }
					if ((ObjIDEE[i]==7))
					   {
						ObjIDEEEpim[CountEP]=18; //Large object
						//ObjIDEEEpim[NumberofObs+1]=37;
						CountEP=CountEP+1;
						XlatTrackP[i]=ObjIDEE[i];
						XlatTrackPl[i]=ObjIDEE[i];
						largeness=1;
						NumberofObsE=NumberofObsE+1;
						cout<<"Large object"<<endl;
					   }
					if ((ObjIDEE[i]==3)||(ObjIDEE[i]==6))
					   {
							if(numcy==0){
							ObjIDEEEpim[CountEP]=13; //Cylinder
							CountEP=CountEP+1;
							}
						XlatTrackP[i]=ObjIDEE[i];
						XlatTrackPl[i]=ObjIDEE[i];
						NumCylID[numcy]=ObjIDEE[i];
                        numcy=numcy+1;
							if(numcy==1)
							{
								NumberofObsE=NumberofObsE+1;
							}
					   }

					if ((ObjIDEE[i]==0)||(ObjIDEE[i]==1)||(ObjIDEE[i]==4))
					   {
						   if(numcu==0){
							ObjIDEEEpim[CountEP]=20; //Cube
							CountEP=CountEP+1;
						   }
						XlatTrackP[i]=ObjIDEE[i];
						XlatTrackPl[i]=ObjIDEE[i];
						NumCubID[numcu]=ObjIDEE[i];
                        numcu=numcu+1;
								if(numcu==1)
								{
								   NumberofObsE=NumberofObsE+1;
								}
						cout<<"Numcu"<< numcu << "NumCubID[0]" << NumCubID[0]<< endl;
					   }
			}
			cout<<"No of objects in the abstract neural representation:" << NumberofObsE << endl;

	}

void ObserverThread::InvXlator(int pi, int pl){
 
	PickMicro=50;
    PlaceMicro=50;
	if(pi==35){
	for(int i=0;i<NumberofObs;i++)
				{
					if (XlatTrackP[i]==2)
						{
						  PickMicro=2;
						  XlatTrackP[i]=50;
						}
	            }
	}

	if(pi==18){
		{
           PickMicro=6;
	     }
	}

	if(pi==13){
	for(int i=0;i<NumberofObs;i++)
				{
                   if ((XlatTrackP[i]==3)||(XlatTrackP[i]==5))
						{
						  PickMicro=XlatTrackP[i];
						  XlatTrackP[i]=50;
						  i=10;
						}
	            }
	}

	if(pi==20){
	for(int i=0;i<NumberofObs;i++)
				{
                  if ((XlatTrackP[i]==0)||(XlatTrackP[i]==1)||(XlatTrackP[i]==4))
						{
						  PickMicro=XlatTrackP[i];
						  XlatTrackP[i]=50;
						  i=10;
						}

	            }
	}
//================================================================================
	if(pl==35){
	for(int i=0;i<NumberofObs;i++)
				{
					if (XlatTrackPl[i]==2)
						{
						  PlaceMicro=2;
						  XlatTrackPl[i]=50;
						}
	            }
	}

	if(pl==18){
		{
           PlaceMicro=6;
	     }
	}

	if(pl==13){
	for(int i=0;i<NumberofObs;i++)
				{
                   if ((XlatTrackPl[i]==3)||(XlatTrackPl[i]==5))
						{
						  PlaceMicro=XlatTrackPl[i];
						  XlatTrackPl[i]=50;
						  i=10;
						}
	            }
	}

	if(pl==20){
	for(int i=0;i<NumberofObs;i++)
				{
                  if ((XlatTrackPl[i]==0)||(XlatTrackPl[i]==1)||(XlatTrackPl[i]==4))
						{
						  PlaceMicro=XlatTrackPl[i];
						  XlatTrackPl[i]=50;
						  i=10;
						}

	            }
	}

}

void ObserverThread::onStop() {
    
//    outputPort.interrupt();
  //  outputPort.close();
}




