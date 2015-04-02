
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/Drivers.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <iCub/ctrl/math.h>
#include <iCub/action/actionPrimitives.h>
#include <iostream>
#include <fstream>
#include "fumblyModule.h"

using namespace std;

#define CMD_CALI                   VOCAB4('c','a','l','i')
#define CMD_FIDL                   VOCAB4('f','i','d','l')

YARP_DECLARE_DEVICES(icubmod)


fumblyModule::fumblyModule()
{
    computePalmOrientations();

    graspOrienL.resize(4);    graspOrienR.resize(4);
    graspDispL.resize(4);     graspDispR.resize(3);
    dOffsL.resize(3);         dOffsR.resize(3);
    dLiftL.resize(3);         dLiftR.resize(3);
    home_xL.resize(3);        home_xR.resize(3);

    // default values for arm-dependent quantities
    graspOrienL[0]=-0.171542; graspOrienR[0]=-0.0191;
    graspOrienL[1]= 0.124396; graspOrienR[1]=-0.983248;
    graspOrienL[2]=-0.977292; graspOrienR[2]= 0.181269;
    graspOrienL[3]= 3.058211; graspOrienR[3]= 3.093746;

    graspDispL[0]= 0.0;       graspDispR[0]= 0.0;
    graspDispL[1]= 0.0;       graspDispR[1]= 0.0;
    graspDispL[2]= 0.05;      graspDispR[2]= 0.05;

    dOffsL[0]=-0.03;          dOffsR[0]=-0.03;
    dOffsL[1]=-0.07;          dOffsR[1]=-0.07;
    dOffsL[2]=-0.02;          dOffsR[2]=-0.02;

    dLiftL[0]= 0.0;           dLiftR[0]= 0.0;
    dLiftL[1]= 0.0;           dLiftR[1]= 0.0;
    dLiftL[2]= 0.15;          dLiftR[2]= 0.15;

    home_xL[0]=-0.29;         home_xR[0]=-0.29;
    home_xL[1]=-0.21;         home_xR[1]= 0.24;
    home_xL[2]= 0.11;         home_xR[2]= 0.07;
    /*
    home_xL[0]=-0.3;         home_xR[0]=-0.3;
    home_xL[1]=-0.24;         home_xR[1]= 0.24;
    home_xL[2]= 0.3;         home_xR[2]= 0.3;    
*/
    action=actionL=actionR=NULL;
    graspOrien=NULL;
    graspDisp=NULL;
    dOffs=NULL;
    dLift=NULL;
    home_x=NULL;

    openPorts=false;
    firstRun=true;
}

void fumblyModule::getArmDependentOptions(Bottle &b, Vector &_gOrien, Vector &_gDisp,
                                Vector &_dOffs, Vector &_dLift, Vector &_home_x)
{
    if (Bottle *pB=b.find("grasp_orientation").asList())
    {
        int sz=pB->size();
        int len=_gOrien.length(); 
        int l=len<sz?len:sz;

        for (int i=0; i<l; i++)
            _gOrien[i]=pB->get(i).asDouble();
    }

    if (Bottle *pB=b.find("grasp_displacement").asList())
    {
        int sz=pB->size();
        int len=_gDisp.length();
        int l=len<sz?len:sz;

        for (int i=0; i<l; i++)
            _gDisp[i]=pB->get(i).asDouble();
    }

    if (Bottle *pB=b.find("systematic_error_displacement").asList())
    {
        int sz=pB->size();
        int len=_dOffs.length();
        int l=len<sz?len:sz;

        for (int i=0; i<l; i++)
            _dOffs[i]=pB->get(i).asDouble();
    }

    if (Bottle *pB=b.find("lifting_displacement").asList())
    {
        int sz=pB->size();
        int len=_dLift.length();
        int l=len<sz?len:sz;
 
        for (int i=0; i<l; i++)
            _dLift[i]=pB->get(i).asDouble();
    }

    if (Bottle *pB=b.find("home_position").asList())
    {
        int sz=pB->size();
        int len=_home_x.length();
        int l=len<sz?len:sz;

        for (int i=0; i<l; i++)
            _home_x[i]=pB->get(i).asDouble();
    }
}

bool fumblyModule::configure(ResourceFinder &rf)
{
    string name=rf.find("name").asString().c_str();
    setName(name.c_str());

    partUsed=rf.check("part",Value("both_arms")).asString().c_str();
    if (partUsed!="both_arms" && partUsed!="left_arm" && partUsed!="right_arm")
    {
        cout<<"Invalid part requested !"<<endl;
        return false;
    }

    Property config; config.fromConfigFile(rf.findFile("from").c_str());
    Bottle &bGeneral=config.findGroup("general");
    if (bGeneral.isNull())
    {
        cout<<"Error: group general is missing!"<<endl;
        return false;
    }

    // parsing general config options
    Property option;
    for (int i=1; i<bGeneral.size(); i++)
    {
        Bottle *pB=bGeneral.get(i).asList();
        if (pB->size()==2)
            option.put(pB->get(0).asString().c_str(),pB->get(1));
        else
        {
            cout<<"Error: invalid option!"<<endl;
            return false;
        }
    }

    option.put("local",name.c_str());
    option.put("hand_sequences_file",rf.findFile("hand_sequences_file"));

        // get table configuration
        tableMan.load(rf.findFile("table_file").c_str());

        // init table parameters transmission
        tableMan.initTxParams(("/"+name+"/table:o").c_str(),
                              rf.find("homography_port").asString().c_str());


    Property optionL(option); optionL.put("part","left_arm");
    Property optionR(option); optionR.put("part","right_arm");

    // parsing left_arm config options
    Bottle &bLeft=config.findGroup("left_arm");
    if (bLeft.isNull())
    {
        cout<<"Error: group left_arm is missing!"<<endl;
        return false;
    }
    else
        getArmDependentOptions(bLeft,graspOrienL,graspDispL,
                               dOffsL,dLiftL,home_xL);

    // parsing right_arm config options
    Bottle &bRight=config.findGroup("right_arm");
    if (bRight.isNull())
    {
        cout<<"Error: group right_arm is missing!"<<endl;
        return false;
    }
    else
        getArmDependentOptions(bRight,graspOrienR,graspDispR,
                               dOffsR,dLiftR,home_xR);

    if (partUsed=="both_arms" || partUsed=="left_arm")
    {
        cout<<"***** Instantiating primitives for left_arm"<<endl;
        actionL=new AFFACTIONPRIMITIVESLAYER(optionL);

        if (!actionL->isValid())
        {
            delete actionL;
            return false;
        }
        else
            useArm(USE_LEFT);
    }

    if (partUsed=="both_arms" || partUsed=="right_arm")
    {
        cout<<"***** Instantiating primitives for right_arm"<<endl;
        actionR=new AFFACTIONPRIMITIVESLAYER(optionR);

        if (!actionR->isValid())
        {
            delete actionR;

            // remind to check to delete the left as well (if any)
            if (actionL)
                delete actionL;

            return false;
        }
        else
            useArm(USE_RIGHT);
    }

    deque<string> q=action->getHandSeqList();
    cout<<"***** List of available hand sequence keys:"<<endl;
    for (size_t i=0; i<q.size(); i++)
        cout<<q[i]<<endl;

    string fwslash="/";
    inPort.open((fwslash+name+"/in").c_str());
    rpcPort.open((fwslash+name+"/rpc").c_str());
    attach(rpcPort);//makes it use respond()

    openPorts=true;

    return true;
}

bool fumblyModule::close()
{
    if (actionL!=NULL)
      delete actionL;

    if (actionR!=NULL)
        delete actionR;

    if (openPorts)
    {
        inPort.close();
        rpcPort.close();
    }

    return true;
}

double fumblyModule::getPeriod()
{
    return 0.1;
}

void fumblyModule::useArm(const int arm)
{
    if (arm==USE_LEFT)
    {
        action=actionL;
	armToBeUsed="left";

        graspOrien=&graspOrienL;
        graspDisp=&graspDispL;
        dOffs=&dOffsL;
        dLift=&dLiftL;
        home_x=&home_xL;
    }
    else if (arm==USE_RIGHT)
    {
        action=actionR;
	armToBeUsed="right";

        graspOrien=&graspOrienR;
        graspDisp=&graspDispR;
        dOffs=&dOffsR;
        dLift=&dLiftR;
        home_x=&home_xR;
    }
}

void fumblyModule::init()
{
    bool f;

    if (partUsed=="both_arms" || partUsed=="right_arm")
    {
        useArm(USE_RIGHT);
        //action->pushAction(*home_x,"open_hand");
	action->pushAction(*home_x,"karate_hand");
        action->checkActionsDone(f,true);
        action->enableArmWaving(*home_x);
    }

    if (partUsed=="both_arms" || partUsed=="left_arm")
    {
        useArm(USE_LEFT);
	//action->pushAction(*home_x,"open_hand");
        action->pushAction(*home_x,"karate_hand");
        action->checkActionsDone(f,true);
        action->enableArmWaving(*home_x);
    }
}

// we don't need a thread since the actions library already
// incapsulates one inside dealing with all the tight time constraints
bool fumblyModule::respond(const Bottle &command, Bottle &reply){
    printf("respond says aloha!\n");
    if(command.get(0).asVocab()==CMD_FIDL)
    {
        printf("respond says fiddle!\n");
//            Vector xd=retrieveTargetAndPrepareArm(command);
        Vector xd(3);
        bool f;
        xd[0]=command.get(1).asDouble();
        xd[1]=command.get(2).asDouble();
        xd[2]=command.get(3).asDouble();
        // switch only if it's allowed
        
        // apply systematic offset
        // due to uncalibrated kinematic
        xd=xd+*dOffs;
        // safe thresholding
        xd[0]=xd[0]>-0.1?-0.1:xd[0];

        //final orientation theta
        double theta = command.get(4).asDouble(); //starting theta
        double rStart = command.get(5).asDouble(); //starting radius
        double ftheta = command.get(6).asDouble(); //final theta
        double rStop = command.get(7).asDouble(); //ending radius
        double execTime = command.get(8).asDouble(); //time of execution

if (partUsed=="both_arms")
        {
	  /** djd not using - better to work from orientation 
            if (xd[1]>0.0)
                useArm(USE_RIGHT);
            else
                useArm(USE_LEFT);
	    */
	    
	    //if (theta>=-M_PI/4&&theta<=3*M_PI/4)//okay rotations? let's see
	    if (theta>=-M_PI/2&&theta<=M_PI/2)//okay rotations? let's see
                useArm(USE_RIGHT);
            else
                useArm(USE_LEFT);
        }	
	
        if (norm(xd)<(Value(1e9)).asDouble())
        {
            fiddle(xd,theta,rStart,ftheta,rStop,execTime);
        }
        else
        {
            cout<<"########## Target out of range ... pointing"<<endl;
//                point(xd);
        }

        action->checkActionsDone(f,true);
        reply.addVocab(VOCAB4('o','k','a','y'));
    } else if(command.get(0).asVocab()==CMD_CALI) {
        printf("respond says calibrate!\n");
//////////////////////////////////////////////////////////////
                    {    
                        Vector x0(3), x1(3);
                        x0[0]=x1[0]=Rand::scalar(-0.35,-0.30);
                        x0[1]=x1[1]=Rand::scalar(-0.05,0.05);
                        x0[2]=0.1; x1[2]=-0.2;

                        // switch only if it's allowed
                        if (partUsed=="both_arms")
                        {
                            if (x1[1]>0.0)
                                useArm(USE_RIGHT);
                            else
                                useArm(USE_LEFT);
                        }

//                        running=true;
                        bool f=false;
    
                        action->pushAction(x0,*graspOrien,"open_hand");
                        action->checkActionsDone(f,true);
                        action->latchWrenchOffset();
                        action->enableContactDetection();
                        action->pushWaitState(1.0);
                        action->pushAction(x1,*graspOrien,3.0);
                        action->checkActionsDone(f,true);
                        action->pushWaitState(2.0);
                        action->disableContactDetection();
                        action->checkContact(f);

                        if (f)
                        {
                            Vector x,o;
                            action->getPose(x,o);
                            double tableHeight=x[2];

                            cout<<"########## Table height found = "<<tableHeight<<endl;
                            tableMan.setTableHeight(tableHeight);
                        }
                        else
                            cout<<"########## Table not found"<<endl;

                        action->pushAction(x0,*graspOrien,HOMING_PERIOD);
                        action->pushAction(*home_x,HOMING_PERIOD);
                        action->checkActionsDone(f,true);
                        goHome();

    //                    running=false;
                    }


//////////////////////////////////////////////////////////////
        bool f;
        action->checkActionsDone(f,true);
        reply.addVocab(VOCAB4('o','k','a','y'));
    } else reply.addVocab(VOCAB4('f','a','i','l'));
    //rpcPort.reply(out); <-- RFModule does this
    return true;
}

bool fumblyModule::updateModule()
{
    // do it only once
    if (firstRun)
    {
        init();
        firstRun=false;
    }
    //printf("aloha (ah, ah, ah, ah, stayin' alive)\n");

return true;
}

bool fumblyModule::interruptModule()
{
    // since a call to checkActionsDone() blocks
    // the execution until it's done, we need to
    // take control and exit from the waiting state
action->syncCheckInterrupt(true);

    inPort.interrupt();
    rpcPort.interrupt();

return true;
}

//void fumblyModule::fiddle(const Vector &xd,  double &theta = 2*M_PI, const double &rStart = 0.2, double &ftheta = 0.0, const double &rStop = 0.1, const double &execTime = 1.0)
void fumblyModule::fiddle(const Vector &xd,  double theta , const double &rStart, double ftheta, const double &rStop, const double &execTime)
{
  
//  //theta=theta+M_PI;
 // ftheta=ftheta+M_PI;
    //double runOffset=(armToBeUsed=="right"?0.1:-0.1);

    double heightOffset=0.15;
    double prepareOffset=0.08;
    bool f=false;
    Vector startPos=xd;
    startPos[0]+=rStart*sin(theta);
    startPos[1]+=rStart*cos(theta);
    startPos[2]+=heightOffset;

    Vector startPosPrepare=startPos;startPosPrepare[2]+=prepareOffset;
    
    Vector endPos=xd;
    //endPos[0]-=rStop*sin(theta+2*M_PI);
    //endPos[1]-=rStop*cos(theta+2*M_PI);
    endPos[0]-=rStop*sin(theta);
    endPos[1]-=rStop*cos(theta);
    endPos[2]+=heightOffset;
    
    Vector endPosDisengage=endPos;endPosDisengage[2]+=prepareOffset;
    std::cout<<"HABERI,VIPI,MAMBO"<<std::endl;
    Vector startOrientation(4),stopOrientation(4);
    //startOrientation.clear();stopOrientation.clear();
    if(true){
      Matrix startAt=(palmOrientations[armToBeUsed+"_starttap"]);
      std::cout<<"startAt="<<startAt.toString()<<std::endl;
      Matrix endAt=(palmOrientations[armToBeUsed+"_stoptap"]);
      //std::cout<<"startAt="<<startAt.toString()<<std::endl;
      
      Vector doThetaInitial(4);
      Vector doThetaFinal(4);
      
      doThetaInitial.zero();
      doThetaFinal.zero();
      
      doThetaInitial[2]=1;
	doThetaFinal[2]=1;
	
	doThetaInitial[3]=theta;
	doThetaFinal[3]=theta;
	
	if(armToBeUsed=="left"){
	  doThetaInitial[3]+=M_PI;
	  doThetaFinal[3]+=M_PI;	  
	}

	    std::cout<<"axis2dcm(doThetaInitial)"<<axis2dcm(doThetaInitial).toString()<<std::endl;
	    std::cout<<"axis2dcm(doThetaInitial)*startAt"<<(axis2dcm(doThetaInitial)*startAt).toString()<<std::endl;	    
	  startOrientation=dcm2axis(axis2dcm(doThetaInitial)*startAt);
	    std::cout<<"axis2dcm(doThetaFinal)"<<axis2dcm(doThetaFinal).toString()<<std::endl;
	    std::cout<<"axis2dcm(doThetaFinal)*startAt"<<(axis2dcm(doThetaFinal)*startAt).toString()<<std::endl;	    	  
	  stopOrientation=dcm2axis(axis2dcm(doThetaFinal)*endAt);
	
/*	
	if(armToBeUsed=="left"){
	  if(true){
	    std::cout<<"axis2dcm(dothetaleft)"<<axis2dcm(dothetaleft).toString()<<std::endl;
	    std::cout<<"axis2dcm(dothetaleft)*startAt"<<(axis2dcm(dothetaleft)*startAt).toString()<<std::endl;	    
	  startOrientation=dcm2axis(axis2dcm(dothetaleft)*startAt);
	  stopOrientation=dcm2axis(axis2dcm(dothetaFinalleft)*endAt);
	  }else{
	  startOrientation=dcm2axis(startAt);
	  stopOrientation=dcm2axis(endAt);
	  }
	}
	else if(armToBeUsed=="right"){
	  if(true){
	    std::cout<<"axis2dcm(dothetaright)"<<axis2dcm(dothetaright).toString()<<std::endl;
	    std::cout<<"axis2dcm(dothetaright)*startAt"<<(axis2dcm(dothetaright)*startAt).toString()<<std::endl;
	  startOrientation=dcm2axis(axis2dcm(dothetaright)*startAt);
	  stopOrientation=dcm2axis(axis2dcm(dothetaFinalright)*endAt);
	  }else{
	  startOrientation=dcm2axis(startAt);
	  stopOrientation=dcm2axis(endAt);
	  }
	}else{
	 std::cout<<"NO ARM"<<std::endl; 
	}*/
    }
    else if(false){

    }
    //Vector startOrientation=dcm2axis(palmOrientations[armToBeUsed+"_starttap"]);
    //Vector stopOrientation=dcm2axis(palmOrientations[armToBeUsed+"_stoptap"]);
//  Vector startOrientation=dcm2axis(palmOrientations[armToBeUsed+"_starttap"]);
//  Vector stopOrientation=dcm2axis(palmOrientations[armToBeUsed+"_stoptap"]);
if(false){
    //this is wrong beccause we actually need to multiply this in to the start pos.
    startOrientation[2] = 1;
    startOrientation[3] = theta;

    
    stopOrientation[2] = 1;
    stopOrientation[3] = ftheta;
}

std::cout<<"startOrientation="<<startOrientation.toString().c_str()<<std::endl;
std::cout<<"stopOrientation="<<startOrientation.toString().c_str()<<std::endl;
//Time::delay(2);
    //std::cout<<"action->tap("<<startPos<<","<<startOrientation<<","<<endPos<<","<<stopOrientation<<","<<execTime<<");"<<std::endl;
    //action->tap(startPos,startOrientation,endPos,stopOrientation,execTime);

    action->pushAction(startPosPrepare,startOrientation, "karate_hand",execTime);
    action->pushWaitState(0.1);
    action->pushAction(startPos,startOrientation, "karate_hand",execTime);
    action->pushWaitState(0.1);
    action->pushAction(endPos,stopOrientation, "karate_hand",execTime);
    action->pushWaitState(0.1);
    action->pushAction(endPosDisengage,stopOrientation, "karate_hand",execTime);
    //action->pushAction(*home_x,dcm2axis(palmOrientations[armToBeUsed+"_down"]),"open_hand",HOMING_PERIOD);
//    Vector out = dcm2axis(palmOrientations[armToBeUsed+"_down"]);
//    std::cout<<"printing out out vector "<<out.toString().c_str()<<std::endl;
    
    //action->pushAction(*home_x,dcm2axis(palmOrientations[armToBeUsed+"_down"]),"open_hand",HOMING_PERIOD);
    action->checkActionsDone(f,true);
    goHome();
}

void fumblyModule::goHome()
{
  std::cout<<"GOING HOME"<<std::endl;
    bool f;
    // regardless of the current settings
    // go home in tracking mode

    if (partUsed=="both_arms" || partUsed=="right_arm")
    {
        useArm(USE_RIGHT);
            armToBeUsed="right";

        //action->pushAction(*home_x,"open_hand",HOMING_PERIOD);
	action->pushAction(*home_x,"karate_hand",HOMING_PERIOD);
	//action->pushAction(*home_x,dcm2axis(palmOrientations[armToBeUsed+"_starttap"]),"karate_hand",HOMING_PERIOD);
        action->checkActionsDone(f,true);
        action->enableArmWaving(*home_x);
    }

    if (partUsed=="both_arms" || partUsed=="left_arm")
    {
        useArm(USE_LEFT);
            armToBeUsed="left";

        //action->pushAction(*home_x,"open_hand",HOMING_PERIOD);
	action->pushAction(*home_x,"karate_hand",HOMING_PERIOD);
	//action->pushAction(*home_x,dcm2axis(palmOrientations[armToBeUsed+"_starttap"]),"karate_hand",HOMING_PERIOD);
        action->checkActionsDone(f,true);
        action->enableArmWaving(*home_x);
    }
}

// get a target object position from a YARP port
/*********************
    // lift the object (wait until it's done)
  action->pushAction(xd+*dLift,*graspOrien);
    action->checkActionsDone(f,true);
    action->pushAction("open_hand");
    action->checkActionsDone(f,true);
    action->enableArmWaving(*home_x);
**********************/

void fumblyModule::computePalmOrientations()
    {
        Matrix Ry(3,3);
        Ry.zero();
        Ry(0,0)=cos(M_PI);
        Ry(0,2)=sin(M_PI);
        Ry(1,1)=1.0;
        Ry(2,0)=-Ry(0,2);
        Ry(2,2)=Ry(0,0);
        
        //palmOrientations["right_down"]=Ry;
        palmOrientations.insert( pair<string, Matrix>("right_down",Ry) );

        Matrix Rx(3,3);
        Rx.zero();
        Rx(0,0)=1.0;
        Rx(1,1)=cos(M_PI);
        Rx(1,2)=-sin(M_PI);
        Rx(2,1)=-Rx(1,2);
        Rx(2,2)=Rx(1,1);
        
//        palmOrientations["left_down"]=Ry*Rx;
        palmOrientations.insert( pair<string, Matrix>("left_down",Ry*Rx) );

        Rx(1,1)=cos(M_PI/2.0);
        Rx(1,2)=-sin(M_PI/2.0);
        Rx(2,1)=-Rx(1,2);
        Rx(2,2)=Rx(1,1);
        
//        palmOrientations["rightssta"]=palmOrientations["right_down"]*Rx;
        palmOrientations.insert( pair<string, Matrix>("right_base",palmOrientations["right_down"]*Rx) );

        Rx(1,1)=cos(-M_PI/2.0);
        Rx(1,2)=-sin(-M_PI/2.0);
        Rx(2,1)=-Rx(1,2);
        Rx(2,2)=Rx(1,1);
        
//        palmOrientations["left_base"]=palmOrientations["left_down"]*Rx;
        palmOrientations.insert( pair<string, Matrix>("left_base",palmOrientations["left_down"]*Rx) );

        Matrix Rz(3,3);
        Rz.zero();
        Rz(0,0)=cos(M_PI/4.0);
        Rz(0,1)=-sin(M_PI/4.0);
        Rz(1,0)=-Rz(0,1);
        Rz(1,1)=Rz(0,0);
        Rz(2,2)=1.0;
        
        //palmOrientations["right_starttap"]=palmOrientations["right_base"];
		palmOrientations.insert( pair<string, Matrix>("right_starttap",palmOrientations["right_base"]) );
        //palmOrientations["left_starttap"] =palmOrientations["left_base"];
  		palmOrientations.insert( pair<string, Matrix>("left_starttap",palmOrientations["left_base"]) );

        Rx(1,1)=cos(M_PI/8.0);
        Rx(1,2)=-sin(M_PI/8.0);
        Rx(2,1)=-Rx(1,2);
        Rx(2,2)=Rx(1,1);
        
        //palmOrientations["right_stoptap"]=palmOrientations["right_starttap"];
        palmOrientations.insert( pair<string, Matrix>("right_stoptap",palmOrientations["right_starttap"]) );

        Rx(1,1)=cos(-M_PI/8.0);
        Rx(1,2)=-sin(-M_PI/8.0);
        Rx(2,1)=-Rx(1,2);
        Rx(2,2)=Rx(1,1);
        
        //palmOrientations["left_stoptap"]=palmOrientations["left_starttap"];
		palmOrientations.insert( pair<string, Matrix>("left_stoptap",palmOrientations["left_starttap"]) );
    }
////////////////////////////////////////

