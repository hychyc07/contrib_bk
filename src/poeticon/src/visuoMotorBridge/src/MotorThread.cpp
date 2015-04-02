

#include <yarp/os/Time.h>

#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>

#include <yarp/math/Rand.h>


#include <fstream>
#include <sstream>

#include "MotorThread.h"



Vector MotorThread::lp_filter(Vector input)
{
    //This is a butterworth low pass first order, with a cut off freqency of 1Hz
    //It must be used with a sampling frequency of 50Hz (20ms)
    static Vector xv[2], yv[2];
    static bool first=true;
    if(first)
    {
        yv[0]=yv[1]=xv[0]=xv[1]=input;
        first=false;
    }
    else
    {
        xv[0]=xv[1];
        xv[1]=(1.0/1.689454484e+01)*input;
        yv[0]=yv[1];
        yv[1]=(xv[0] + xv[1]) + (  0.8816185924 * yv[0]);
    }

    return yv[1];
}



bool MotorThread::loadExplorationPoses(const string &file_name)
{
    Property optExpl;
    optExpl.fromConfigFile(rf.findFile(file_name.c_str()));

    if(!optExpl.check("torso") ||!optExpl.check("hand") ||!optExpl.check("head"))
        return false;

    fprintf(stdout,"\nTORSO\n");
    Bottle &tmpTorso=optExpl.findGroup("torso");
    for(int i=1; i<tmpTorso.size(); i++)
    {
        Bottle *b=tmpTorso.get(i).asList();
        Vector v(b->size());
        for(int j=0; j<b->size(); j++)
            v[j]=b->get(j).asDouble();
        fprintf(stdout,"%s\n",v.toString().c_str());
        torsoPoses.push_back(v);
    }

    fprintf(stdout,"\nHAND\n");
    Bottle &tmpHand=optExpl.findGroup("hand");
    for(int i=1; i<tmpHand.size(); i++)
    {
        Bottle *b=tmpHand.get(i).asList();
        Vector v(b->size());
        for(int j=0; j<b->size(); j++)
            v[j]=b->get(j).asDouble();
        fprintf(stdout,"%s\n",v.toString().c_str());
        handPoses.push_back(v);
    }
    
    fprintf(stdout,"\nHEAD\n");
    Bottle &tmpHead=optExpl.findGroup("head");
    for(int i=1; i<tmpHead.size(); i++)
    {
        Bottle *b=tmpHead.get(i).asList();
        Vector v(b->size());
        for(int j=0; j<b->size(); j++)
            v[j]=b->get(j).asDouble();
        fprintf(stdout,"%s\n",v.toString().c_str());
        headPoses.push_back(v);
    }

    return true;
}

Vector MotorThread::posToEyesOrient(const Vector &originalPose)
{
    double theta=Rand::scalar(0.0, 0.25*M_PI);

    Matrix M=axis2dcm(originalPose);
    Vector x(4);
    x[0] = 1.0;
    x[1] = 0.0;
    x[2] = 0.0;
    x[3]=theta;

    return dcm2axis(M * axis2dcm(x));
}


Vector MotorThread::visionToMotorNet(const Vector &stereo)
{
    Vector h(6);
    head->getEncoders(h.data());

    Vector in(7);
    in[0]=stereo[0];
    in[1]=stereo[1];
    in[2]=stereo[2];
    in[3]=stereo[3];
    in[4]=h[3];
    in[5]=h[4];
    in[6]=h[5];

    Vector out=net.predict(in);
    Vector out_hom(4);
    out_hom[0]=out[0];
    out_hom[1]=out[1];
    out_hom[2]=out[2];
    out_hom[3]=1.0;

    Vector eyePos,eyeOrient;
    gazeCtrl->getHeadPose(eyePos,eyeOrient);

    Matrix T=axis2dcm(eyeOrient);
    T(0,3)=eyePos[0];
    T(1,3)=eyePos[1];
    T(2,3)=eyePos[2];

    Vector root=T*out_hom;

    // safe thresholding
    if (root[0]>-0.15)
        root[0]=-0.15;

    if (root[2]<=table_height)
        root[2]=table_height;

    Vector xd(3);
    xd[0]=root[0];
    xd[1]=root[1];
    xd[2]=root[2];
  

    return xd;
}


Vector MotorThread::visionToMotorHomography(const Vector &stereo)
{
    Vector xd(3);

    Bottle bEye;
    bEye.addString("left");
    bEye.addDouble(stereo[0]);
    bEye.addDouble(stereo[1]);
    
    eyeOutPort.write(bEye);

    Bottle *bX=eyeInPort.read(true);

    if(bX!=NULL)
    {
        xd[0]=bX->get(0).asDouble();
        xd[1]=bX->get(1).asDouble();    
        xd[2]=bX->get(2).asDouble();
    }

    return xd;
}



Vector MotorThread::randomDeployOffset()
{
    Vector offset(3);
    offset[0]=Rand::scalar(-0.1, 0.0);
    offset[1]=Rand::scalar(-0.05,0.05);
    offset[2]=0.0;

    return offset;
}

bool MotorThread::getGeneralOptions(Bottle &b)
{
    if (Bottle *pB=b.find("home_fixation_point").asList())
    {
        homeFix.resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            homeFix[i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("reach_above_displacement").asList())
    {
        reachAboveDisp.resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            reachAboveDisp[i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("grasp_above_relief").asList())
    {
        graspAboveRelief.resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            graspAboveRelief[i]=pB->get(i).asDouble();
    }
    else
        return false;

    targetInRangeThresh=b.find("target_in_range_thresh").asDouble();

    table_height_tolerance=b.find("table_height_tolerance").asDouble();

    return true;
}



bool MotorThread::getArmOptions(Bottle &b, const int &arm)
{
    if (Bottle *pB=b.find("home_position").asList())
    {
        homePos[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            homePos[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("home_orientation").asList())
    {
        homeOrient[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            homeOrient[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("reach_side_displacement").asList())
    {
        reachSideDisp[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            reachSideDisp[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("reach_above_orientation").asList())
    {
        reachAboveOrient[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            reachAboveOrient[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("reach_above_calib_table").asList())
    {
        reachAboveCata[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            reachAboveCata[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("reach_side_orientation").asList())
    {
        reachSideOrient[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            reachSideOrient[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("deploy_position").asList())
    {
        deployPos[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            deployPos[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("push_position").asList())
    {
        pushDisp[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            pushDisp[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("draw_near_position").asList())
    {
        drawNearPos[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            drawNearPos[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("draw_near_orientation").asList())
    {
        drawNearOrient[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            drawNearOrient[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("grasp_detect_mean").asList())
    {
        graspDetectMean[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            graspDetectMean[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("grasp_open_mean").asList())
    {
        graspOpenMean[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            graspOpenMean[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;


    if (Bottle *pB=b.find("homography_offset").asList())
    {
        homographyOffset[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            homographyOffset[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;


    if (b.check("grasp_detect_thresh"))
        graspDetectThresh[arm]=b.find("grasp_detect_thresh").asDouble();

    if (b.check("external_forces_thresh"))
        extForceThresh[arm]=b.find("external_forces_thresh").asDouble();

    return true;
}


void MotorThread::close()
{

    compensatedTactilePortLeft.interrupt();
    compensatedTactilePortLeft.close();
    compensatedTactilePortRight.interrupt();
    compensatedTactilePortRight.close();

    if(drvHead!=NULL)
        delete drvHead;

    if(drvTorso!=NULL)
        delete drvTorso;

    if(drvArm[LEFT]!=NULL)
    {
        //be sure to set everything back to velocity mdoe
        for(int i=0; i<=4; i++)
            armCtrlMode[LEFT]->setVelocityMode(i);
        delete drvArm[LEFT];
    }

    if(drvArm[RIGHT]!=NULL)
    {
        //be sure to set everything back to velocity mdoe
        for(int i=0; i<=4; i++)
            armCtrlMode[RIGHT]->setVelocityMode(i);
        delete drvArm[RIGHT];
    }

    if(drvGazeCtrl!=NULL)
    {
        gazeCtrl->restoreContext(initial_gaze_context);
        delete drvGazeCtrl;
    }

    if (action[LEFT]!=NULL)
        delete action[LEFT];

    if (action[RIGHT]!=NULL)
        delete action[RIGHT];


    eyeInPort.interrupt();
    eyeOutPort.interrupt();
    eyeInPort.close();
    eyeOutPort.close();

    graspDetectPort[LEFT].interrupt();
    graspDetectPort[RIGHT].interrupt();
    graspDetectPort[LEFT].close();
    graspDetectPort[RIGHT].close();
}


bool MotorThread::threadInit()
{
    eyeInPort.open("/poeticonVMB/eye2world/in");
    eyeOutPort.open("/poeticonVMB/eye2world/out");

    graspDetectPort[LEFT].open("/poeticonVMB/left/detectGrasp:i");
    graspDetectPort[RIGHT].open("/poeticonVMB/right/detectGrasp:i");

    wrenchPort.open("/poeticonVMB/wrench:o");

    Bottle bMotor=rf.findGroup("motor");

    if(bMotor.isNull())
    {
        fprintf(stdout,"Motor part is missing!\n");
        return false;
    }

    string name=rf.find("name").asString().c_str();
    string robot=bMotor.check("robot",Value("icub")).asString().c_str();
    string partUsed=bMotor.check("part_used",Value("both_arms")).asString().c_str();
    setRate(bMotor.check("thread_period",Value(100)).asInt());

    double eyesTrajTime=bMotor.check("eyes_traj_time",Value(1.0)).asDouble();
    double neckTrajTime=bMotor.check("neck_traj_time",Value(2.0)).asDouble();

    double kp=bMotor.check("stereo_kp",Value(0.001)).asDouble();
    double ki=bMotor.check("stereo_ki",Value(0.001)).asDouble();
    double kd=bMotor.check("stereo_kd",Value(0.0)).asDouble();

    stereo_track=bMotor.check("stereo_track",Value("on")).asString()=="on";
    dominant_eye=(bMotor.check("dominant_eye",Value("left")).asString()=="left")?LEFT:RIGHT;


    //fprintf(stdout,"dominant eye= %d\n",dominant_eye);

    Bottle *neckPitchRange=bMotor.find("neck_pitch_range").asList();
    Bottle *neckRollRange=bMotor.find("neck_roll_range").asList();

    // open controllers
    Property optHead("(device remote_controlboard)");
    Property optLeftArm("(device remote_controlboard)");
    Property optRightArm("(device remote_controlboard)");
    Property optTorso("(device remote_controlboard)");
    Property optGazeCtrl("(device gazecontrollerclient)");

    optHead.put("remote",("/"+robot+"/head").c_str());
    optHead.put("local",("/"+name+"/head").c_str());

    optLeftArm.put("remote",("/"+robot+"/left_arm").c_str());
    optLeftArm.put("local",("/"+name+"/left_arm").c_str());

    optRightArm.put("remote",("/"+robot+"/right_arm").c_str());
    optRightArm.put("local",("/"+name+"/right_arm").c_str());

    optTorso.put("remote",("/"+robot+"/torso").c_str());
    optTorso.put("local",("/"+name+"/torso").c_str());

    optGazeCtrl.put("remote","/iKinGazeCtrl");
    optGazeCtrl.put("local",("/"+name+"/gaze").c_str());

    //open the compensated tactile data port
    std::string compensatedTactilePortNameLeft = "/" + name + "/";
    compensatedTactilePortNameLeft += rf.check("compensatedTactilePortLeft", 
            Value("compTactDataLeft:i")).asString().c_str();

    if (! compensatedTactilePortLeft.open(compensatedTactilePortNameLeft.c_str())) 
    {
        fprintf(stdout, "unable to open the compensated left tactile input port \n");
        return false; 
    }

    std::string compensatedTactilePortNameRight = "/" + name + "/";
    compensatedTactilePortNameRight += rf.check("compensatedTactilePortRight", 
            Value("compTactDataRight:i")).asString().c_str();

    if (! compensatedTactilePortRight.open(compensatedTactilePortNameRight.c_str())) 
    {
        fprintf(stdout, "unable to open the compensated right tactile input port \n");
        return false; 
    }

    for (int x=0; x<2; x++){
        percentile[x].resize(SKIN_DIM, 0);
    }
    touchPerFinger.resize(NUM_FINGERS, 0);
    touch_threshold=rf.check("touch_threshold", Value(2.0)).asDouble();// hardcoded for now

    drvHead=new PolyDriver;
    drvTorso=new PolyDriver;
    drvArm[LEFT]=new PolyDriver;
    drvArm[RIGHT]=new PolyDriver;
    drvGazeCtrl=new PolyDriver;
    if (!drvHead->open(optHead)             ||
        !drvTorso->open(optTorso)           ||
        !drvGazeCtrl->open(optGazeCtrl)       )
    {
        close();
        return false;
    }


    // open views
    drvHead->view(head);
    drvTorso->view(torsoPos);
    if(partUsed=="both_arms" || partUsed=="left_arm")
    {
        if(!drvArm[LEFT]->open(optLeftArm))
        {   
            close();
            return false;
        }        

        drvArm[LEFT]->view(armEnc[LEFT]);
        drvArm[LEFT]->view(armCtrlMode[LEFT]);
        drvArm[LEFT]->view(armTorqueCtrl[LEFT]);
        drvArm[LEFT]->view(armImpedenceCtrl[LEFT]);
        drvArm[LEFT]->view(armVelocityCtrl[LEFT]);

        armVelocityCtrl[LEFT]->setRefAcceleration(6,6000.0);
    }

    if(partUsed=="both_arms" || partUsed=="right_arm")
    {
        if(!drvArm[RIGHT]->open(optRightArm))
        {   
            close();
            return false;
        }        

        drvArm[RIGHT]->view(armEnc[RIGHT]);
        drvArm[RIGHT]->view(armCtrlMode[RIGHT]);
        drvArm[RIGHT]->view(armTorqueCtrl[RIGHT]);
        drvArm[RIGHT]->view(armImpedenceCtrl[RIGHT]);
        drvArm[RIGHT]->view(armVelocityCtrl[RIGHT]);

        armVelocityCtrl[RIGHT]->setRefAcceleration(6,6000.0);
    }

    flipHand=7.0;

    drvGazeCtrl->view(gazeCtrl);

    
    Vector vels(3),accs(3);
    vels=5.0; accs=6000.0;
    torsoPos->setRefSpeeds(vels.data());
    torsoPos->setRefAccelerations(accs.data());


    // initialize the gaze controller

    //store the current context
    gazeCtrl->storeContext(&initial_gaze_context);

    gazeCtrl->setTrackingMode(false);
    gazeCtrl->setEyesTrajTime(eyesTrajTime);
    gazeCtrl->setNeckTrajTime(neckTrajTime);


    // set the values for the stereo PID controller
    Bottle stereoOpt;
    Bottle &bKp=stereoOpt.addList();
    bKp.addString("Kp");
    Bottle &bKpVal=bKp.addList();
    bKpVal.addDouble(kp);

    Bottle &bKi=stereoOpt.addList();
    bKi.addString("Ki");
    Bottle &bKiVal=bKi.addList();
    bKiVal.addDouble(ki);

    Bottle &bKd=stereoOpt.addList();
    bKd.addString("Kd");
    Bottle &bKdVal=bKd.addList();
    bKdVal.addDouble(kd);

    Bottle &bTs=stereoOpt.addList();
    bTs.addString("Ts");
    bTs.addDouble(0.05); 

    Bottle &bDominantEye=stereoOpt.addList();
    bDominantEye.addString("dominantEye");
    dominant_eye==LEFT?bDominantEye.addString("left"):bDominantEye.addString("right");

    gazeCtrl->setStereoOptions(stereoOpt);

    //bind neck pitch and roll;
    if(neckPitchRange->size()==1)
    {
        double neckPitchBlock=neckPitchRange->get(0).asDouble();
        gazeCtrl->blockNeckPitch(neckPitchBlock);
    }
    else if(neckPitchRange->size()>1)
    {
        double neckPitchMin=neckPitchRange->get(0).asDouble();
        double neckPitchMax=neckPitchRange->get(1).asDouble();
        gazeCtrl->bindNeckPitch(neckPitchMin,neckPitchMax);
    }
    if(neckRollRange->size()==1)
    {
        double neckRollBlock=neckRollRange->get(0).asDouble();
        gazeCtrl->blockNeckRoll(neckRollBlock);
    }
    else if(neckRollRange->size()>1)
    {
        double neckRollMin=neckRollRange->get(0).asDouble();
        double neckRollMax=neckRollRange->get(1).asDouble();
        gazeCtrl->bindNeckRoll(neckRollMin,neckRollMax);
    }

    //store the current context and restore the initial one
    gazeCtrl->storeContext(&default_gaze_context);
    gazeCtrl->restoreContext(initial_gaze_context);
    gazeInControl=false;

    //-------------------------------

    // extract the exploration poses from a .ini
    string exploration_name=bMotor.find("exploration_poses").asString().c_str();
    if(!loadExplorationPoses(exploration_name))
    {
        fprintf(stdout,"Error while loading exploration poses!\n");
        close();
        return false;
    }

    // init the NN
    Property netOptions;
    string net_name=bMotor.find("net").asString().c_str();
    netOptions.fromConfigFile(rf.findFile(net_name.c_str()).c_str());
    if(!net.configure(netOptions))
    {
        fprintf(stdout,"Error while loading neural network!\n");
        close();
        return false;
    }

    fprintf(stdout,"\n Network:\n");
    net.printStructure();

    // get the general options
    if(!getGeneralOptions(bMotor))
    {
        fprintf(stdout,"Error extracting general options!\n");
        close();
        return false;
    }

    Bottle bArm[2];
    bArm[LEFT]=rf.findGroup("left_arm");
    bArm[RIGHT]=rf.findGroup("right_arm");

    // parsing general arm config options
    Property option;
    for (int i=1; i<bMotor.size(); i++)
    {
        Bottle *pB=bMotor.get(i).asList();
        if (pB->size()==2)
        {
            if(pB->get(0).asString()=="hand_sequences_file")
            {
                string hand_seq_name=bMotor.find("hand_sequences_file").asString().c_str();
                option.put("hand_sequences_file",rf.findFile(hand_seq_name.c_str()).c_str());
            }
            else
                option.put(pB->get(0).asString().c_str(),pB->get(1));
        }
        else
        {
            fprintf(stdout,"Error: invalid option!\n");
            close();
            return false;
        }
    }

    option.put("local",name.c_str());
    //option.put("hand_sequences_file",rf.findFile("hand_sequences_file").c_str());





    //impedence values
    vector<double> stiffness(0),damping(0);
	Bottle *bImpedanceStiff=bMotor.find("impedence_stiffness").asList();
	Bottle *bImpedanceDamp=bMotor.find("impedence_damping").asList();
    
    for(int i=0; i<bImpedanceStiff->size(); i++)
    {
        stiffness.push_back(bImpedanceStiff->get(i).asDouble());
        damping.push_back(bImpedanceDamp->get(i).asDouble());
    }

    double reachingTimeout=2.0*option.check("default_exec_time",Value("3.0")).asDouble();

    string arm_name[]={"left_arm","right_arm"};

    for(int arm=0; arm<2; arm++)
    {
        if (partUsed=="both_arms" || (partUsed=="left_arm" && arm==LEFT)
                                  || (partUsed=="right_arm" && arm==RIGHT))
        {
            // parsing left_arm config options
            if (bArm[arm].isNull())
            {
                fprintf(stdout,"Missing %s parameter list!\n",arm_name[arm].c_str());
                close();
                return false;
            }
            else if(!getArmOptions(bArm[arm],arm))
            {
                fprintf(stdout,"Error extracting %s options!\n",arm_name[arm].c_str());
                close();
                return false;
            }

            Property option_tmp(option);
            option_tmp.put("part",arm_name[arm].c_str());

            fprintf(stdout,"***** Instantiating primitives for %s\n",arm_name[arm].c_str());
            action[arm]=new ActionPrimitivesLayer2(option_tmp);
            action[arm]->setExtForceThres(extForceThresh[arm]);
            action[arm]->enableReachingTimeout(reachingTimeout);

            if (!action[arm]->isValid())
            {
                close();
                return false;
            }
            

            deque<string> q=action[arm]->getHandSeqList();
            fprintf(stdout,"***** List of available %s hand sequence keys:\n",arm_name[arm].c_str());
            for (size_t i=0; i<q.size(); i++)
            {
                Bottle sequence;
                action[arm]->getHandSequence(q[i],sequence);

                fprintf(stdout,"***** %s:\n",q[i].c_str());
                fprintf(stdout,"%s\n",sequence.toString().c_str());
            }

            ICartesianControl *tmpCtrl;
            action[arm]->getCartesianIF(tmpCtrl);


            double armTargetTol=bMotor.check("arm_target_tol",Value(0.01)).asDouble();
            tmpCtrl->setInTargetTol(armTargetTol);

            double tmpTargetTol;
            tmpCtrl->getInTargetTol(&tmpTargetTol);

            fprintf(stdout,"new arm target tol%f\n",tmpTargetTol);

            running[arm]=false;
            armInUse=arm;

            if(arm==LEFT)
            {
                //this is debug set limits on joint 4 to avoid fault
                ICartesianControl *iCart;
                action[LEFT]->getCartesianIF(iCart);
                //iCart->setLimits(7,-50.0, 50.0);
            }


            for(int i=0; i<bImpedanceStiff->size(); i++)
            	armImpedenceCtrl[arm]->setImpedance(i,stiffness[i],damping[i]);		
        }
    }

    //set impedance on or off
    impedanceAvailable=bMotor.check("impedance",Value("off")).asString()=="on";
    impedanceAvailable=impedanceAvailable && Network::isConnected("/filtered/inertial:o","/wholeBodyDynamics/inertial:i");           
    setImpedance(true);
    
    // init the table height
    ifstream fin("table.ini");
    string line;
    if(fin.is_open())
    {
        getline(fin,line);
        table_height=atof(line.c_str());
        fin.close();
    }
    else
        table_height=0.1;

    // drag initialization
    dragLearner.extForceThresh=bMotor.check("drag_force_thresh",Value(1e9)).asDouble();
    dragLearner.samplingRate=bMotor.check("drag_sampling_rate",Value(500.0)).asDouble();
    //--------------------

    grasp_state=GRASP_STATE_IDLE;

    Rand::init();

    head_mode=HEAD_MODE_IDLE;
    arm_mode=ARM_MODE_IDLE;

    return true;
}

void MotorThread::run()
{

    switch(head_mode)
    {
        case(HEAD_MODE_GO_HOME):
        {
            if(!gazeInControl)
            {
                gazeCtrl->restoreContext(default_gaze_context);
                gazeInControl=true;
            }

            gazeCtrl->lookAtFixationPoint(homeFix);
            break;
        }

        case(HEAD_MODE_TRACK_HAND):
        {
            if(!gazeInControl)
            {
                gazeInControl=true;
                gazeCtrl->restoreContext(default_gaze_context);
            }


            Vector x,o;
            action[armInUse]->getPose(x,o);
            gazeCtrl->lookAtFixationPoint(x);
            break;
        }

        case(HEAD_MODE_TRACK_TEMP):
        {
            if(!gazeInControl)
            {
                gazeCtrl->restoreContext(default_gaze_context);
                gazeInControl=true;
            }

            Vector stereo=res.getStereo();
            if(stereo.size()==4)
            {
                Vector px[2];
                px[LEFT].resize(2);
                px[RIGHT].resize(2);


                px[LEFT][0]=stereo[0];
                px[LEFT][1]=stereo[1];
                px[RIGHT][0]=stereo[2];
                px[RIGHT][1]=stereo[3];

                if(stereo_track)
                    gazeCtrl->lookAtStereoPixels(px[LEFT],px[RIGHT]);
                else
                    gazeCtrl->lookAtMonoPixel(dominant_eye,px[dominant_eye],0.4);
            }
            break;
        }


        case(HEAD_MODE_TRACK_FIX):
        {
            if(!gazeInControl)
            {
                gazeInControl=true;
                gazeCtrl->restoreContext(default_gaze_context);
            }
            break;
        }
        default:
            break;
    }

    switch(arm_mode)
    {
        case(ARM_MODE_LEARN):
        {
            Vector wrench, force(3);

            // get the wrench at the end-effector
            // and filter out forces under threshold
            action[dragLearner.armType]->getExtWrench(wrench);
            force[0]=wrench[0];
            force[1]=wrench[1];
            force[2]=wrench[2];

            Vector x(3),o(4);
        
            dragLearner.ctrl->getPose(x,o);

            fprintf(stdout,"curr arm pos= %s \n",x.toString().c_str());

            if(Time::now()-dragLearner.currTime>dragLearner.samplingRate)
            {
                //add the new position to the list of actions
                Bottle &tmp_action=dragLearner.actions.addList();
                Vector tmp_x=dragLearner.initialPos - x;
                for(int i=0; i<tmp_x.size(); i++)
                    tmp_action.addDouble(tmp_x[i]);

                for(int i=0; i<o.size(); i++)
                    tmp_action.addDouble(o[i]);

                dragLearner.currTime=Time::now();
            }

            if(norm(force)>dragLearner.extForceThresh)
                x=x+0.1*(1.0/norm(force))*force;

            //dragLearner.ctrl->goToPositionSync(x,1.0);

            //dragCtrl->askForPosition(x,xd,od,qd);

            //fprintf(stdout,"desired torso positions = %f\t%f\t%f\n\n",qd[0],qd[1],qd[2]);

            //torsopos->positionmove(0,qd[0]);
            //torsopos->positionmove(1,qd[1]);
            //torsopos->positionmove(2,qd[2]);


            break;
        }

        case ARM_MODE_FINE_REACHING:
        {
            bool done;
            action[armInUse]->checkActionsDone(done,false);
            if(done)
            {
                Vector stereoHand=res.getStereo();
                if(stereoHand.size()==4)
                {
                    //move toward the target point
                    

                }



            }

            break;
        }

        case ARM_MODE_FLIP_HAND:
        {
            double v,e;
            armEnc[armInUse]->getEncoder(6,&v);
            e=flipHand-v;

            if (fabs(e)<1.0)
            {
                flipHand=-flipHand;
                e=flipHand-v;
            }

            // here we are required to keep on feeding the velocity
            // to prevent the FW to stop moving because of the low-level guard
            armVelocityCtrl[armInUse]->velocityMove(6,60.0*sign(e));
        }

        default:
            break;
    }
}

void MotorThread::threadRelease()
{
    close();
}

// in case it is implemented....
void MotorThread::onStop()
{
    if(action[LEFT]!=NULL)
        action[LEFT]->syncCheckInterrupt(true);

    if(action[RIGHT]!=NULL)
        action[RIGHT]->syncCheckInterrupt(true);
}


void MotorThread::reachSide(const Vector &stereo, int arm)
{
    Vector xd;
    if(stereo.size()==4)
        xd=visionToMotorHomography(stereo);
    else
    {
        Vector stereoObj; 
        while(stereoObj.size()!=4)
        {
            stereoObj=res.getStereo();    
            xd=visionToMotorHomography(stereoObj);
        }
    }

    if(xd.size()!=3)
        return;

    Vector d(3); d=0.0;

    arm=checkArm(arm,xd);

    if(arm==LEFT)
    {
        xd[0]+=0.05;
        xd[2]+=0.01;
    }
    else
    {
        xd[0]+=0.05;
        xd[2]+=0.00;
    }

    gazeCtrl->setTrackingMode(true);
    gazeCtrl->lookAtFixationPoint(xd);

    //action[arm]->latchWrenchOffset();
    //action[arm]->enableContactDetection();
    action[arm]->pushAction(xd+reachSideDisp[arm],reachSideOrient[arm],"pregrasp_hand",2.0);
    action[arm]->pushAction(xd-reachSideDisp[arm],reachSideOrient[arm],2.0);

    bool f;
    action[arm]->checkActionsDone(f,true);

    //action[arm]->disableContactDetection();

    gazeCtrl->setTrackingMode(false);
}

void MotorThread::reachAbove(const Vector &stereo, int arm, bool pregrasp_hand)
{
    Vector xd;
    if(stereo.size()==4)
        xd=visionToMotorHomography(stereo);
    else
    {
        Vector stereoObj;
        while(stereoObj.size()!=4)
        {
            stereoObj=res.getStereo();    
            xd=visionToMotorHomography(stereoObj);
        }
    }

    if(xd.size()!=3)
        return;

    arm=checkArm(arm,xd);


    head_mode=HEAD_MODE_TRACK_FIX;
    gazeCtrl->setTrackingMode(true);
    gazeCtrl->lookAtFixationPoint(xd);

    if(pregrasp_hand)
        action[arm]->pushAction(xd+reachAboveDisp,reachAboveOrient[arm],"pregrasp_hand",2.0);
    else
        action[arm]->pushAction(xd+reachAboveDisp,reachAboveCata[arm],2.0);

    action[arm]->latchWrenchOffset();
    action[arm]->enableContactDetection();

    if(arm==LEFT)
    {
        xd[2]=table_height-table_height_tolerance+0.05;
    }
    else
    {
        xd[2]=table_height-table_height_tolerance+0.06;
    }

    action[arm]->pushAction(xd,reachAboveOrient[arm],3.0);



    bool f;
    action[arm]->checkActionsDone(f,true);
    action[arm]->disableContactDetection();

    grasp_state=GRASP_STATE_ABOVE;

    //Vector x,o;
    //action[arm]->getPose(x,o);
    //action[arm]->pushAction(x+graspAboveRelief,reachAboveOrient[arm],1.0);
    action[arm]->checkActionsDone(f,true);

    gazeCtrl->setTrackingMode(false);
}




void MotorThread::reachAboveNotSafe(const Vector &stereo, int arm, bool pregrasp_hand)
{
    Vector xd;
    if(stereo.size()==4)
        xd=visionToMotorHomography(stereo);
    else
    {
        Vector stereoObj;
        while(stereoObj.size()!=4)
        {
            stereoObj=res.getStereo();    
            xd=visionToMotorHomography(stereoObj);
        }
    }

    if(xd.size()!=3)
        return;

    arm=checkArm(arm,xd);



    head_mode=HEAD_MODE_TRACK_FIX;
    gazeCtrl->setTrackingMode(true);
    gazeCtrl->lookAtFixationPoint(xd);

    if(pregrasp_hand)
        action[arm]->pushAction(xd+reachAboveDisp,reachAboveOrient[arm],"pregrasp_hand",2.0);
    else
        action[arm]->pushAction(xd+reachAboveDisp,reachAboveCata[arm],2.0);


    if(arm==LEFT)
    {
        xd[2]=table_height-table_height_tolerance+0.15;
    }
    else
    {
        xd[2]=table_height-table_height_tolerance+0.16;
    }

    action[arm]->pushAction(xd,reachAboveOrient[arm],3.0);



    bool f;
    action[arm]->checkActionsDone(f,true);

    grasp_state=GRASP_STATE_ABOVE;

    //Vector x,o;
    //action[arm]->getPose(x,o);
    //action[arm]->pushAction(x+graspAboveRelief,reachAboveOrient[arm],1.0);
    action[arm]->checkActionsDone(f,true);

    gazeCtrl->setTrackingMode(false);
}




void MotorThread::reachAboveSide(const Vector &stereo, int arm)
{
    Vector xd;
    if(stereo.size()==4)
        xd=visionToMotorHomography(stereo);
    else
    {
        Vector stereoObj;
        while(stereoObj.size()!=4)
        {
            stereoObj=res.getStereo();    
            xd=visionToMotorHomography(stereoObj);
        }
    }

    if(xd.size()!=3)
        return;

    arm=checkArm(arm,xd);



    head_mode=HEAD_MODE_TRACK_FIX;
    gazeCtrl->setTrackingMode(true);
    gazeCtrl->lookAtFixationPoint(xd);



    if(arm==LEFT)
    {
        xd[2]=table_height-table_height_tolerance+0.35;
    }
    else
    {
        xd[2]=table_height-table_height_tolerance+0.40;
    }


    action[arm]->pushAction(xd,reachSideOrient[arm],3.0);


    if(arm==LEFT)
    {
        xd[2]=table_height-table_height_tolerance+0.20;
    }
    else
    {
        xd[2]=table_height-table_height_tolerance+0.20;
    }

    action[arm]->pushAction(xd,reachSideOrient[arm],3.0);


    bool f;
    
    //Vector x,o;
    //action[arm]->getPose(x,o);
    //action[arm]->pushAction(x+graspAboveRelief,reachAboveOrient[arm],1.0);
    action[arm]->checkActionsDone(f,true);

    grasp_state=GRASP_STATE_SIDE;

    gazeCtrl->setTrackingMode(false);
}


void MotorThread::reachSpace(const Vector &stereo, int arm)
{
    Vector xd;
    if(stereo.size()==4)
        xd=visionToMotorNet(stereo);
    else
    {
        Vector stereoObj; 
        while(stereoObj.size()!=4)
        {
            stereoObj=res.getStereo();    
            xd=visionToMotorNet(stereoObj);
        }
    }

    Vector d(3); d=0.0;

    arm=checkArm(arm,xd);

    head_mode=HEAD_MODE_TRACK_FIX;
    gazeCtrl->setTrackingMode(true);
    gazeCtrl->lookAtFixationPoint(xd);

    action[arm]->latchWrenchOffset();
    action[arm]->enableContactDetection();
    action[arm]->pushAction(xd+reachSideDisp[arm],reachSideOrient[arm],"pregrasp_hand",2.0);
    action[arm]->pushAction(xd-reachSideDisp[arm],reachSideOrient[arm],2.0);

    bool f;
    action[arm]->checkActionsDone(f,true);

    action[arm]->disableContactDetection();

    grasp_state=GRASP_STATE_SIDE;

    gazeCtrl->setTrackingMode(false);
}


void MotorThread::push(const Vector &stereo, int arm)
{
    Vector xd;
    if(stereo.size()==4)
        xd=visionToMotorHomography(stereo);
    else
    {
        Vector stereoObj;
        while(stereoObj.size()!=4)
        {
            stereoObj=res.getStereo();    
            xd=visionToMotorHomography(stereoObj);
        }
    }

    if(xd.size()!=3)
        return;

    arm=checkArm(arm,xd);

    if(arm==LEFT)
    {
        xd[2]+=0.025;
    }
    else
    {
        xd[2]+=0.03;
    }

    Vector startPos=xd-pushDisp[arm];
    Vector endPos=xd+pushDisp[arm];


    action[arm]->pushAction(xd+reachSideDisp[arm],reachSideOrient[arm],2.0);

    bool f;
    action[arm]->checkActionsDone(f,true);

    //action[arm]->latchWrenchOffset();
    //action[arm]->enableContactDetection();

    lookAtHand(true);

    action[arm]->pushAction(xd-3*reachSideDisp[arm],reachSideOrient[arm],2.0);
    //action[arm]->tap(startPos,reachSideOrient[arm],endPos,reachSideOrient[arm],2.0);

    action[arm]->checkActionsDone(f,true);

    //action[arm]->disableContactDetection();
    lookAtHand(false);
}


void MotorThread::point(const Vector &stereo, int arm)
{
    Vector xd;
    if(stereo.size()==4)
        xd=visionToMotorHomography(stereo);
    else
    {
        Vector stereoObj;
        while(stereoObj.size()!=4)
        {
            stereoObj=res.getStereo();    
            xd=visionToMotorHomography(stereoObj);
        }
    }

    if(xd.size()!=3)
        return;

    //xd[2]=table_height-0.05;
    Vector target=xd;

    arm=checkArm(arm,xd);

    Vector x,o;
    action[arm]->getPose(x,o);


    //set the new position
    Vector d=0.5*(target-x);
    xd=x+d;

    //set the new orientation
    Vector x_o(3),y_o(3),z_o(3);
    x_o=(1/norm(d))*d;
    
    z_o=0.0;
    if(arm==LEFT)
        z_o[2]=1.0;
    else
        z_o[2]=-1.0;

    y_o=cross(z_o,x_o);
    y_o=(1/norm(y_o))*y_o;

    z_o=cross(x_o,y_o);
    z_o=(1/norm(z_o))*z_o;

    /*
    double alpha=atan2(0.025,norm(target-xd));

    Matrix P(3,3);
    P(0,0)=cos(alpha);
    P(0,1)=-sin(alpha);
    P(0,2)=0.0;
    P(1,0)=sin(alpha);
    P(1,1)=cos(alpha);
    P(1,2)=0.0;
    P(2,0)=0.0;
    P(2,1)=0.0;
    P(2,2)=1.0;

    fprintf(stdout,"x = %s\n",x_o.toString().c_str());
    fprintf(stdout,"y = %s\n",y_o.toString().c_str());
    fprintf(stdout,"z = %s\n\n",z_o.toString().c_str());
    */

    Matrix R(3,3);
    R.setCol(0,x_o);
    R.setCol(1,y_o);
    R.setCol(2,z_o);

    //Vector od=dcm2axis(P*R);
    Vector od=dcm2axis(R);

    action[arm]->pushAction(xd,od,"pointing_hand",2.0);

    bool f;
    action[arm]->checkActionsDone(f,true);
}

void MotorThread::grasp(int arm)
{
    arm=checkArm(arm);
    Vector x,o,d(3);
    d=0.0;
    action[arm]->getPose(x,o);
    fprintf(stdout, "strange....%s %s \n", x.toString().c_str(), o.toString().c_str());
    //action[arm]->grasp(x,o,d);
    action[arm]->pushAction(x,o,"close_hand",0.8);

    bool f;
    action[arm]->checkActionsDone(f,true);
}

void MotorThread::release(int arm)
{
    arm=checkArm(arm);

    action[arm]->pushAction("open_hand");

    bool f;
    action[arm]->checkActionsDone(f,true);
}


void MotorThread::goHome(bool head_home)
{
    bool fix=false;
    gazeCtrl->getTrackingMode(&fix);

    if(!fix && head_mode!=HEAD_MODE_TRACK_TEMP && head_home)
        head_mode=HEAD_MODE_GO_HOME;


    if(action[LEFT]!=NULL)
    {
        bool f;
        action[LEFT]->setTrackingMode(true);
        action[LEFT]->pushAction(homePos[LEFT],homeOrient[LEFT],ARM_HOMING_PERIOD);
        action[LEFT]->checkActionsDone(f,true);
    }


    if(action[RIGHT]!=NULL)
    {
        bool f;
        action[RIGHT]->setTrackingMode(true);
        action[RIGHT]->pushAction(homePos[RIGHT],homeOrient[RIGHT],ARM_HOMING_PERIOD);
        action[RIGHT]->checkActionsDone(f,true);
    }


    if(action[RIGHT]!=NULL)
    {
        //action[RIGHT]->enableArmWaving(homePos[RIGHT]);
        action[RIGHT]->setTrackingMode(false);
    }

    if(action[LEFT]!=NULL)
    {
        //action[LEFT]->enableArmWaving(homePos[LEFT]);
        action[LEFT]->setTrackingMode(false);
    }

    if(!fix && head_mode!=HEAD_MODE_TRACK_TEMP)
        setGazeIdle();
}


void MotorThread::deploy(bool random, int arm, const Vector &stereo)
{
    arm=checkArm(arm);

    Vector x,o;
    action[arm]->getPose(x,o);

    Vector deployZone;

    if(stereo.size()==4)
    {
        deployZone=visionToMotorHomography(stereo);

        if(arm==LEFT)
        {
            deployZone[0]+=0.01;
            deployZone[1]-=0.04;
        }
        else
        {
            deployZone[0]+=0.01;
            deployZone[1]+=0.04;
        }
    }    
    else
    {
        deployZone=deployPos[arm];

        if(random)
            deployZone=deployZone+randomDeployOffset();
    }   





    Vector deployFixZone=deployZone;
    deployFixZone[2]=table_height;

    keepFixation(true);
    gazeCtrl->lookAtFixationPoint(deployFixZone);

    Vector *tmpOrient=grasp_state==GRASP_STATE_SIDE?&reachSideOrient[arm]:&reachAboveOrient[arm];

    deployZone[2]=x[2];  

    // prepare hand for deployment
    action[arm]->pushAction(deployZone,*tmpOrient,2.0);
    bool f;
    action[arm]->checkActionsDone(f,true);

    action[arm]->latchWrenchOffset();
    action[arm]->enableContactDetection();


    if(stereo.size()==4)
        deployZone[2]+=table_height-table_height_tolerance+0.10;
    else
        deployZone[2]=deployPos[arm][2];
    action[arm]->pushAction(deployZone,*tmpOrient,3.0);
    action[arm]->checkActionsDone(f,true);

    action[arm]->getPose(x,o);
    gazeCtrl->lookAtFixationPoint(x);
    action[arm]->disableContactDetection();

    release(arm);

    keepFixation(false);
}



void MotorThread::putAway(bool random)
{
    int arm=checkArm(ARM_IN_USE);

    Vector x,o;
    action[arm]->getPose(x,o);

    Vector deployZone=deployPos[arm];
    
    deployZone[0]=-0.15;

    if(arm==LEFT)
        deployZone[1]=-0.35;
    else
        deployZone[1]=0.35;

    if(random)
        deployZone=deployZone+randomDeployOffset();


    Vector deployFixZone=deployZone;
    deployFixZone[2]=table_height;

    keepFixation(true);
    gazeCtrl->lookAtFixationPoint(deployFixZone);

    Vector *tmpOrient=grasp_state==GRASP_STATE_SIDE?&reachSideOrient[arm]:&reachAboveOrient[arm];

    deployZone[2]=x[2];  

    // prepare hand for deployment
    action[arm]->pushAction(deployZone,*tmpOrient,2.0);
    bool f;
    action[arm]->checkActionsDone(f,true);

    action[arm]->latchWrenchOffset();
    action[arm]->enableContactDetection();

    deployZone[2]=deployPos[arm][2];

    action[arm]->pushAction(deployZone,*tmpOrient,3.0);
    action[arm]->checkActionsDone(f,true);

    action[arm]->getPose(x,o);
    gazeCtrl->lookAtFixationPoint(x);
    action[arm]->disableContactDetection();

    release(arm);

    keepFixation(false);
}




void MotorThread::drawNear(int arm)
{
    arm=checkArm(arm);

    action[arm]->pushAction(drawNearPos[arm],drawNearOrient[arm],2.0);

    bool f;
    action[arm]->checkActionsDone(f,true);
}

bool MotorThread::isHolding(int arm)
{
    arm=checkArm(arm);

    bool holding=false;
    //action[arm]->areFingersInPosition(holding);

    Vector *vHold=graspDetectPort[arm].read(true);
    if(vHold!=NULL)
    {
        holding=(norm(graspDetectMean[arm]-*vHold)>graspDetectThresh[arm] && norm(graspOpenMean[arm]-*vHold)>80.0);
    }

    holding= holding;// || isTactileHolding(arm);

    return holding;
}

bool MotorThread::setPercentile(vector<float> newPercentile, int arm){
	// check the vector size
	if(newPercentile.size() < SKIN_DIM)
		return false;

	//check the percentile values (not negative AND not greater than or equal to 255)
	for(int i=0; i<SKIN_DIM; i++){
		if(newPercentile[i]<0 || newPercentile[i]>=MAX_SKIN)
			return false;
	}

	// update the percentile
	percentileSem.wait();
	for(int i=0; i<SKIN_DIM; i++){
		percentile[arm][i] = newPercentile[i];
	}
	percentileSem.post();

	return true;
}

bool MotorThread::isTactileHolding(int arm)
{
    vector<bool> detect_touch_finger(NUM_FINGERS-1, false);
    arm=checkArm(arm);
    bool tactHolding = false;

    if (arm==LEFT)
        compensatedData = *(compensatedTactilePortLeft.read());
    else
        compensatedData = *(compensatedTactilePortRight.read());
    
    //FIND THE MAX TOUCH FOR EACH FINGER
    touchPerFinger.assign(touchPerFinger.size(), 0);
    fprintf(stdout,"checking percentile %d compensated data %d  touch %d percentile %d\n", arm, compensatedData.size(), touchPerFinger.size(), percentile[arm].size());
    for (int i=0; i<60; i++) //here 48 as we use the 4 fingers...no thumbs for now...12 taxels each fingers..
    {
        
        if (compensatedData[i]-percentile[arm][i] > touchPerFinger[i/12]){
            touchPerFinger[i/12] = (float)(compensatedData[i]-percentile[arm][i]);
            fprintf(stdout,"%lf ", percentile[arm][i]);
        }
    }

    fprintf(stdout, "sensor vals: %lf %lf %lf \n",touchPerFinger[0], touchPerFinger[1], touchPerFinger[2]);    
    
    if(touchPerFinger[0]>touch_threshold)
        detect_touch_finger[0] = true;
    if(touchPerFinger[1]>touch_threshold)
        detect_touch_finger[1] = true;
    if(touchPerFinger[2]>touch_threshold || touchPerFinger[3]>touch_threshold)
        detect_touch_finger[2] = true;

    //for a grasp we require at least two fingers to have contact
    
    /*if (static_cast<int>(detect_touch_finger[0]) + static_cast<int>(detect_touch_finger[1]) + static_cast<int>(detect_touch_finger[2]) + static_cast<int>(detect_touch_finger[3]) > 1)
    {
        int sum=0;
        for(int i=0; i<5; i++)
        {
            fprintf(stdout,"%d = %d\n",i,static_cast<int>(detect_touch_finger[i]));
            if(static_cast<int>(detect_touch_finger[i])>1)
                sum++;
        }

        if(sum>2)
            tactHolding = true;
    }*/

    if (static_cast<int>(detect_touch_finger[0]) + static_cast<int>(detect_touch_finger[1]) + static_cast<int>(detect_touch_finger[2]) + static_cast<int>(detect_touch_finger[3]) > 1)
    {
            tactHolding = true;
    }
    
    return tactHolding;
}

bool MotorThread::calibTable(int arm)
{
    arm=checkArm(arm);

    Vector deployZone=deployPos[arm];
    deployZone=deployZone+randomDeployOffset();

    Vector deployPrepare,deployEnd;
    deployPrepare=deployEnd=deployZone;

    deployPrepare[2]=0.1;
    deployEnd[2]=-0.2;

    bool f=false;
    
    gazeCtrl->blockNeckRoll(0.0);
    gazeCtrl->setTrackingMode(true);
    gazeCtrl->lookAtFixationPoint(deployEnd);

    if(isHolding(arm))
        action[arm]->pushAction("open_hand");

    action[arm]->pushAction(deployPrepare,reachAboveCata[arm],3.0);
    action[arm]->checkActionsDone(f,true);

    action[arm]->latchWrenchOffset();
    action[arm]->enableContactDetection();
    action[arm]->pushWaitState(1.0);

    action[arm]->pushAction(deployEnd,reachAboveCata[arm],5.0);
    action[arm]->checkActionsDone(f,true);
    action[arm]->pushWaitState(2.0);
    action[arm]->disableContactDetection();

    action[arm]->checkContact(f);

    bool found=false;
    if (f)
    {
        Vector x,o;
        action[arm]->getPose(x,o);
        table_height=x[2]+table_height_tolerance;

        // write to file
        ofstream fout("table.ini");
        fout << table_height << endl;
        fout.close();

        fprintf(stdout,"########## Table height found: %f\n",table_height);
        found=true;
    }
    else
    {
        fprintf(stdout,"########## Table height not found.\n");
        found=false;    
    }

    action[arm]->pushAction(deployPrepare,reachAboveOrient[arm],ARM_HOMING_PERIOD);

    gazeCtrl->setTrackingMode(false);


    goHome();
    gazeCtrl->clearNeckRoll();

    return found;
}




bool MotorThread::exploreTable(int arm, Vector xd)
{
    arm=checkArm(arm);

    Vector deployZone=xd;

    Vector deployPrepare,deployEnd;
    deployPrepare=deployEnd=deployZone;

    deployPrepare[2]=0.1;
    deployEnd[2]=-0.2;

    bool f=false;
    
    /*
    gazeCtrl->blockNeckRoll(0.0);
    gazeCtrl->setTrackingMode(true);
    gazeCtrl->lookAtFixationPoint(deployEnd);
    */

    action[arm]->pushAction(deployPrepare,reachAboveCata[arm],"open_hand",2.0);
    action[arm]->checkActionsDone(f,true);

    action[arm]->latchWrenchOffset();
    action[arm]->enableContactDetection();
    action[arm]->pushWaitState(1.0);

    action[arm]->pushAction(deployEnd,reachAboveCata[arm],5.0);
    action[arm]->checkActionsDone(f,true);
    action[arm]->pushWaitState(2.0);
    action[arm]->disableContactDetection();

    action[arm]->checkContact(f);

    bool found=false;
    if (f)
    {
        Vector x,o;
        action[arm]->getPose(x,o);
        table_height=x[2]+table_height_tolerance;

        // write to file
        ofstream fout("table.ini");
        fout << table_height << endl;
        fout.close();

        fprintf(stdout,"########## Table height found: %f\n",table_height);
        found=true;
    }
    else
    {
        fprintf(stdout,"########## Table height not found.\n");
        found=false;    
    }

    Vector x,o;
    action[arm]->getPose(x,o);
    x[2]+=table_height_tolerance;
    o[0]=0.128037;
    o[1]=-0.016785;
    o[2]=0.991627;
    o[3]=2.925947;

    //action[arm]->pushAction(x,reachAboveOrient[arm],1.0);
    action[arm]->pushAction(x,o,2.0);

    gazeCtrl->setTrackingMode(false);


    //gazeCtrl->clearNeckRoll();

    return found;
}


void MotorThread::exploreHand(const double &trial_time, int arm)
{
    arm=checkArm(arm);

    double t0=Time::now();

    Vector min(3),max(3);
    min=-0.05;
    max=0.05;
    
    Vector x,orient;
    action[arm]->getPose(x,orient);
    while(isRunning() && (Time::now()-t0<trial_time))
    {
        Vector randPos=drawNearPos[arm]+Rand::vector(min,max);
        Vector randOrient=posToEyesOrient(orient);
        action[arm]->pushAction(randPos,randOrient,2.0);
        bool f;
        action[arm]->checkActionsDone(f,true);
    }
}

void MotorThread::exploreTorso(const double &trial_time)
{
    // avoid torso control
    if(action[LEFT]!=NULL)
        action[LEFT]->setTrackingMode(false);

    if(action[RIGHT]!=NULL)
        action[RIGHT]->setTrackingMode(false);

    double t0=Time::now();

    fprintf(stdout,"torso\n");

    int i=0;
    while(isRunning() && (Time::now()-t0<trial_time))
    {
        torsoPos->positionMove(torsoPoses[i%torsoPoses.size()].data());
        bool done=false;
        while(isRunning() && !done)
        {
            Time::delay(0.1);
            torsoPos->checkMotionDone(&done);
        }
        i++;
    }

    fprintf(stdout,"torso\n");

    //go back to torso home last pose
    torsoPos->positionMove(torsoPoses.back().data());
    bool done=false;
    while(isRunning() && !done)
    {
        Time::delay(0.1);
        torsoPos->checkMotionDone(&done);
    }


    fprintf(stdout,"torso\n");
}

//drag mode
bool MotorThread::startDragMode(const string &action_name, int arm)
{
    arm=checkArm(arm);

    string arm_in_use=arm==LEFT?"left":"right";

    ifstream action_fin(("actions/"+arm_in_use+"/"+action_name+".txt").c_str());
    if(action_fin.is_open())
    {
        fprintf(stdout,"action '%s' already learned\n",action_name.c_str());
        return false;
    }

    dragLearner.actionName=action_name;
    dragLearner.actions.clear();


    bool f;
    action[arm]->checkActionsDone(f,true);
    action[arm]->latchWrenchOffset();
    action[arm]->enableTorsoDof();

    action[arm]->getCartesianIF(dragLearner.ctrl);

    ICartesianControl *dragCtrl;
    action[arm]->getCartesianIF(dragCtrl);

    if(dragCtrl==NULL)
    {
        return false;
    }

    dragLearner.armType=arm;

    Vector x(3),o(4);
    dragCtrl->getPose(x,o);
    dragLearner.initialPos=x;

    Vector newPos;
    dragCtrl->getDOF(newPos);

    newPos=0.0;
    //newPos[0]=1.0;
    newPos[2]=1.0;

    dragLearner.ctrl->setDOF(newPos,newPos);

    dragLearner.ctrl->getLimits(2,&dragLearner.min,&dragLearner.max);    
    //dragLearner.ctrl->setLimits(2,-40.0,40.0);
    


    for(int i=0; i<=4; i++)
        armCtrlMode[arm]->setTorqueMode(i);

    dragLearner.currTime=Time::now();

    arm_mode=ARM_MODE_LEARN;

    return true;
}

void MotorThread::suspendDragMode()
{
    int arm=dragLearner.armType;
    string arm_in_use=arm==LEFT?"left":"right";

    //save the actions in a file
    ofstream action_fout(("actions/"+arm_in_use+"/"+dragLearner.actionName+".txt").c_str());

    if(!action_fout.is_open())
    {
        fprintf(stdout,"error opening the file for action %s\n",dragLearner.actionName.c_str());
        return;
    }

    action_fout << dragLearner.actions.toString();
    //-------------

    // set back again to velocity mode

    arm_mode=ARM_MODE_IDLE;
    Time::delay(0.1);

    for(int i=0; i<=4; i++)
        armCtrlMode[arm]->setImpedanceVelocityMode(i);
    Vector newPos;
    dragLearner.ctrl->getDOF(newPos);
    newPos=1.0;
    newPos[1]=0.0;
    dragLearner.ctrl->setDOF(newPos,newPos);
    //dragLearner.ctrl->setLimits(2,dragLearner.min,dragLearner.max);  

}


bool MotorThread::imitateAction(const string &action_name, int arm)
{
    arm=checkArm(arm);

    string arm_in_use=arm==LEFT?"left":"right";

    ifstream action_fin(("actions/"+arm_in_use+"/"+action_name+".txt").c_str());
    if(!action_fin.is_open())
    {
        fprintf(stdout,"action '%s' unkown\n",action_name.c_str());
        return false;
    }

    stringstream strstr;
    strstr << action_fin.rdbuf();

    Bottle actions(strstr.str().c_str());




    ICartesianControl *ctrl;
    action[arm]->getCartesianIF(ctrl);


    double currTrajTime;
    ctrl->getTrajTime(&currTrajTime);
    ctrl->setTrajTime(0.75);
    
    Vector newPos;
    ctrl->getDOF(newPos);
    newPos=1.0;
    newPos[0]=0.0;
    newPos[1]=0.0;
    //newPos[0]=0.0;
    ctrl->setDOF(newPos,newPos);


    ctrl->setInTargetTol(0.01);

    //ctrl->setTrajTime(1.5*dragLearner.samplingRate);


    Vector init_x(3),init_o(4);
    ctrl->getPose(init_x,init_o);

    //Bottle *actions=actionList.find(action_name.c_str()).asList();


    //fprintf(stdout, "action list %s\n", actions->toString().c_str());

    for(int i=0; i<actions.size(); i++)
    {
        Bottle *b=actions.get(i).asList();
        Vector x(3),o(4);
        for(int j=0; j<3; j++)
            x[j]=b->get(j).asDouble();

        for(int j=0; j<4; j++)
            o[j]=b->get(j+3).asDouble();

        //ctrl->goToPosition(init_x-x);
        ctrl->goToPose(init_x-x,o);

        //Time::delay(dragLearner.samplingRate);
        Time::delay(0.1);
    }

    ctrl->getDOF(newPos);
    newPos=1.0;
    newPos[1]=0.0;
    ctrl->setDOF(newPos,newPos);

    //tmpCtrl->setInTargetTol(armTargetTol);

    ctrl->setTrajTime(currTrajTime);

    return true;
}



