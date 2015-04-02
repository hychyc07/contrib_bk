#include "armExploration.h"

armExploration::armExploration(int _rate, PolyDriver *_dd, 
        const string &_part, const string &_name, Vector &_FT, bool _v) : RateThread(_rate), client(_dd), v(_v) {
            
    FT = &_FT;

    FTlinEst =new AWLinEstimator(16,1.0);

    part = _part.c_str();

    client->view(arm);

    arm->setTrajTime(1.0);

    Vector newDof, curDof;
    arm->getDOF(curDof);
    newDof=curDof;
    newDof[0]=1;
    newDof[1]=0;
    newDof[2]=1;
    arm->setDOF(newDof,curDof);

    limitTorsoPitch();

    xd.resize(3);
    od.resize(4);
    odl.resize(4);
    odr.resize(4);
    x.resize(3);
    o.resize(4);
    
    alt=0.1; // [m]
    
    startTraj=true;
    f_cnt=fz=fz_orig=0;
    
    arm->getPose(x,o);
    o0=o;
    x0=x;
    
    FT_f.resize(3,0.0);
    FT_mu.resize(3,0.0);
    dFT_f.resize(3,0.0);
    dFT_mu.resize(3,0.0);
    
    odl[0]=0.0;
    odl[1]=0.0;
    odl[2]=-1.0;
    odl[3]=M_PI;
    
    Vector oz(4), ox(4);
    oz[0]=0.0; oz[1]=0.0; oz[2]=1.0; oz[3]=M_PI;
    ox[0]=1.0; ox[1]=0.0; ox[2]=0.0; ox[3]=M_PI;

    Matrix Rz=axis2dcm(oz);   // from axis/angle to rotation matrix notation
    Matrix Rx=axis2dcm(ox);

    Matrix R=Rz*Rx;
    
    odr=dcm2axis(R);
    
    done=true;
}

bool armExploration::threadInit() {
    
    t=t0=t1=t2=Time::now();
    
    outfile.open ("threshold/nostop_final.txt", ios_base::app);
    //outfile<<"##########################################################################################\n";
    //outfile<<"#done,(*FT).toString(),dFT.toString(),norm(FT_f),norm(FT_mu),norm(dFT_f),norm(dFT_mu)\n";
    //outfile<<"##########################################################################################\n";
    
    return true;
}
    
void armExploration::run() {
    t=Time::now();
    
    arm->getPose(x,o);

    dFT = evalVel((*FT));
    
    for (int i=0; i<3; i++) {
        FT_f[i] = (*FT)[i];
        FT_mu[i] = (*FT)[i+3];
        dFT_f[i] = dFT[i];
        dFT_mu[i] = dFT[i+3];
    }    
    
    if(startTraj) {
        if(v) cout<<"\nGenerating the target.. ";
        generateTarget();

        //posiziona la mano prima di prendere l'offset:
        if (v) cout<<"\tx0: "<<x0.toString();
        
        if (part=="left_arm") {
            if(v) cout<<"\todl: "<<odl.toString();
            arm->goToPose(x0,odl);
        }
        else {
            if(v) cout<<"\todr: "<<odr.toString();
            arm->goToPose(x0,odr);
        }
        
        bool mov=false;
        arm -> checkMotionDone(&mov);
        
        if (mov) startTraj=false;
    }
    else {
        if(v) cout<<"\nGenerating the target.. ";
        generateTarget();
        
        if (done) {
            if(v) cout<<"\tMoving.. ";
            arm->goToPose(xd,od);   // send request and doesn't wait for reply
        }

        printStatus();
    }
}

void armExploration::threadRelease() {    
    // we require an immediate stop before closing the client for safety
    // reason (anyway it's already done internally in the destructor)
    
    outfile.close();
    
    if (FTlinEst) {
        delete FTlinEst;
        FTlinEst = 0;
    }
    
    arm->stopControl();
    delete client;
}

void armExploration::generateTarget() {      
    //The desired configuration is the same at the resting configuration,
    //except from xd[2] wich changes according to the target
    switch( fz ) {
    case (-1 ):
        done=false;
        //fz=0;
        break;
    case ( 0 ):
        arm->getPose(x,o);
        o0=o;
        x0=x;
        od=o0;
        xd=x0;
        t=Time::now();
        t0=t;
        done=false;
        //if (t-t1>=3) fz=1;
        if ((*FT)[0] != 0) fz=1;
        //cout<<"x0 = "<<x0.toString()<<endl;
        break;
    case ( 1 ):
        //xd[2] = x0[2] - (1/DELTAT) * alt * (t-t0);
        xd[2] = xd[2] - (1/DELTAT) * alt * (getRate()/1000);
        
        if ( xd[2] - x0[2] + alt <= 0.00000 ) {
            fz=2;
            //t0 = t;
        }
        
        done=true;
        
        break;
    case ( 2 ):
        //xd[2] = x0[2] - alt + (1/DELTAT) * alt * (t-t0);
        xd[2] = xd[2] + (1/DELTAT) * alt * (getRate()/1000);
        if ( xd[2] - x0[2] >= 0.00000 )     {
            f_cnt++;
            if (f_cnt==100)  fz=-1;
            else        {
                cout<<"# Trajectory: "<<f_cnt<<endl;            
                fz=1;
            }
        }
        done=true;
        
        break;
    case ( 3 ):
        t0=t;
        fz=4;
        done=false;
        
        break;
    case ( 4 ):
        //cout<<"t - t0 = "<<t-t0<<" ";
        if (t-t0>STOP_PER) {fz=fz_orig; for (int i=0; i<6; i++)     (*FT)[i] = 0;}
        done=false;
        
        break;
    }
    
    //if(v) cout<<"xd = "<<xd.toString();
    if(v) cout<<"xd[2] = "<<xd[2];
    
    // check if something goes wrong
    if ( xd[2] < 0.2 && xd[2] > -0.2 /*&& xd[1] > -0.25 && xd[1] < 0.2*/ && xd[0] < -0.15 ) {
        if(v) cout<<"\t OK. ";
        done = done && true;
    }
    else {
        if(v) cout<<"\t ERROR IN THE TRAJECTORY. ";
        done=false;
    }
    
    int nts = Need_to_Stop();
    
    if (fz != -1 && fz != 3 && fz != 4 && nts) {
        cout<<"\nCONTACT:\n\tnorm(FT_f) = "<<norm(FT_f)<<"\tnorm(FT_mu) = "<<norm(FT_mu);
        cout<<"\tnorm(FT_f)/"<<F_AVG<<": "<<norm(FT_f)/F_AVG<<"\tnorm(FT_mu)/"<<MU_AVG<<": "<<norm(FT_mu)/MU_AVG;

        fz_orig=fz;
        fz=3;
    }
    else if (fz != -1 && fz != 3 && fz != 4 && !nts)
        if(v) cout<<"NO CONTACT: norm(FT_f) = "<<norm(FT_f)<<"\tnorm(FT_mu) = "<<norm(FT_mu);
        //if(v) cout<<"NO CONTACT: FT = "<<(*FT).toString();
    else if (fz == 3 || fz == 4)    if(v) cout<<"TRAJECTORY IS STALLED.";
    else if ( fz == -1 )            cout<<"MY JOB IS FINISHED.";
    else                            cout<<"BOH";  //the algorithm should never be here (just a check)
        
    // constant orientation of the arm: we want the middle finger forward, and palm turned down
    if (part=="left_arm")   od=odl;
    else                    od=odr;
}
    
void armExploration::limitTorsoPitch() {
    int axis=0; // pitch joint
    double min, max;

    // sometimes it may be helpful to reduce the range of variability of the joints;
    // for example here we don't want the torso to lean out more than 30 degrees forward
    arm->getLimits(axis,&min,&max);
    arm->setLimits(axis,MIN_TORSO_PITCH,MAX_TORSO_PITCH);
}

void armExploration::printStatus() {    

    if (fz!=0 && fz!=-1) outfile<<int(done)<<" "<<(*FT).toString()<<" "<<dFT.toString()<<" "<<norm(FT_f)<<" "<<norm(FT_mu)<<" "<<norm(dFT_f)<<" "<<norm(dFT_mu)<<endl;

    if (t-t1>=PRINT_STATUS_PER_AE) {
    
        Vector xdhat,odhat,qdhat;

        x.resize(3);
        o.resize(4);
        xdhat.resize(3);
        odhat.resize(4);
        qdhat.resize(10);

        // we get the current arm pose in the operational space
        arm->getPose(x,o);

        // we get the final destination of the arm as found by the solver: it differs a bit
        // from the desired pose according to the tolerances
        arm->getDesired(xdhat,odhat,qdhat);

        double e_x=norm(xdhat-x);
        double e_o=norm(odhat-o);

        fprintf(stdout,"\n++++++++++++++++++++++++++++++ arm Exploration ++++++++++++++++++++++++++++++\n");
        fprintf(stdout,"xd    [m] = %s\t",xd.toString().c_str());
        fprintf(stdout,"od  [rad] = %s\n",od.toString().c_str());
        //fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
        fprintf(stdout,"x     [m] = %s\t",x.toString().c_str());
        fprintf(stdout,"o   [rad] = %s\n",o.toString().c_str());
        //fprintf(stdout,"odhat     [rad] = %s\n",odhat.toString().c_str());
        //fprintf(stdout,"qdhat  [degree] = %s\n",qdhat.toString().c_str());
        fprintf(stdout,"e_x   [m] = %g\t\t\t\t\t",e_x);
        fprintf(stdout,"e_o [rad] = %g\n",e_o);
        fprintf(stdout,"FT    [N] = %s\n",(*FT).toString().c_str());
        fprintf(stdout,"dFT    [] = %s\n",dFT.toString().c_str());
        fprintf(stdout,"FT_f  [N] = %g\t\t\t\t\t",norm(FT_f));
        fprintf(stdout,"FT_mu [N] = %g\n",norm(FT_mu));
        fprintf(stdout,"------------------------------------------------------------------------------\n");

        t1=t;
    }
}

bool armExploration::Need_to_Stop() {
    //function that computes if there's the need to stop the arm
    //first and simple method: give a threshold on the force and moments
    //purely empyrical values
    //if ( norm(FT_f) >= 2.5 || norm(FT_mu) >= 0.5 ) return true;
    //else return false;
    
    //second method: an OR if the signal is high, an AND if it's low (to avoid false positives)
    //computation is done over the signals averaged by their mean value (off-line taken)
    
    return false;
    
    if ( norm(FT_f)/F_AVG >= 2.7 || norm(FT_mu)/MU_AVG >= 2.7 || ( norm(FT_f)/F_AVG >= 2.1 && norm(FT_mu)/MU_AVG >= 2.1 ) ) return true;
    else return false;
}
