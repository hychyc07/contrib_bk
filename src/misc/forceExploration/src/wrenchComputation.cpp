#include "wrenchComputation.h"

wrenchComputation::wrenchComputation(int _rate, PolyDriver *_dd, PolyDriver *_tt,
            const string &_part, const string &_name, Vector &_FT) : RateThread(_rate), dd(_dd), tt(_tt) {
    
    FT_ref = &_FT;
    part = _part.c_str();
    first = true;
    
    string fwdSlash = "/";
    string port = fwdSlash+_name;
    port += (fwdSlash+part.c_str());
    
    port_FT=new BufferedPort<Vector>;
		port_FT->open((port+"/FT:i").c_str());

    dd->view(iencs);
    tt->view(tencs);

    if (part=="left_arm")   limb = new iCubArmDyn("left");
    else                    limb = new iCubArmDyn("right");
				
    limb->releaseLink(0);
	limb->releaseLink(1);
	limb->releaseLink(2);

    chain = limb->asChain();

    if (part=="left_arm")   sens = new iDynInvSensorArm(chain,"left",DYNAMIC);
    else            sens = new iDynInvSensorArm(chain,"right",DYNAMIC);
    
    //FTtoBase = new iFTransformation(sens);

    // I parametri sono la lunghezza massima della finestra e la soglia - ma per cosa la soglia?
    linEst =new AWLinEstimator(16,1.0);
    quadEst=new AWQuadEstimator(25,1.0);

    int jnt1=0;
    int jnt2=0;

    iencs->getAxes(&jnt1);
    encoders.resize(jnt1);

    tencs->getAxes(&jnt2);
    encodersT.resize(jnt2);

    int jnt=jnt1+jnt2;

    q.resize(10,0.0);
    dq.resize(10,0.0);
    d2q.resize(10,0.0);
    w0.resize(3,0.0);
    dw0.resize(3,0.0);
    d2p0.resize(3,0.0);
    Fend.resize(3,0.0);
    Mend.resize(3,0.0);
    F_measured.resize(6,0.0);
    F_iDyn.resize(6,0.0);
    F_offset.resize(6,0.0);
    FT.resize(6,0.0);
    d2p0[2]=9.81; // frame 0 acceleration
    
    ft_flag=0;

    limb->setAng(q);
    limb->setDAng(dq);
    limb->setD2Ang(d2q);
    limb->prepareNewtonEuler(DYNAMIC);
    limb->initNewtonEuler(w0,dw0,d2p0,Fend,Mend);
    
    //Vecchio
    /*F_offset[0] = 52.8346;
    F_offset[1] = 54.8612;
    F_offset[2] = 66.6180;
    F_offset[3] =  1.3391;
    F_offset[4] = -1.5392;
    F_offset[5] =  0.0369;
    //Nuovo
    F_offset[0] = 55.0447;
    F_offset[1] = 55.8306;
    F_offset[2] = 73.8924;
    F_offset[3] =  1.3349;
    F_offset[4] = -1.6129;
    F_offset[5] =  0.0234;*/
}

bool wrenchComputation::threadInit() {       
    
    t=t0=t1=Time::now();
    
    //outfile.open ("test2.txt");
    //outfile<<"###################################################\n";
    //outfile<<"#q,dq,d2q\n";
    //outfile<<"###################################################\n";
    
    return true;
}

void wrenchComputation::run() {   
    
    t=Time::now();
    
    //Questo lo provo con icub
    iencs->getEncoders(encoders.data());

    tencs->getEncoders(encodersT.data());
    for (int i=0;i<3;i++)    q(i) = encodersT(2-i);
    for (int i=3;i<q.length();i++)     q(i) = encoders(i-3);

    dq = evalVel(q);
    d2q = evalAcc(q);

    limb->setAng(CTRL_DEG2RAD * q);
    limb->setDAng(CTRL_DEG2RAD * dq);
    limb->setD2Ang(CTRL_DEG2RAD * d2q);

    limb->computeNewtonEuler(w0,dw0,d2p0,Fend,Mend);
    sens->computeSensorForceMoment();
    
    F_iDyn = -1.0 * sens->getSensorForceMoment();    //Forza stimata

    ft = port_FT->read(false);
    
    if (ft!=0) {
    
        F_measured = *ft;
        
        if (first) {
            //F_offset = F_measured-F_iDyn;
            F_offset = F_offset + F_measured-F_iDyn;
            
            if (ft_flag < 9) {
                ft_flag++;
                F_offset[0] = F_offset[1] = F_offset[2] = F_offset[3] = F_offset[4] = F_offset[5] = 0;
            }
            else if (ft_flag < 18) ft_flag++;
            else {
                for (int i=0;i<6; i++) F_offset[i] = F_offset[i]/10;
                first = false;
            }
                cout<<"\nF_offset: "<<F_offset.toString()<<"\nF_measured - F_iDyn: "<<(F_measured - F_iDyn).toString();
        }

        if (!first)
        FT = F_measured - F_offset - F_iDyn;
    }
    
    //trasmit the force
    for (int i=0; i<FT.length(); i++)     (*FT_ref)[i] = FT[i];
    
    printStatus();
}

void wrenchComputation::threadRelease() {

    //outfile.close();
    
    if (sens) {
        delete sens;
        sens = 0;
    }

    if (limb) {
        delete limb;
        limb = 0;
    }

    if (linEst) {
        delete linEst;
        linEst = 0;
    }

    if (quadEst) {
        delete quadEst;
        quadEst = 0;
    }

    if (ft) {
        delete ft;
        ft = 0;
    }

		/*if (port_FT)
    {
        port_FT->interrupt();
        port_FT->close();

        delete port_FT;
        port_FT = 0;
    }*/
}   
    
void wrenchComputation::printStatus() {    

    //outfile<<q.toString()<<"\t"<<d2q.toString()<<"\t"<<dq.toString()<<"\t"<<endl;
    
    /*if (t-t1>=PRINT_STATUS_PER_WR) {
    
        fprintf(stdout,"\n++++++++++++++++++++++++++++++ wrench Computation ++++++++++++++++++++++++++++++\n");
        //cout<<"q:   "<<q.toString()<<endl;
        //cout<<"dq:  "<<dq.toString()<<endl;
        //cout<<"d2q: "<<d2q.toString()<<endl;
        cout<<"F_measured:\t"<<F_measured.toString()<<endl;
        cout<<"F_iDyn:  \t"<<F_iDyn.toString()<<endl;
        cout<<"F_offset:\t"<<F_offset.toString()<<endl;
        cout<<"FT:     \t"<<FT.toString()<<endl;
        fprintf(stdout,"---------------------------------------------------------------------------------\n");

        t1=t;
    }*/
}
