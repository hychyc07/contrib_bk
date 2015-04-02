#include "iCubDblTchThrd.h"

doubleTouchThread::doubleTouchThread(int _dtt_rate, const string &_name, const string &_robot, int _v, string _type) :
                                      RateThread(_dtt_rate), name(_name), robot(_robot), verbosity(_v), testLimb(_type)
{
    step = 0;
    cntctRdr = new BufferedPort<iCub::skinDynLib::skinContactList>;

    probl = new iCubDoubleTouch_Problem(_type);        // problem
    g     = new iCubDoubleTouch_Variables(probl->getNVars()); // guess
    s0    = new iCubDoubleTouch_Variables(probl->getNVars()); // solution - waypoint
    s1    = new iCubDoubleTouch_Variables(probl->getNVars()); // solution

    (*g).joints[1] = -45.0*CTRL_DEG2RAD; (*g).joints[3] = -30.0*CTRL_DEG2RAD;
    (*g).joints[4] =  30.0*CTRL_DEG2RAD; (*g).joints[5] = -30.0*CTRL_DEG2RAD;
    (*g).joints[6] =  30.0*CTRL_DEG2RAD; (*g).joints[8] =  45.0*CTRL_DEG2RAD;

    s0->clone(*g);

    probl -> limb.setAng(g->joints);
    slv   =  new iCubDoubleTouch_Solver(*probl);

    fal = 2; ual = -1;
    // far = 5; uar = -1;
    far = -1; uar = -1;

    contextGaze = -1;

    cntctPosLink.resize(3,0.0);
    cntctPosWRF.resize(3,0.0);
    cntctPosEE.resize(3,0.0);
    cntctNormDir.resize(3,0.0);
    cntctPressure = -1;
    cntctLinkNum = -1;
    cntctArm = "";
    cntctH0 = eye(4);
}

bool doubleTouchThread::threadInit()
{
    cntctRdr      -> open(("/"+name+"/contacts:i").c_str());

    Property OptGaze;
    OptGaze.put("device","gazecontrollerclient");
    OptGaze.put("remote","/iKinGazeCtrl");
    OptGaze.put("local",("/"+name+"/gaze").c_str());

    Property OptR;
    OptR.put("robot",  robot.c_str());
    OptR.put("part",   "right_arm");
    OptR.put("device", "remote_controlboard");
    OptR.put("remote",("/"+robot+"/right_arm").c_str());
    OptR.put("local", ("/"+name+"/posCtrl/right_arm").c_str());

    Property OptL;
    OptL.put("robot",  robot.c_str());
    OptL.put("part",   "left_arm");
    OptL.put("device", "remote_controlboard");
    OptL.put("remote",("/"+robot+"/left_arm").c_str());
    OptL.put("local", ("/"+name+"/posCtrl/left_arm").c_str());

    if ((!ddG.open(OptGaze)) || (!ddG.view(igaze))){
        printMessage(0,"Error: could not open the Gaze Controller!\n");
        return false;
    }

    igaze -> storeContext(&contextGaze);
    igaze -> setNeckTrajTime(1.5);
    igaze -> setEyesTrajTime(0.5);
    igaze -> setSaccadesStatus(false);

    if (!ddR.open(OptR))
    {
        printMessage(0,"ERROR: could not open right_arm PolyDriver!\n");
        return false;
    }

    // open the view
    bool ok = 1;
    if (ddR.isValid())
    {
        ok = ok && ddR.view(iencsR);
        ok = ok && ddR.view(iposR);
        ok = ok && ddR.view(ictrlR);
        ok = ok && ddR.view(ilimR);
    }

    if (!ok)
    {
        printMessage(0,"\nERROR: Problems acquiring right_arm interfaces!!!!\n");
        return false;
    }

    iencsR->getAxes(&jntsR);
    encsR = new Vector(jntsR,0.0);

    if (!ddL.open(OptL))
    {
        printMessage(0,"ERROR: could not open left_arm PolyDriver!\n");
        return false;
    }

    // open the view
    ok = 1;
    if (ddL.isValid())
    {
        ok = ok && ddL.view(iencsL);
        ok = ok && ddL.view(iposL);
        ok = ok && ddL.view(ictrlL);
        ok = ok && ddL.view(ilimL);
    }

    if (!ok)
    {
        printMessage(0,"\nERROR: Problems acquiring left_arm interfaces!!!!\n");
        return false;
    }

    iencsL->getAxes(&jntsL);
    encsL = new Vector(jntsL,0.0);

    alignJointsBounds();

    Vector joints;
    iencsR->getEncoders(encsR->data());
    probl->index.getChainJoints(*encsR,joints);
    Matrix indexH=probl->index.getH(joints*CTRL_DEG2RAD);
    printf("indexH:\n%s\n", indexH.toString().c_str());

    probl->limb.setHN(indexH);
    testLimb.setHN(indexH);

    return true;
}

void doubleTouchThread::run() {
    // printf("positionmove: %i\n",iposR -> positionMove(2, 84.406405));
    skinContactList *skinContacts  = cntctRdr -> read(false);
    handleGaze();

    if (checkMotionDone())
    {
        // cout << "step: " << step << endl;
        if (step == 0) {
            printMessage(1,"dblTchThrd: switching to impedance position mode..\n");

            for (int i = 0; i < 5; ++i)
            {
                // ictrlR -> setImpedancePositionMode(i);
                if (i!=4 && i!=1 && i!=0)
                {
                    ictrlL -> setImpedancePositionMode(i);
                }

            }

            printMessage(1,"Moving to rest...\n");
            goToRest();
            // move the thumbs close to the hand
            iposR -> positionMove(9,90);
            iposR -> positionMove(8,10);
            iposL -> positionMove(9,90);
            iposL -> positionMove(8,10);

            step++;
            printMessage(0,"*************\nWAITING FOR CONTACT...\n");
        }
        else if (step == 1)
        { 
            if(skinContacts)
            {
                detectContact(skinContacts); // READ A CONTACT ON THE SKIN
                if (cntctArm != "")
                {
                    printMessage(0,"CONTACT!!! Arm: %s Position: %s NormDir: %s Pressure: %g ID: %i\n",cntctArm.c_str(),
                               cntctPosLink.toString().c_str(),cntctNormDir.toString().c_str(),cntctPressure,cntctSkin.getId());
                    step+=2;
                }
            }
        }
        else if (step == 2) {
            printMessage(1,"Going to waypoint...\n");
            goToTaxel("waypoint");
            delay(10);
            step++;
        }
        else if (step == 3) {
            goToTaxel();
            printMessage(0,"Going to taxel...Desired EE: %s\n",(s1->ee).toString().c_str());
            printMessage(0,"jnts=%s\n",(s1->joints*CTRL_RAD2DEG).toString().c_str());
            delay(10);
            testAchievement();
            printMessage(0,"                 Final EE    %s\n", testLimb.EndEffPosition().toString().c_str());
            printMessage(0,"jnts=%s\n",(testLimb.getAng()*CTRL_RAD2DEG).toString().c_str());
            step++;
        }
        else if (step == 4) {
            printMessage(1,"Going to rest...\n");
            goToRest();
            delay(5);
            printMessage(0,"*************\nWAITING FOR CONTACT...\n");
            step = 1;
        }
        else {
            printMessage(0,"ERROR!!! doubleTouchThread should never be here!!!\nStep: %d",step);
            delay(1);
        }
    }
}

void doubleTouchThread::delay(int sec) {
    for (int i = 0; i < sec*2; ++i)
    {
        Time::delay(0.5);
        handleGaze();
    }
}

void doubleTouchThread::handleGaze()
{
    if (step == 0 || step == 1 || step == 4) {
      Vector ang(3,0.0);
      igaze -> lookAtAbsAngles(ang);
    }
    else
    {   
        cntctPosWRF = locateContact(cntctSkin);
        /*printf("cntctPosWRF: %s\n",cntctPosWRF.toString().c_str());*/
        igaze -> lookAtFixationPoint(cntctPosWRF);
    }
}

void doubleTouchThread::testAchievement()
{
    iencsL->getEncoders(encsL->data());
    iencsR->getEncoders(encsR->data());

    testLimb.setAng((*encsL)*CTRL_DEG2RAD,(*encsR)*CTRL_DEG2RAD);
}

void doubleTouchThread::goToTaxel(string s="standard")
{
    cntctH0 = findH0(cntctSkin);

    if (s == "waypoint")
    {
        printMessage(0,"H0: \n%s\n",cntctH0.toString().c_str());
        Matrix safetyPoint = eye(4);
        safetyPoint(0,3) = 0.02; // let's move 2 cm from the cover
        cntctH0 = cntctH0 * safetyPoint;
    }

    probl->limb.setH0(SE3inv(cntctH0));
    testLimb.setH0(SE3inv(cntctH0));

    Vector sol;

    if (s == "waypoint")
    {
        probl->limb.setAng(g->joints);
        slv->setInitialGuess(*g);
        slv->solve(*s0);
        s0->print();
        sol = CTRL_RAD2DEG * s0->joints;
    }
    else
    {
        probl->limb.setAng(s0->joints);
        slv->setInitialGuess(*s0);
        slv->solve(*s1);
/*      s1->print();*/
        sol = CTRL_RAD2DEG * s1->joints;
    }

    iposR -> positionMove(0,sol[5]);
    iposR -> positionMove(1,sol[6]);
    iposR -> positionMove(2,sol[7]);
    iposR -> positionMove(3,sol[8]);
    iposR -> positionMove(4,sol[9]);
    iposR -> positionMove(5,sol[10]);
    iposR -> positionMove(6,sol[11]);

    delay(3);

    iposL -> positionMove(4,-sol[0]);
    iposL -> positionMove(3,-sol[1]);
    iposL -> positionMove(2,-sol[2]);
    iposL -> positionMove(1,-sol[3]);
    iposL -> positionMove(0,-sol[4]);

    testLimb.setAng(s1->joints);
}

void doubleTouchThread::goToRest()
{   
    Vector rest = CTRL_RAD2DEG * g->joints;

    iposL -> positionMove(4,-rest[0]);
    iposL -> positionMove(3,-rest[1]);
    iposL -> positionMove(2,-rest[2]);
    iposL -> positionMove(1,-rest[3]);
    iposL -> positionMove(0,-rest[4]);

    delay(3);

    iposR -> positionMove(0,rest[5]);
    iposR -> positionMove(1,rest[6]);
    iposR -> positionMove(2,rest[7]);
    iposR -> positionMove(3,rest[8]);
    iposR -> positionMove(4,rest[9]);
    iposR -> positionMove(5,rest[10]);
    iposR -> positionMove(6,rest[11]);

    while(!checkMotionDone())   Time::delay(1);
}

bool doubleTouchThread::alignJointsBounds()
{
    deque<IControlLimits*> lim;
    lim.push_back(ilimL);
    lim.push_back(ilimR);

    if (testLimb.alignJointsBounds(lim) == 0) return false;
    if (probl->limb.alignJointsBounds(lim) == 0) return false;

    lim.pop_front();
    if (probl->index.alignJointsBounds(lim) == 0) return false;

    return true;
}

void doubleTouchThread::detectContact(skinContactList *_sCL)
{
    // Reset variables:
    cntctPosLink.resize(3,0.0);
    cntctPosWRF.resize(3,0.0);
    cntctNormDir.resize(3,0.0);
    cntctPressure = -1;
    cntctLinkNum = -1;
    cntctArm = "";

    // Search for a suitable contact:
    for(skinContactList::iterator it=_sCL->begin(); it!=_sCL->end(); it++) {
        int skinPart = it -> getSkinPart(); // Retrieve the skinPart of the skinContact

        if(skinPart == fal || skinPart == ual || skinPart == far || skinPart == uar)
        {
            // Store the skinContact for eventual future use
            cntctSkin = *it;
            // Get the position of the contact:
            cntctPosLink = it -> getGeoCenter();
            // Retrieve the link number in order to find the origin of the reference frame
            cntctLinkNum = it -> getLinkNumber();
            // Retrieve the normal direction of the contact
            cntctNormDir = it -> getNormalDir();
            // Retrieve the pressure of the contact
            cntctPressure = it -> getPressure();

            if(skinPart == fal || skinPart == ual)
            {
                cntctArm = "left_arm";
            }
            else if(skinPart == far || skinPart == uar)
            {
                cntctArm = "right_arm";
            }
            cntctPosWRF = locateContact(cntctSkin);
            cntctH0     = findH0(cntctSkin);
            break;
        }
    }
}

Vector doubleTouchThread::locateContact(skinContact &sc)
{
    Vector result(4,0.0);
    Matrix Twl = eye(4);
    iCubArm *lmb = new iCubArm("left");

    iencsL->getEncoders(encsL->data());
    Vector q(10,0.0);
    q.setSubvector(3,encsL->subVector(0,6));

    // printf("encsL%s\n", q.toString().c_str());
    lmb -> setAng(q*CTRL_DEG2RAD);

    Twl = lmb -> getH(cntctLinkNum+3, true);
    Vector posLink = cntctPosLink;
    posLink.push_back(1);
    result = Twl * posLink;
    result.pop_back();

    delete lmb;
    return result;
}

Matrix doubleTouchThread::findH0(skinContact &sc)
{
    // Set the proper orientation for the touching end-effector
    Matrix H0(4,4);
    Vector x(3,0.0), z(3,0.0), y(3,0.0);

    x = sc.getNormalDir();
    z[0] = -x[2]/x[0]; z[2] = 1;
    y = -1*(cross(x,z));
    
    // Let's make them unitary vectors:
    x = x / norm(x);
    y = y / norm(y);
    z = z / norm(z);

    H0.zero();
    H0(3,3) = 1;
    H0.setSubcol(x,0,0);
    H0.setSubcol(y,0,1);
    H0.setSubcol(z,0,2);
    H0.setSubcol(sc.getGeoCenter(),0,3);
    return H0;
}

void doubleTouchThread::closePort(Contactable *_port)
{
    if (_port)
    {
        _port -> interrupt();
        _port -> close();

        delete _port;
        _port = 0;
    }
}

int doubleTouchThread::printMessage(const int level, const char *format, ...) const
{
    if (verbosity>=level)
    {
        fprintf(stdout,"*** %s: ",name.c_str());

        va_list ap;
        va_start(ap,format);
        int ret=vfprintf(stdout,format,ap);
        va_end(ap);

        return ret;
    }
    else
        return -1;
}

void doubleTouchThread::threadRelease()
{
    printMessage(0,"Returning to position mode..\n");
        for (int i = 0; i < 5; ++i)
        {
            ictrlR -> setPositionMode(i);
            ictrlL -> setPositionMode(i);
        }

    printMessage(0,"Closing ports..\n");
        closePort(cntctRdr);
        printMessage(1,"skin port successfully closed!\n");

    printMessage(0,"Closing controllers..\n");
        ddR.close();
        ddL.close();
        igaze -> restoreContext(contextGaze);
        igaze -> stopControl();
        ddG.close();
}

// empty line to make gcc happy