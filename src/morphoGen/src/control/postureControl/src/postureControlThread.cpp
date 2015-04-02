// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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

/**
 * @file postureControlThread.cpp
 * @brief Implementation of the eventDriven thread (see postureControlThread.h).
 */

#include <iCub/postureControlThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;


const double postureControlThread::armMax[] = {-10.0, 70.0,  30.0, 80.0,  50.0,   0.0 };
const double postureControlThread::armMin[] = {-70.0, 20.0, -20.0, 20.0, -50.0, -30.0 };  

postureControlThread::postureControlThread() {
    robot = "icub";        
}

postureControlThread::postureControlThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

postureControlThread::~postureControlThread() {
    // do nothing
}

bool postureControlThread::threadInit() {

    //initialization of the controller
    bool ok = initController();
    if(!ok){
        return false;
    }
    
    if (!inputLeftArm.open(getName("/leftArm:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
   
    if (!inputRightArm.open(getName("/rightArm:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    return true;
    

}

bool postureControlThread::initController() {
    Property options;

    // ================== instantiate right arm ==================================
    
    options.put("device", "remote_controlboard");
    options.put("local", "/postureControl/client/rightarm");                 //local port names
    string remoteRightArm("/");
    remoteRightArm.append(robot.c_str());
    remoteRightArm.append("/right_arm");
    options.put("remote",(const char*)remoteRightArm.c_str());      //where we connect to
    
    robotDeviceRightArm = new PolyDriver(options);
   
    if (!robotDeviceRightArm->isValid()) {
      printf("Device Right Arm not available.  Here are the known devices:\n");
      printf("%s", Drivers::factory().toString().c_str());
      return 0;
    }

    bool okRight;
    okRight = robotDeviceRightArm->view(posRightArm);
    okRight = okRight && robotDeviceRightArm->view(encsRightArm);
    okRight = okRight && robotDeviceRightArm->view(ictrlRightArm);
    okRight = okRight && robotDeviceRightArm->view(iimpRightArm);
    okRight = okRight && robotDeviceRightArm->view(itrqRightArm);
    if (!okRight) {
        printf("Problems acquiring interfaces from the right arm\n");
        return 0;
    }
    else {
        printf("Ok. proceed! \n");
    }

    
    Time::delay(0.01);
    cout<<"after"<<endl;

    // checking the readings
    encodersRightArm.resize(16);
    bool getRightCorrect = encsRightArm->getEncoders(encodersRightArm.data());
    printf("initial encoders position (%s) \n",encodersRightArm.toString().c_str());
    
    
    if(!getRightCorrect){
        printf("just read crap from encoders \n");
        return 0;
    }
    else{
        printf("correct encoders \n");
    }

    // ================== instantiate left arm ==================================
    
    options.put("device", "remote_controlboard");
    options.put("local", "/postureControl/client/leftarm");                 //local port names
    string remoteLeftArm("/");
    remoteLeftArm.append(robot.c_str());
    remoteLeftArm.append("/left_arm");
    options.put("remote",(const char*)remoteLeftArm.c_str());      //where we connect to
    
    robotDeviceLeftArm = new PolyDriver(options);
   
    if (!robotDeviceLeftArm->isValid()) {
      printf("Device Left Arm not available.  Here are the known devices:\n");
      printf("%s", Drivers::factory().toString().c_str());
      return 0;
    }

    bool okLeft;
    okLeft = robotDeviceLeftArm->view(posLeftArm);
    okLeft = okLeft && robotDeviceLeftArm->view(encsLeftArm);
    okLeft = okLeft && robotDeviceLeftArm->view(ictrlLeftArm);
    okLeft = okLeft && robotDeviceLeftArm->view(iimpLeftArm);
    okLeft = okLeft && robotDeviceLeftArm->view(itrqLeftArm);
    if (!okLeft) {
        printf("Problems acquiring interfaces from the left arm\n");
        return 0;
    }

    // ================= interfacing to robot parts ==========================

    int jnts = 0;
    posRightArm->getAxes(&jnts);
    jntsRightArm = jnts;
    
    Vector tmp;
    Vector command_velocity;

    tmp.resize(jnts);
    encodersRightArm.resize(jnts);

    // setting reference acceleration
    int i;
    for (i = 0; i < jnts; i++) {
        tmp[i] = 50.0;
    }    
    posRightArm->setRefAccelerations(tmp.data());
    // setting reference speed
    for (i = 0; i < jnts; i++) {
        tmp[i] = 10.0;
        posRightArm->setRefSpeed(i, tmp[i]);
    }
    //==================== checking the readings ===================================
    
   

    Vector command_position;
    posRightArm->getAxes(&jntsRightArm);
    printf("got the number of axis initialization %d \n", jntsRightArm);
    Vector torques;
    torques.resize(jntsRightArm);
    itrqRightArm->getTorques(torques.data());
    printf("got the reading of the torques %s \n", torques.toString().c_str());

    // =================== setting impedence control  ===============================

    double stiffness = 0.081;       // stiffness coefficient, units are Nm/deg
    double damping   = 0.020;       // damping coefficient,   units are Nm/(deg/s)
    double offset    = 0.0;         // torque offset,         units are Nm
    bool okImpP = iimpRightArm->setImpedance(3, stiffness, damping);  
    bool okImpO = iimpRightArm->setImpedanceOffset(3,offset);
    bool okImp  = ictrlRightArm->setImpedancePositionMode(3);

    if(okImpP & okImp & okImpO) {
        printf("success in sending switching to impedence mode control \n");
    }
    else {
        printf("Error! in sending switching to impedence mode control \n");
        return false;
    }

    // =================== setting torque control ==================================

    //ictrlLeftArm->setTorqueMode(3);
    //double jnt_torque= 0.0; //expressed in Nm
    //itrqLeftArm->setRefTorque(3,jnt_torque); 

    okImpP = iimpLeftArm->setImpedance(3, stiffness, damping);  
    okImpO = iimpLeftArm->setImpedanceOffset(3,offset);
    okImp  = ictrlLeftArm->setImpedancePositionMode(3);

    if(okImpP & okImp & okImpO) {
        printf("success in sending switching to impedence mode control \n");
    }
    else {
        printf("Error! in sending switching to impedence mode control \n");
        return false;
    }
    

    //==================== moving to default position ==============================
    
    command_position.resize(jntsRightArm);
    command_position[0]  = -30;
    command_position[1]  = 30;
    command_position[2]  = 0;
    command_position[3]  = 45;
    command_position[4]  = 0;
    command_position[5]  = 0;
    command_position[6]  = 0;
    command_position[7]  = 15;
    command_position[8]  = 30;
    command_position[9]  = 4;
    command_position[10] = 1;
    command_position[11] = 9;
    command_position[12] = 0;
    command_position[13] = 4;
    command_position[14] = 1;
    command_position[15] = 1;
    
    printf("sending command %s \n", command_position.toString().c_str());
    
    
    posRightArm->positionMove(command_position.data());
    
    return true;
}

void postureControlThread::setName(string str) {
    this->name=str;
    printf("name: %s \n", name.c_str());
}


std::string postureControlThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

bool postureControlThread::checkArm(Bottle* b) {
    printf("dimension of the received bottle %d \n", b->size());
    printf("componets of the received bottle  \n");
    Bottle* values =   b->get(0).asList();

    printf("content of the bottle %s \n", values->toString().c_str());
    printf("content comprises %d values \n", values->size());
        
    if (values->size() > 16) {
        return false;
    }
    
    for(int i = 0; i < values->size() ; i++){
        double b = values->get(i).asDouble();
        if((b < armMin[i]) || (b > armMax[i])) {
            printf("b value %f out of limit \n", b);
            return false;        
        }
    }
        
    return true;
}

Vector postureControlThread::parseBottle(Bottle* b, int dim) {
    Bottle* values =   b->get(0).asList();
    Vector result(dim); 
    encsRightArm->getEncoders(encodersRightArm.data());
    printf("encoders position \n (%s) \n",encodersRightArm.toString().c_str());
    for (int i = 0; i < dim ; i++) {
        if( i < values->size()) {
            result[i] = values->get(i).asDouble();
        }
        else{            
            result[i] = encodersRightArm(i);
        }
    }    
    return result;
}

void postureControlThread::setInputPortName(string InpPort) {
    
}

void postureControlThread::run() {
    while (isStopping() != true) {

        Bottle* receivedBottle;

        //**********************************************************
        if (inputRightArm.getInputCount()) {
            
            receivedBottle = inputRightArm.read(false);
            if(receivedBottle != NULL){
                printf("Bottle %s \n", receivedBottle->toString().c_str());

                bool rightArmOk = checkArm(receivedBottle);

                if(rightArmOk) {

                    int jnts = 0;
                    //Vector command_position;
                    Vector command_position = parseBottle(receivedBottle, 16);                    
                    
                    command_position.resize(jntsRightArm);
                    
                    printf("jnt dimension %d \n", jntsRightArm);
                    
                    /*
                    command_position[0]  = -45;
                    command_position[1]  = 65;
                    command_position[2]  = 0;
                    command_position[3]  = 15;
                    command_position[4]  = 0;
                    command_position[5]  = 0;
                    command_position[6]  = 0;
                    command_position[7]  = 15;
                    command_position[8]  = 30;
                    command_position[9]  = 4;
                    command_position[10] = 1;
                    command_position[11] = 9;
                    command_position[12] = 0;
                    command_position[13] = 4;
                    command_position[14] = 1;
                    command_position[15] = 1;
                    */
                    
                    printf("sending command\n %s \n", command_position.toString().c_str());
                    //printf("temporary command\n %s \n", command_position_temp.toString().c_str());

                    bool ok = posRightArm->positionMove(command_position.data());
                    if(!ok){
                        break;
                    }
                }
                else {
                    printf("detected out of limits control \n");
                }
            }
        }

        //**************************************************************

        if (inputLeftArm.getInputCount()) {
            
            receivedBottle = inputLeftArm.read(false);
            if(receivedBottle != NULL){
                printf("Bottle %s \n", receivedBottle->toString().c_str());

                bool leftArmOk = checkArm(receivedBottle);

                if(leftArmOk) {

                    int jnts = 0;
                    //Vector command_position;
                    Vector command_position = parseBottle(receivedBottle, 16);                    
                    
                    command_position.resize(jntsRightArm);
                    
                    printf("jnt dimension %d \n", jntsRightArm);
                    
                    /*
                    command_position[0]  = -45;
                    command_position[1]  = 65;
                    command_position[2]  = 0;
                    command_position[3]  = 15;
                    command_position[4]  = 0;
                    command_position[5]  = 0;
                    command_position[6]  = 0;
                    command_position[7]  = 15;
                    command_position[8]  = 30;
                    command_position[9]  = 4;
                    command_position[10] = 1;
                    command_position[11] = 9;
                    command_position[12] = 0;
                    command_position[13] = 4;
                    command_position[14] = 1;
                    command_position[15] = 1;
                    */
                    
                    printf("sending command\n %s \n", command_position.toString().c_str());
                    //printf("temporary command\n %s \n", command_position_temp.toString().c_str());

                    bool ok = posLeftArm->positionMove(command_position.data());
                    if(!ok){
                        break;
                    }
                }
                else {
                    printf("detected out of limits control \n");
                }
            }
        }

        Time::delay(0.1);
    }               
}

void postureControlThread::threadRelease() {
    robotDeviceRightArm->close();
    robotDeviceLeftArm->close();
    printf("postureControlThread::threadRelease \n");  
}

void postureControlThread::onStop() {
    
    ictrlRightArm->setPositionMode(3);
    ictrlLeftArm->setPositionMode(3);

    printf("postureControlThread::onStop \n");
    inputRightArm.interrupt();
    inputLeftArm.interrupt();
    printf("interrupted the port \n");
    inputRightArm.close();
    inputLeftArm.close();
    printf("postureControlThread::onStop \n");
}

