/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Bjoern Browatzki
 * email:   bjoern.browatzki@tuebingen.mpg.de
 * website: www.robotcub.org 
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

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <iCub/InvKinThread.h>
#include <iCub/ObjRecModule.h>
#include <iCub/HandPoseUtil.h>

#define CARTESIAN_MOVE 0
#define POSITION_MOVE 0
#define POSITION_MOVE_SINGLE_JOINTS 1

double minLimits[] = {
        -90, // 0
        30,  // 1
        -15, // 2
        30,  // 3
        -68, // 4
        -30, // 5
        -10  // 6
};
double maxLimits[] = {
        -40, // 0
        70,  // 1
        30,  // 2
        100, // 3
        -15,  // 4
        -2,   // 5
        30   // 6
};
double phaseTimes[] = {
        65,
        55,
        58,
        20,
        40,
        20,
        25 
};
double phaseTimesTorso[] = {
        45,
        35,
        38,
};

double minLimitsTorso[] = {-20, 0, 0};
double maxLimitsTorso[] = {0, 0, 5}; 


void InvKinThread::threadRelease() 
{
    if (velCtrl) velCtrl->stop();
    if (torsoVelCtrl) torsoVelCtrl->stop();
    if (armCtrl) armCtrl->stop();
    if (torsoCtrl) torsoCtrl->stop();
}

bool InvKinThread::threadInit() 
{
    if (! robotArm.isValid()) 
    {
        std::cout << "Cannot connect to robot right arm" << std::endl;  
        std::cout << "Device not available.  Here are the known devices:" << std::endl;
        std::cout << Drivers::factory().toString() << std::endl;
        return false;
    }
    if (! robotArm.view(velCtrl))
    {
        std::cout << "Cannot get interface to the arm" << std::endl;  
        robotArm.close();
        return false;
    }
    if (! robotArm.view(armCtrl))
    {
        std::cout << "Cannot get interface to the arm" << std::endl;  
        robotArm.close();
        return false;
    }
    if (! robotArm.view(limArm))
    {
        std::cout << "Cannot get interface to the arm" << std::endl;  
        robotArm.close();
        return false;
    }
    if (! robotArm.view(encArm))
    {
        std::cout << "Cannot get interface to the arm" << std::endl;  
        robotArm.close();
        return false;
    }
    if (! armCartDriver.view(armCart))
    {
        std::cout <<  "Cannot get cartesian interface to the arm" << std::endl;  
        armCartDriver.close();
        return false;
    }

    // Torso 
    
    if (! robotTorso.isValid()) 
    {
        std::cout << "Cannot connect to robot torso" << std::endl;  
        std::cout << "Device not available.  Here are the known devices:" << std::endl;
        std::cout << Drivers::factory().toString() << std::endl;
        return false;
    }
    if (! robotTorso.view(encTorso))
    {
        std::cout << "Cannot get interface to torso" << std::endl;  
        robotTorso.close();
        return false;
    }
    if (! robotTorso.view(torsoCtrl))
    {
        std::cout << "Cannot get interface to torso" << std::endl;  
        robotTorso.close();
        return false;
    }
    if (! robotTorso.view(torsoVelCtrl))
    {
        std::cout << "Cannot get interface to torso" << std::endl;  
        robotTorso.close();
        return false;
    }
    if (! robotTorso.view(limTorso))
    {
        std::cout << "Cannot get interface to torso" << std::endl;  
        robotTorso.close();
        return false;
    }

    kinExpSphere.clear();

    kinExpTargetQ.clear();
    nextQ = Vector(NUM_ARM_JOINTS, minLimits);

    invKinExplorationMap = cv::Mat(45, 45, CV_8U, cv::Scalar::all(255));

    explSampleCount = 0;
    lastInvKinPrintTime = 0;

    // clear data file
    //handGazeControl.deleteTrainingData();
    
    threadStartTime = Time::now();

    std::cout << "Thread init done." << std::endl;
    return true;
}

void InvKinThread::run()
{
    if (filenameTrainingData.empty())
    {
        std::cerr << "!!! Please specify a file to save training data. Call InvKinThread::setSampleFile()." << std::endl;
        stop();
        return;
    }
    invKinExploration(Time::now()-threadStartTime);
}

#if CARTESIAN_MOVE
void InvKinThread::invKinExploration(double time)
{

    int nAxesTorso;
    int nAxesArm;
    encTorso->getAxes(&nAxesTorso);
    encArm->getAxes(&nAxesArm);
    Vector torsoQ(nAxesTorso);
    Vector armQ(nAxesArm);
    encTorso->getEncoders(torsoQ.data());
    encArm->getEncoders(armQ.data());

    double e, r;
    Vector handX(3);
    Vector handO(4);
    armCart->getPose(handX, handO);
    Vector eyeX = module->getEyePosition();
    HandPoseUtil::getGazeAngles(eyeX, handX, handO, e, r, !module->isRightArm());

    saveTrainingSample(filenameTrainingData, e, r, handX, eyeX, torsoQ, armQ, handO);

    explSampleCount++;
    kinExpSphere.markGaze(e, r);
    if (time-lastInvKinPrintTime > 1.0)
    {
        lastInvKinPrintTime = time;
        std::cout << "Samples, time(min), samples/sec: \t" << explSampleCount << "\t" 
            << (int)time/60 << "\t" << (double)explSampleCount/time << std::endl;
    }
    kinExpSphere.show("Kinexp", 5);

    
    // check if last target is reached, only take into account torso and arm+wrist joints
    if (kinExpTargetQ.size() > 0)
    {
        for (int joint = 0; joint < 7; ++joint)
        {
            double d = std::abs(kinExpTargetQ[joint+3] - armQ[joint]);
            if (d > 3.)
            {
                return;
            } 
        }
    }

    // go to new position

    Vector minLimitsCart(3);
    Vector maxLimitsCart(3);

    minLimitsCart[0] = -25.0;    // [deg] // up/down
    minLimitsCart[1] = -20.0;    // [deg] // left/right
    minLimitsCart[2] = 0.12;     // [m]

    maxLimitsCart[0] = 50.0;
    maxLimitsCart[1] = 70.0;
    maxLimitsCart[2] = 0.35;


    // pick random point (spherical coords)
    Vector sph_p(3);
    for (int i = 0; i < 3; i++)
    {
        sph_p[i] = yarp::os::Random::uniform()*(maxLimitsCart[i] - minLimitsCart[i]) + minLimitsCart[i];
    }

    // convert spherical to cartesian
    handX = Util::sphericalToCartesian(sph_p);

    handX[0] += eyeX[0];
    handX[1] += eyeX[1];
    handX[2] += eyeX[2];

    // convert cartesian to joints
    Vector xhat,ohat,qhat;
    armCart->askForPosition(handX, xhat, ohat, qhat);


    // make sure desired joints are within limits
    for (int i = 0; i < 4; i++)
    {
        qhat[i+3] = std::max(minLimits[i], std::min(maxLimits[i], qhat[i+3]));
    }


    // move torso
    //for (int i = 0; i < 3; i++)
    //{
        //Util::safePositionMove(i, qhat[i], torsoCtrl, limTorso);
    //}
    // move arm
    for (int i = 0; i < 4; i++)
    {
        Util::safePositionMove(i, qhat[i+3], armCtrl, limArm);
    }

    // set random wrist orientation
    int ARM_ROT_JNT = 4;
    int WRIST_JNT_1 = 5;
    int WRIST_JNT_2 = 6;
    double qrot = yarp::os::Random::uniform() * (maxLimits[ARM_ROT_JNT] - minLimits[ARM_ROT_JNT]) + minLimits[ARM_ROT_JNT];
    double q1 = yarp::os::Random::uniform() * (maxLimits[WRIST_JNT_1] - minLimits[WRIST_JNT_1]) + minLimits[WRIST_JNT_1];
    double q2 = yarp::os::Random::uniform() * (maxLimits[WRIST_JNT_2] - minLimits[WRIST_JNT_2]) + minLimits[WRIST_JNT_2];
    //std::cout << "q1: " << q1 << std::endl;
    //std::cout << "q2: " << q2 << std::endl;
    Util::safePositionMove(ARM_ROT_JNT, qrot, armCtrl, limArm);
    Util::safePositionMove(WRIST_JNT_1, q1, armCtrl, limArm);
    Util::safePositionMove(WRIST_JNT_2, q2, armCtrl, limArm);
    qhat[ARM_ROT_JNT+3] = qrot;
    qhat[WRIST_JNT_1+3] = q1;
    qhat[WRIST_JNT_2+3] = q2;

    std::cout << "qhat: " << qhat.toString() << std::endl;
    kinExpTargetQ = qhat; 
}
#elif POSITION_MOVE_SINGLE_JOINTS
void InvKinThread::invKinExploration(double time)
{
    int nAxesTorso;
    int nAxesArm;
    encTorso->getAxes(&nAxesTorso);
    encArm->getAxes(&nAxesArm);
    Vector torsoQ(nAxesTorso);
    Vector armQ(nAxesArm);
    encTorso->getEncoders(torsoQ.data());
    encArm->getEncoders(armQ.data());

    double e, r;
    Vector handX(3);
    Vector handO(4);
    armCart->getPose(handX, handO);
    Vector eyeX = module->getEyePosition();
    HandPoseUtil::getGazeAngles(eyeX, handX, handO, e, r, !module->isRightArm());

    saveTrainingSample(filenameTrainingData, e, r, handX, eyeX, torsoQ, armQ, handO);

    explSampleCount++;
    kinExpSphere.markGaze(e, r);
    if (time-lastInvKinPrintTime > 1.0)
    {
        lastInvKinPrintTime = time;
        std::cout << "Samples, time(min), samples/sec: \t" << explSampleCount << "\t" 
            << (int)time/60 << "\t" << (double)explSampleCount/time << std::endl;
    }
    kinExpSphere.show("Kinexp", 5);

    if (kinExpTargetQ.size() == 0 )
    {
        // set new target value
        for (int j = 0; j < 7; ++j)
        {
            Util::safePositionMove(j, nextQ[j], armCtrl, limArm);
        }
        kinExpTargetQ = Vector(NUM_ARM_JOINTS+3);
    }
    else 
    {
        for (int j = 0; j < 7; ++j)
        {
            if (std::abs(kinExpTargetQ[j+3] - armQ[j]) < 3.0)
            {
                double distMin = std::abs(minLimits[j] - armQ[j]);
                double distMax = std::abs(maxLimits[j] - armQ[j]);
                if (distMin < distMax)
                {
                    nextQ[j] = maxLimits[j];
                }
                else
                {
                    nextQ[j] = minLimits[j];
                }
                Util::safePositionMove(j, nextQ[j], armCtrl, limArm);
            }
        }
    }

    for (int j = 0; j < 7; ++j)
    {
        kinExpTargetQ[j+3] = nextQ[j]; 
    }
    //std::cout << "nextQ " << nextQ.toString() << std::endl;
}

#elif POSITION_MOVE
void InvKinThread::invKinExploration(double time)
{
    double q;
    double min, max;
    double phaseT = 20; 

 
    for (int i = 0; i < NUM_ARM_JOINTS; i++)
    {
        limArm->getLimits(i, &min, &max);
        int offset = 0;
        if (i < 3)
        {
            offset = 10;
            minLimits[i] = std::max(minLimits[i], min+offset);
            maxLimits[i] = std::min(maxLimits[i], max-offset);
        }
    }

    Vector arm(NUM_JOINTS);
    arm.zero();
    // move arm
    for (int i = 0; i < NUM_ARM_JOINTS; i++)
    {
        double rad = fmod((time * M_PI / phaseTimes[i]), M_PI);  
        arm[i] = sin(rad) * (maxLimits[i] - minLimits[i]) + minLimits[i];
    }
    Util::safePositionMove(arm, armCtrl, limArm); 

    // move torso
    for (int i = 0; i < 3; i++)
    {
        double rad = fmod((time * M_PI / phaseTimesTorso[i]), M_PI);  
        q = sin(rad) * (maxLimitsTorso[i] - minLimitsTorso[i]) + minLimitsTorso[i];
        Util::safePositionMove(i, q, torsoCtrl, limTorso);
    }

    double e, r;
    module->getHandGazeAngles(e,r);

    int nAxesTorso;
    int nAxesArm;
    encTorso->getAxes(&nAxesTorso);
    encArm->getAxes(&nAxesArm);
    Vector torsoQ(nAxesTorso);
    Vector armQ(nAxesArm);
    encTorso->getEncoders(torsoQ.data());
    encArm->getEncoders(armQ.data());

    handGazeControl.addTrainingSample(e, r, handX, eyeX, torsoQ, armQ, handO);

    explSampleCount++;
    kinExpSphere.markGaze(e, r);
    if (time-lastInvKinPrintTime > 1.0)
    {
        lastInvKinPrintTime = time;
        std::cout << "Samples, time(min), samples/sec: \t" << explSampleCount << "\t" 
            << (int)time/60 << "\t" << (double)explSampleCount/time << std::endl;
    }
    kinExpSphere.show("Kinexp", 5);
}
#else
void InvKinThread::invKinExploration(double time)
{
    double q;
    double min, max;
    double phaseT = 20; 

    for (int i = 0; i < NUM_ARM_JOINTS; i++)
    {
        limArm->getLimits(i, &min, &max);
        int offset = 0;
        if (i < 3)
        {
            offset = 10;
            minLimits[i] = std::max(minLimits[i], min+offset);
            maxLimits[i] = std::min(maxLimits[i], max-offset);
        }
    }

    int nAxesTorso;
    int nAxesArm;
    CV_Assert(encTorso);
    CV_Assert(encArm);
    encTorso->getAxes(&nAxesTorso);
    encArm->getAxes(&nAxesArm);
    Vector torsoQ(nAxesTorso);
    Vector armQ(nAxesArm);
    encTorso->getEncoders(torsoQ.data());
    encArm->getEncoders(armQ.data());

    Vector arm(NUM_JOINTS);
    arm.zero();
    double maxArmSpeed = 8; 
    double maxTorsoSpeed = 8; 

    double speedUp = 1.0; 

    // move arm
    for (int i = 0; i < NUM_ARM_JOINTS; i++)
    {
        double rad = fmod((speedUp * time * M_PI / phaseTimes[i]), M_PI);  
        double armQ_des = sin(rad) * (maxLimits[i] - minLimits[i]) + minLimits[i];
        double err = armQ_des - armQ[i];
        double vel = std::max(-maxArmSpeed, std::min(maxArmSpeed, err * maxArmSpeed)); 
        velCtrl->velocityMove(i,vel);  
        //std::cout << i << ": " << armQ_des << " " << armQ[i] << " " << err << " " << vel << std::endl;  
    }
    //std::cout << std::endl;

    // move torso
    for (int i = 0; i < 3; i++)
    {
        double rad = fmod((speedUp * time * M_PI / phaseTimesTorso[i]), M_PI);  
        double torsoQ_des = sin(rad) * (maxLimitsTorso[i] - minLimitsTorso[i]) + minLimitsTorso[i];
        double err = torsoQ_des - torsoQ[i];
        double vel = std::max(-maxTorsoSpeed, std::min(maxTorsoSpeed, err * maxTorsoSpeed)); 
        torsoVelCtrl->velocityMove(i,vel);  
    }

    // get robot pose
    double e, r;
    Vector handX, handO;
    armCart->getPose(handX, handO);
    Vector eyeX = module->getEyePosition();
    HandPoseUtil::getGazeAngles(eyeX, handX, handO, e, r, !module->isRightArm());


    // save pose data 
    saveTrainingSample(filenameTrainingData, e, r, handX, eyeX, torsoQ, armQ, handO);
    explSampleCount++;

    // Visualize exploration on view sphere
    kinExpSphere.markGaze(e, r);
    if (time-lastInvKinPrintTime > 1.0)
    {
        lastInvKinPrintTime = time;
        std::cout << "Samples, time(min), samples/sec: \t" << explSampleCount << "\t" 
            << (int)time/60 << "\t" << (double)explSampleCount/time << std::endl;
    }
    kinExpSphere.show("Kinexp", 5);
}
#endif

/**
  * Set the velocity of the specified joint taking into account the min and max value specified for the encoder
  * and the max and min values specified for the velocities.
  */
//void InvKinThread::safeVelocityMove(int encIndex, double speed)
//{
	//if(encoders[encIndex] <= MOTOR_MIN[encIndex-11])
    //{
		//speed = (speed<0) ? 0 : speed;
	//}
	//else if(encoders[encIndex] >= MOTOR_MAX[encIndex-11])
    //{
		//speed = (speed>0) ? 0 : speed;
	//}

	//speed = (speed>VELOCITY_MAX) ? VELOCITY_MAX : speed;
	//speed = (speed<VELOCITY_MIN) ? VELOCITY_MIN : speed;

	//velCtrl->velocityMove(encIndex, speed);
//}

void InvKinThread::saveTrainingSample(
        const std::string &filename, double e, double r, 
        const Vector &handX, const Vector &eyeX, 
        const Vector &torsoQ, const Vector &armQ, 
        const Vector &handO)
{
    // create directory if necessary
    std::string handGazeDataDir = fs::path(filenameTrainingData).parent_path().string();
    if (! fs::exists(handGazeDataDir))
    try
    {
        fs::create_directories(handGazeDataDir);
    }
    catch(std::exception &e)
    {
        std::cerr << "ERROR: Could not create hand gaze control directory " << handGazeDataDir << std::endl;
        std::cerr << "\t ... " << e.what() << std::endl; 
        return;
    }

    // write to file
    std::ofstream file(filenameTrainingData.c_str(), std::ios::out | std::ios::app);
    if (!file.is_open())
    {
        std::cerr << "ERROR: Could not open file '" << filenameTrainingData << "' for writing." << std::endl;
        return;
    }

    file << e << " " << r << " ";
    file << Util::toString(handX) << " " << Util::toString(eyeX) << " ";
    file << Util::toString(torsoQ) << " " ;
    int numArmJoints = 7;  // without fingers
    for (int i = 0; i < numArmJoints; i++)
    {
        file << armQ[i] << " ";
    }
    file << Util::toString(handO);

    file << std::endl;

    file.close();
}

void InvKinThread::deleteTrainingData()
{
    std::ofstream file(filenameTrainingData.c_str(), std::ios::out);
    if (file.is_open())
        file.close();
}

