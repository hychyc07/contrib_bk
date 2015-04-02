/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#ifndef __MODULE_H__
#define __MODULE_H__

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <cv.h>

#include "utils.h"
#include "classifierHandling.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


/**********************************************************/
class Manager : public RFModule
{
protected:
    RpcServer           rpcHuman;    
    RpcClient           rpcClassifier;
    RpcClient           rpcMotor;
    RpcClient           rpcGet3D;
    RpcClient           rpcMotorStop;
    RpcClient           rpcMemory;
    StopCmdPort         rxMotorStop;
    PointedLocationPort pointedLoc;
    MemoryReporter      memoryReporter;

    BufferedPort<Bottle>             blobExtractor;
    BufferedPort<ImageOf<PixelBgr> > imgIn;
    Port imgOut;
    Port imgRtLocOut;
    Port imgClassifier;

    Speaker speaker;
    Attention attention;
    RtLocalization rtLocalization;
    Exploration exploration;
    MemoryUpdater memoryUpdater;
    ClassifiersDataBase db;
    map<string,int> memoryIds;

    ImageOf<PixelBgr> img;
    ImageOf<PixelBgr> imgRtLoc;
    Semaphore mutexResources;
    Semaphore mutexResourcesMemory;
    Semaphore mutexAttention;
    Semaphore mutexMemoryUpdate;
    
    string name;
    string camera;
    string objectToBeKinCalibrated;
    bool scheduleLoadMemory;
    bool enableInterrupt;
    bool actionInterrupted;
    bool skipGazeHoming;
    bool trainOnFlipped;
    bool trainBurst;
    double improve_train_period;
    double classification_threshold;

    bool    trackStopGood;
    bool    whatGood;
    CvPoint trackStopLocation;
    CvPoint whatLocation;

    Bottle lastBlobs;
    Bottle memoryBlobs;
    Bottle memoryScores;

    Vector skim_blobs_x_bounds;
    Vector skim_blobs_y_bounds;

    friend class Attention;
    friend class RtLocalization;
    friend class Exploration;
    friend class MemoryUpdater;
    friend class MemoryReporter;

    int     processHumanCmd(const Bottle &cmd, Bottle &b);
    Bottle  skimBlobs(const Bottle &blobs);
    Bottle  getBlobs();
    CvPoint getBlobCOG(const Bottle &blobs, const int i);
    bool    get3DPosition(const CvPoint &point, Vector &x);
    void    acquireImage(const bool rtlocalization=false);
    void    drawBlobs(const Bottle &blobs, const int i, Bottle *scores=NULL);
    void    loadMemory();
    void    updateClassifierInMemory(Classifier *pClassifier);
    void    updateObjCartPosInMemory(const string &object, const Bottle &blobs, const int i);
    int     findClosestBlob(const Bottle &blobs, const CvPoint &loc);
    Bottle  classify(const Bottle &blobs, const bool rtlocalization=false);
    void    burst(const string &tag="");
    void    train(const string &object, const Bottle &blobs, const int i);
    void    improve_train(const string &object, const Bottle &blobs, const int i);
    void    home(const string &part="all");
    void    calibTable();
    bool    calibKinStart(const string &object, const string &hand, const int recogBlob);
    void    calibKinStop();
    void    _motor(const string &cmd, const string &object);
    void    _motor(const string &cmd, const Bottle &blobs, const int i);
    bool    interruptableAction(const string &action, deque<string> *param, const string &object);
    void    point(const string &object);
    void    look(const string &object);
    void    point(const Bottle &blobs, const int i);
    void    look(const Bottle &blobs, const int i);
    int     recognize(const string &object, Bottle &blobs, Classifier **ppClassifier=NULL);
    int     recognize(Bottle &blobs, Bottle &scores, string &object);
    void    execName(const string &object);
    void    execForget(const string &object);
    void    execWhere(const string &object, const Bottle &blobs, const int recogBlob, Classifier *pClassifier);
    void    execWhat(const Bottle &blobs, const int pointedBlob, const Bottle &scores, const string &object);
    void    execExplore(const string &object);
    void    execInterruptableAction(const string &action, const string &object, const int recogBlob);
    void    switchAttention();
    void    doLocalization();
    bool    get3DPositionFromMemory(const string &object, Vector &position);
    void    doExploration(const string &object, const Vector &position);
    void    updateMemory();

public:
    void    interruptMotor();
    void    reinstateMotor(const bool saySorry=true);
    bool    configure(ResourceFinder &rf);
    bool    interruptModule();
    bool    close();
    bool    updateModule();
    double  getPeriod();    
};

#endif

