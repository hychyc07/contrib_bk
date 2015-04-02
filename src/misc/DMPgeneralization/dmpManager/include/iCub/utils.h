/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Ugo Pattacini
 * email:  vadim.tikhanoff@iit.it
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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>
#include <map>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/PortReport.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/PortInfo.h>
#include <yarp/sig/Vector.h>

#include <cv.h>
//#include "iCub/DMP/DMPstructure.h"
//#include "iCub/DMP/DMP.h"
class Manager;  //forward declaration

class ObjectPropertiesCollectorPort: public yarp::os::RpcClient, public yarp::os::PortReport
{
private:
    bool scheduleUpdate;
    std::map<std::string,int> memoryIds;
    bool checkConnection();

public:
    ObjectPropertiesCollectorPort()
        :scheduleUpdate(false)
    {
        this->setReporter(*this);
    }

    void report(const yarp::os::PortInfo &info)
    {
        if(info.created && !info.incoming)
            scheduleUpdate=true;
    }

    bool addActionTarget(int id, yarp::sig::Vector targetPosition);
    int  createActionTarget(std::string actionName, yarp::sig::Vector targetPosition);
    bool get2DPositionFromMemory(const std::string &object, yarp::sig::Vector &position);
  //  dmp::DMP* get_information_for(int32_t id);
//     bool getStereoPosition(const std::string &obj_name, yarp::sig::Vector &stereo);
//     bool getCartesianPosition(const std::string &obj_name, yarp::sig::Vector &x);
//     bool getKinematicOffsets(const std::string &obj_name, yarp::sig::Vector *kinematic_offset);
//     bool setKinematicOffset(const std::string &obj_name, const yarp::sig::Vector *kinematic_offset);
//     bool getTableHeight(double &table_height);
//     bool setTableHeight(const double &table_height);
//     int setAction(const std::string &act_name, const yarp:Bottle *traj_endeff, const Bottle *traj_joints=NULL);
//     bool getAction(const std::string &act_name, Bottle *traj_endeff, Bottle *traj_joints=NULL);

    bool isUpdateNeeded()
    {
        if(scheduleUpdate)
        {
            scheduleUpdate=false;
            return true;
        }

        return false;
    }
};

/**********************************************************/
class ParticleFilter : public yarp::os::BufferedPort<yarp::os::Bottle>
{
protected:
    CvPoint loc;
    void onRead(yarp::os::Bottle &b);
public:
    ParticleFilter();
    bool getTraker(CvPoint &loc);
};
/**********************************************************/
class SegmentationPoint : public yarp::os::Port
{
public:
    void segment(yarp::os::Bottle &b);
};
/**********************************************************/
class PointedLocation : public yarp::os::BufferedPort<yarp::os::Bottle>
{
protected:
    CvPoint loc;
    double  rxTime;
    double  timeout;

    void onRead(yarp::os::Bottle &b);

public:
    PointedLocation();
    bool getLoc(CvPoint &loc);
};

#endif
