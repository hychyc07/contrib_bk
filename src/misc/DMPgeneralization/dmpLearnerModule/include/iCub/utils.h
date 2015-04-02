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

#ifndef __DMP_LEARNER_UTILS_H__
#define __DMP_LEARNER_UTILS_H__

#include <stdint.h>
#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/PortReport.h>
#include <yarp/os/PortInfo.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

//#include <iCub/DMP/DMPstructure.h>
#include <iCub/DMP/DMP.h>

class DmpLearner;  //forward declaration

class ObjectPropertiesCollectorPort: public yarp::os::RpcClient, public yarp::os::PortReport
{
private:
    bool scheduleUpdate;
    DmpLearner *learnerModule;
    bool checkConnection();
    

public:
    ObjectPropertiesCollectorPort()
        :scheduleUpdate(false)
    {
        this->setReporter(*this);
        learnerModule=NULL;
    }
    
    void setLearner(DmpLearner* newLearner)
    {
        learnerModule=newLearner;
    }
    

    void report(const yarp::os::PortInfo &info)
    {
        if(info.created && !info.incoming)
            scheduleUpdate=true;
    }

    bool addActionDMP(int32_t id, dmp::DMP* dmpToAdd);
    std::vector<int32_t> get_all_actions_with_traj_endeff();
    bool get_information_for(int32_t id);

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



#endif
