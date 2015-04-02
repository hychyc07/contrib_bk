/*
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Elena Ceseracciu
 * email:  vadim.tikhanoff@iit.it elena.ceseracciu@iit.it
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

#ifndef __DMPMANAGER_MODULE_H__
#define __DMPMANAGER_MODULE_H__

#include <string>

#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RFModule.h>
//#include <yarp/os/RpcServer.h>
//#include <yarp/os/RpcClient.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>


#include "iCub/utils.h"
#include "iCub/dmpLearnerInterface.h"
#include "iCub/DMP/DmpGPR.h"

/**********************************************************/
class DmpLearner : public yarp::os::RFModule, iCub::dmpLearnerInterface
{
protected:
    yarp::os::Port              thriftPort;
    std::string                 name;               //name of the module
    ObjectPropertiesCollectorPort opcPort;
    
    std::map<std::string, DmpGPR*> learnersMap;
    
    double alpha_x;
    double alpha_z;
    double beta_z;
    int N;
    int dof;
    dmp::Trajectory* currentTrajectory;
    //yarp::sig::Matrix currentTrajectory;
    dmp::DMP* currentDMP; //using pointer because there is no default constructor...
    yarp::sig::Vector currentTarget;
    std::string actionName;
    std::string dmpType;
    yarp::os::Semaphore currentMutex; //most likely, it's useless...
    void resetCurrent();
    void resetMap();
    
    
    virtual bool sync_opc();
    virtual bool estimate_DMP(const int32_t id);
    virtual bool train_ids(const std::vector<int32_t> & trainInputIds);
    virtual bool train_action(const std::string& actionName);
    virtual bool generalize_DMP(const int32_t id);
    virtual void set_num_basis_functions(const int32_t N);
    virtual void set_alphax(const double alphax);
    virtual void set_alphaz(const double alphaz);
    virtual void set_betaz(const double betaz);
    virtual void quit();
    bool attach(yarp::os::Port &source);

public:
    DmpLearner();
    ~DmpLearner();
    bool    configure(yarp::os::ResourceFinder &rf);
    bool    interruptModule();
    bool    close();
    bool    updateModule();
    double  getPeriod();
    
    friend class ObjectPropertiesCollectorPort;
};
#endif

