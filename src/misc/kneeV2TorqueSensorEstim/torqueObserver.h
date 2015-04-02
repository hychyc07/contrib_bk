/* 
 * Copyright (C)2013  iCub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
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

#ifndef __TORQUEOBSERVER_H__
#define __TORQUEOBSERVER_H__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

#include <iostream>
#include <iomanip>
#include <string.h>


/**
* A thread reading FT sensor measurements and computing joint torques for a certain limb.
**/
class SensToTorques: public yarp::os::RateThread
{
private:

    yarp::os::Stamp             timestamp;
    yarp::dev::PolyDriver       *dd;
    yarp::dev::IEncoders        *iEncs;
    yarp::dev::IPositionControl *iPos;
    yarp::dev::IPidControl      *iPid;

    yarp::os::BufferedPort<yarp::sig::Vector> *port_FT;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_TAU;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_LOG;

    /// encoders values for the limb
    yarp::sig::Vector encoders;
    yarp::sig::Vector FTsensor;
    yarp::sig::Vector Tau;
    yarp::sig::Vector FTsensor_offset;
    double            SPsensor_offset;

    /**
    * Calibrate the sensor by finding the sensor offset.
    * @param Ntrials the number of samples to use for the offset computation
    */
    void calibrateOffset(const unsigned int Ntrials);

    void readAndUpdate(bool waitMeasure = false);

public:

    //SensToTorques(int _rate, yarp::dev::PolyDriver *_dd, yarp::os::BufferedPort<yarp::sig::Vector> *port_FT, yarp::os::BufferedPort<yarp::sig::Vector> *port_Torques_limb, yarp::os::BufferedPort<yarp::sig::Vector> *port_FTendef, std::string _type);
    
    SensToTorques(int _rate, yarp::dev::PolyDriver *_dd);
      
    bool threadInit();
      
    void run();
      
    void threadRelease();


      
};


class TorqueObserver: public yarp::os::RFModule
{
private:

    yarp::os::Property OptionsLimb;
    yarp::dev::PolyDriver *dd;

    SensToTorques *sens2torques;

    std::string handlerPortName;
    yarp::os::Port handlerPort;      //a port to handle messages 

    /*
    * Check driver consistency
    * @return true if driver is working properly and encoders are correctlty viewed
    */
    bool checkDriver(yarp::dev::PolyDriver *pd_limb);
    
public:

    /*
    * Constructor
    */
    TorqueObserver();
    /*
    * Configure the module parameters
    * @return true if successful
    */
    bool configure(yarp::os::ResourceFinder &rf); 
    /*
    * Interrupt, e.g. the ports
    * @return true if successful
    */
   bool interruptModule();
   /*
   * Close and shut down the module
   * @return true if successful
   */
   bool close();     
   /*
   * Respond if necessary
   * @return true if successful, false otherwise (e.g. bad command)
   */
   bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
   /*
   * @return period
   */
   double getPeriod(); 
   /*
   * Update module
   * @return true if successful
   */
   bool updateModule();



};



#endif //__TORQUEOBSERVER_H__
