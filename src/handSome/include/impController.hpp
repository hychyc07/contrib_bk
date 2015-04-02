//Poisition/Impedance controller. 
//The class inherits from the generic controller.

//Author: 	Cristiano Alessandro
//email:	alessandro@ifi.uzh.ch

#ifndef IMPCONTROLLER_HPP_
#define IMPCONTROLLER_HPP_

#include <yarp/sig/Vector.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include "controller.hpp"

#define DAMP	0.01	/*	N/w		*/
#define STIFF	0.16	/*	N/deg	*/

using namespace yarp;
using namespace yarp::dev;

namespace HandSome {
	class PositionImpedanceThd: public ControllerThd {

	 protected:
		PolyDriver *client; //Ploydriver of the device to control
		IImpedanceControl *imp; //Impedance control interface
		IPositionControl *pos; //Position control interface
		IControlMode *mode; //Interface to change the control mode
		Vector compJ; //Compliant joints
		bool flag;	//Check motion done


	 private:
		double refAcc; //Reference acceleration
		double refSpeed; //Reference speed

	 public:
		PositionImpedanceThd(PolyDriver*, double, double, Vector&, double);

		//Called after thread->start()
		virtual bool threadInit();

		//Core of the thread
		virtual void run();

		//Called after thread->stop()
		virtual void threadRelease();

		//Vector has the same size of the number of variables(joints) to control.
		//If the j-th element is equal to 1, the j-th variabe will be controlled in
		//impedance; if it contains 0, the j-th variable will be controlled only in posiiton.
		bool enableImpedance(Vector&);

		//Accepting the number of variables/joints to control, it will set the references
		//speeds and accelerations at each joint.
		void setReferences(int);

		/*
		 * Check if previous movement is done
		 *
		 * Ask the controller if converged to commanded position
		 *
		 * @param flag Flag where the status is written to
		 */
		bool checkMotionDone(bool* flag);
	};

}
#endif /* IMPCONTROLLER_HPP_ */
