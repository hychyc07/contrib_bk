// yarp includes
#include <yarp/os/Time.h>

// local includes
#include <iCub/tactileLearning/robotInterface.h>
#include <iCub/tactileLearning/constants.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::tactileLearning;

const string RobotInterface::RPC_CLIENT_PORT = "/robotInterface";
const string RobotInterface::RPC_SERVER_PORT = "/skinManager/rpc";
const double RobotInterface::POSITION_GAIN = 0.001;
const double RobotInterface::POSITION_GAIN_INST = 0.01;

RobotInterface::RobotInterface(PolyDriver* rd, IPositionControl* p, IVelocityControl* v, IEncoders* e, BufferedPort<Vector>* tdp,
							   std::vector<bool>& af, std::vector<bool>& aj) :
				robotDevice(rd),
				pos(p),
				vel(v),
				encs(e),
				tactileDataPort(tdp),
				activeFingers(af),
				activeJoints(aj),
				jointsNumber(0),
				activeFingersNumber(0),
				activeJointsNumber(0),
				iteration(0)

{
	Network yarp;

	// acquire the number of joints
	if(!pos->getAxes(&jointsNumber)) printf("ERROR: impossible to get the number of joints\n");

	// retrieve the number of active fingers
	for(unsigned int i = 0; i < activeFingers.size(); i++)
	{
		if(activeFingers[i] == true) activeFingersNumber++;
	}

	// retrieve the number of active joints
	for(unsigned int i = 0; i < activeJoints.size(); i++)
	{
		if(activeJoints[i] == true) activeJointsNumber++;
	}

	// go to home position (only for simulation, the real robot should be positioned manually)
	Vector home = Vector(jointsNumber, HOME_POSITION_VECTOR);
	positionMoveHand(home);

	// resize vectors so that they are coherent with the number of joints
	homePosition.resize(jointsNumber);
	
	// save the current joint position as an home position for each trial
	encs->getEncoders(homePosition.data());

	// debug
	printf("HOME POSITION:\t");
	for(int p = 0; p < jointsNumber; p++) printf("%f ", homePosition(p));

	// rpc connection for calibration
	rpcSkin.open(RPC_CLIENT_PORT.c_str());
	yarp.connect(RPC_CLIENT_PORT.c_str(), RPC_SERVER_PORT.c_str());
}

RobotInterface::~RobotInterface()
{
}

void RobotInterface::robotRelease()
{
	//return to home position
	positionMoveHand(homePosition);

	// close the poly driver
	if(robotDevice) robotDevice->close();
}

bool RobotInterface::performTrajectory(dmp::Trajectory& inputTrajectory, double dt, bool velocityControl)
{
	// specify the dimensions of the state matrices depending on both the time steps and the ACTIVE fingers/joints
	pressureState.resize(inputTrajectory.getLength(), activeFingersNumber);
	positionState.resize(inputTrajectory.getLength(), activeJointsNumber);
	velocityState.resize(inputTrajectory.getLength(), activeJointsNumber);

	// specify the dimensions of the reference matrix depending on both the time steps and the WHOLE joints number
	refTrajectory.resize(inputTrajectory.getLength(), jointsNumber);
	refPosTrajectory.resize(inputTrajectory.getLength(), jointsNumber);
	
	// specify the dimensions of the actual matrices depending on both the time steps and the WHOLE joints number
	positionTrajectory.resize(inputTrajectory.getLength(), jointsNumber);
	velocityTrajectory.resize(inputTrajectory.getLength(), jointsNumber);

	/*
	firstly let the whole arm return to home position. Since a trajectory only involves hand joints, the
	other joints will trivially remain always in their home positions
	*/
	if(!positionMoveHand(homePosition))
	{
		printf("ERROR: impossible to return to home position before starting a new trajectory\n");
		return false;
	}

	// ask for calibration via rpc
	//askForCalibration();
	
	if(!controlHand(inputTrajectory, dt, velocityControl))
	{
		printf("ERROR: impossible to perform input trajectory and/or acquiring sensors data\n");
		return false;
	}

	writeTrajectoriesToFiles();

	iteration++;

	return true;
}

bool RobotInterface::positionMoveHand(Vector& target)
{
	Vector tmp;
	bool done = false;

	// set reference velocity for the motion
	tmp.resize(jointsNumber);
	for(int i = 0; i < jointsNumber; i++) tmp[i] = REFERENCE_SPEED;
	pos->setRefSpeeds(tmp.data());

	// start the movement and check if it is finished
	if(!pos->positionMove(target.data())) return false;
	while(!done)
	{
		pos->checkMotionDone(&done);
		// two seconds to ensure the reaching
		Time::delay(2);
	}

	pos->stop();
	return true;
}

bool RobotInterface::positionMoveArm(Vector& armTarget, std::vector<Vector>& pressure, double dt)
{
	Vector targetPosition, tmp;
	bool done = false;

	// check for consistency
	if(armTarget.length() != jointsNumber - NUM_HAND_JOINTS)
	{
		printf("ERROR: vector dimension mismatch. Impossible to construct target position vector\n");
		return false;
	}

	// build the target position vector
	targetPosition.resize(jointsNumber);
	encs->getEncoders(targetPosition.data());
	targetPosition.setSubvector(0, armTarget);

	// set reference velocity for the motion
	tmp.resize(jointsNumber);
	for(int i = 0; i < jointsNumber; i++) tmp[i] = REFERENCE_SPEED;
	pos->setRefSpeeds(tmp.data());

	// debug
	for(unsigned int i = 0; i < targetPosition.size(); i++) printf("%f\n", targetPosition[i]);

	// start the movement and check if it is finished
	// in the meanwhile, store the perceived pressure
	if(!pos->positionMove(targetPosition.data())) return false;
	while(!done)
	{
		getCurrentPressure(tmp);
		pressure.push_back(tmp);
		Time::delay(dt);
		pos->checkMotionDone(&done);
	}

	//pos->stop();
	return true;
}

bool RobotInterface::controlHand(dmp::Trajectory& inputTrajectory, double dt, bool velocityControl)
{
	// set startTime to zero just for the first iteration
	double startTime;
	double globalStartTime;

	// used for holding encoders data and velocity commands
	Vector encodersPos, encodersSpeed, velCommand, refAcc;
	Vector posError;
	posError.resize(jointsNumber);
	encodersPos.resize(jointsNumber);
	encodersSpeed.resize(jointsNumber);
	refAcc.resize(jointsNumber);
	velCommand.resize(jointsNumber);

	// build the whole reference trajectory matrix in order to have pre-computed velocity and acceleration commands,
	// avoiding to compute them at each time step.
	buildRefTrajectoryMatrix(inputTrajectory, velocityControl, dt);

	// set reference acceleration in case of velocity control
    if(velocityControl)
	{
		for(int i = 0; i < jointsNumber; i++) refAcc[i] = 10000;
		vel->setRefAccelerations(refAcc.data());
	}

	double tmp = 0;
	Time::turboBoost();

	// initialize the velocity command for those joints which are not involved in the control.
	// this should avoid the unexpected movements caused by the position compensation.
	int jointIndex = 0;
	while(jointIndex < jointsNumber)
	{
		velCommand[jointIndex] = 0.0;
		jointIndex++;
	}

	globalStartTime = Time::now();
	startTime = Time::now();

	// iterate through the time steps of the trajectory
	for(int i = 0; i < inputTrajectory.getLength(); i++)
	{
		startTime = Time::now();

		// read from position and velocity encoders
		encs->getEncoders(encodersPos.data());
		encs->getEncoderSpeeds(encodersSpeed.data());

		// compute the current velocity command, ONLY for the active joints, taking into account both feedforward 
		// velocity and position error.
		// Compromise: a unique velocity command is used -> less time spent in giving the commands (8-10 ms!);
		// the distinction between active and not active joints allows to avoid the unexpected movements
		tmp = Time::now();
		for(int j = 0; j < jointsNumber; j++)
		{
			posError[j] += ((refPosTrajectory.getRow(i))[j] - encodersPos[j]);
			if(activeJoints[j] == true) velCommand[j] = (refTrajectory.getRow(i))[j] + POSITION_GAIN * posError[j] + POSITION_GAIN_INST*((refPosTrajectory.getRow(i))[j] - encodersPos[j]);
		}

		if(velocityControl)
		{
			if(!vel->velocityMove(velCommand.data())) printf("\nWARNING: failed to give the command!\n");
		}
		else pos->positionMove(refTrajectory.getRow(i).data());

		//printf("%i-th: c = %f\t", i, Time::now()-tmp);

		for(int j = 0; j < jointsNumber; j++)
		{
			if(activeJoints[j] == true)
			{
				printf("trackingError = %f\t", (refPosTrajectory.getRow(i))[j] - encodersPos[j]);
				//printf("rP = %f\t", (refPosTrajectory.getRow(i))[j]);
				//printf("eP = %f\t", encodersPos[j]);
				//printf("vC = %f\t", (refTrajectory.getRow(i))[j]);
				//printf("pC = %f\t", POSITION_GAIN * posError[j]);
				//printf("%f\t", velCommand[j]);
				printf("\n");
			}
		}
		printf("\n");

		// in the meanwhile, perform the remaining computation. The reading of encoders and control signal
		// computation had to be done before, obviously.

		// i-1 because the element refers to the outcome of the previous velocity command
		if(!acquirePressureState(i-1)) return false;

		// store trajectory values
		positionTrajectory.setRow(i, encodersPos);
		velocityTrajectory.setRow(i, encodersSpeed);

		// printf("time elapsed: %f\t", Time::now() - startTime);

		Time::delay(dt - (Time::now() - startTime));
	}

	// final state
	if(!acquirePressureState(inputTrajectory.getLength() - 1)) return false;

	if(velocityControl) vel->stop();
	else pos->stop();
	printf("TRAJECTORY DURATION:%f\n", Time::now() - globalStartTime);

	// construct the position state matrix (only active joints)
	if(!acquireTrajectoryState()) return false;

	//printf("TRAJECTORY DURATION:%f\n", Time::now() - globalStartTime);

	/*
	printf("STATE PRESSURE TRAJECTORY:\t");
	for(unsigned int p = 0; p < presState.getCol(0).size(); p++) printf("%f ", presState.getCol(0)[p]);
	*/

	return true;
}

bool RobotInterface::moveHandUntilTouch(Vector& startPosition, Vector& endPosition)
{
	// used for holding encoders data and velocity commands
	Vector encodersPos;
	Vector velocityCommand;

	encodersPos.resize(jointsNumber);
	velocityCommand.resize(jointsNumber);

	startPosition.resize(activeJointsNumber);
	endPosition.resize(activeJointsNumber);

	int posIndex;
	
	// save the current joint position as start position
	posIndex = 0;
	for(unsigned int i = 0; i < activeJoints.size(); i++)
	{
		if(activeJoints[i])
		{
			encs->getEncoder(i, &startPosition[posIndex]);
			posIndex++;
		}
	}

	// check for consistency
	if(posIndex != activeJointsNumber)
	{
		printf("ERROR: dimensions mismatch happened during initial position vector construction\n");
		return false;
	}

	// debug
	for(unsigned int i = 0; i < startPosition.size(); i++) printf("\n %f", startPosition[i]);

	int jointIndex = 0;

	while(jointIndex < (jointsNumber - HAND_JOINTS_OFFSET))
	{
		velocityCommand[jointIndex] = 0.0;
		jointIndex++;
	}
	
	for(int j = jointIndex; j < jointsNumber; j++)
	{
		if(activeJoints[j] == true) velocityCommand[j] = SLOW_REFERENCE_SPEED;
	}
	
	//------------------------------------------------------------------------------------------------------------
	// this peace of code works on the simulator, but not on the real robot: the active fingers don't move! MISTERY
	// maybe it is caused by the timeout for the velocity commands
	//------------------------------------------------------------------------------------------------------------

	//if(!vel->velocityMove(velocityCommand.data())) return false;

	//// check for touches while moving
	//while(!checkIfTouched()) Time::delay(0.02);

	//Time::delay(1.0);
	//vel->stop();

	//------------------------------------------------------------------------------------------------------------

	//------------------------------------------------------------------------------------------------------------
	// this should work on the real robot
	//------------------------------------------------------------------------------------------------------------

	while(!checkIfTouched(velocityCommand))
	{
		if(!vel->velocityMove(velocityCommand.data())) return false;
		Time::delay(0.01);
	}

	Time::delay(1.0);
	vel->stop();

	//------------------------------------------------------------------------------------------------------------

	// save the current joint position as end position
	posIndex = 0;
	for(unsigned int i = 0; i < activeJoints.size(); i++)
	{
		if(activeJoints[i])
		{
			encs->getEncoder(i, &endPosition[posIndex]);
			posIndex++;
		}
	}

	positionMoveHand(homePosition);

	// debug
	for(unsigned int i = 0; i < endPosition.size(); i++) printf("\n %f", endPosition[i]);

	return true;
}

void RobotInterface::buildRefTrajectoryMatrix(dmp::Trajectory& trajectory, bool velocityControl, double dt)
{
	Vector command, refPosition;
	int jointIndex;
	
	command.resize(jointsNumber);
	refPosition.resize(jointsNumber);

	// if a velocity control is being applied, then set to zero those indexes which refer to arm joints
	if(velocityControl)
	{
		jointIndex = 0;
		while(jointIndex < (jointsNumber - HAND_JOINTS_OFFSET))
		{
			command[jointIndex] = 0.0;
			refPosition[jointIndex] = homePosition(jointIndex);
			jointIndex++;
		}
	}

	// otherwise, in case of a position control, set to home values those indexes which refer to arm joints
	else
	{
		jointIndex = 0;
		while(jointIndex < (jointsNumber - HAND_JOINTS_OFFSET))
		{
			command[jointIndex] = homePosition(jointIndex);
			jointIndex++;
		}
	}

	// iterate through the time steps of the trajectory
	for(int i = 0; i < trajectory.getLength(); i++)
	{
		// fill reference vectors simply by parsing the trajectory data matrix for the current time step values.
		int dimensionIndex = 0;
		for(int j = jointIndex; j < jointsNumber; j++)
		{
			if(activeJoints[j] == true)
			{
				command[j] = trajectory.getTrajectoryPosition(i, dimensionIndex);
				if(i > 0) refPosition[j] = refPosTrajectory(i - 1, j) + dt * refTrajectory(i - 1, j);
				else refPosition[j] = homePosition(j);
				dimensionIndex++;
			}
			else
			{
				if(velocityControl)
				{
					command[j] = 0.0;
					refPosition[j] = homePosition(j);
				}
				else command[j] = homePosition(j);
			}
		}

		// add vectors to the whole matrix
		refTrajectory.setRow(i, command);
		refPosTrajectory.setRow(i, refPosition);
	}
	
	//print ref speed (debug)
	//printf("\n\nREFERENCE SPEED\n\n");
	//for(int i = 0; i < refTrajectory.rows(); i++)
	//{
	//	printf("%f\t", refPosTrajectory(i,11));
	//	printf("%f\t", refTrajectory(i,11));
	//	printf("%f\t", refPosTrajectory(i,12));
	//	printf("%f\t", refTrajectory(i,12));
	//	printf("\n\n");
	//}
}

bool RobotInterface::acquirePressureState(int step)
{
	// trick for managing the mismatch between the end of a velocity command and the state pressure acquisition
	if(step >= 0)
	{
		Vector currentState = zeros(pressureState.cols());
		if(currentState.size() != activeFingersNumber)
		{
			printf("ERROR: dimension mismatch between state columns and number of fingers");
			return false;
		}

		// read the tactile sensors		
		Vector compensatedData = *(tactileDataPort->read(false));
		
		// find the max touch for each finger, taking into account the order:
		// index, middle, ring, little, thumb.
		// Actually the order in which each state pressure dimension is written into the
		// state matrix is not important.
		int stateDimIndex = 0;

		// ad hoc management for thumb
		if(activeFingers[0])
		{
			for(int i = 48; i < 60; i++)
			{
				if(compensatedData[i] > currentState[stateDimIndex]) currentState[stateDimIndex] = compensatedData[i];
			}
			stateDimIndex++;
		}

		// the others fingers
		int sensorIndex = 0;
		for(unsigned int j = 1; j < activeFingers.size(); j++)
		{
			if(activeFingers[j])
			{
				for(int i = 0; i < 12; i++)
				{
					if(compensatedData[sensorIndex] > currentState[stateDimIndex]) currentState[stateDimIndex] = compensatedData[sensorIndex];
					sensorIndex++;
				}
				stateDimIndex++;
			}
			else sensorIndex += 12;
		}

		printf("PRESSURE STATE:\t");
		for(int p = 0; p < activeFingersNumber; p++) printf("%f ", currentState[p]);
		printf("\n");

		pressureState.setRow(step, currentState);
	}
	 
	return true;
}

bool RobotInterface::checkIfTouched(Vector& velocityCommand)
{
	Vector currentState = zeros(activeFingers.size());
	int remainingTouches = activeFingersNumber;
	
	// read the tactile sensors
	Vector compensatedData = *(tactileDataPort->read());
		
	// find the max touch for each finger, taking into account the order:
	// index, middle, ring, little, thumb.
	// The order metters, because for each finger which has touched the actuating joints have to be stopped

	// ad hoc management for thumb
	if(activeFingers[0])
	{
		for(int i = 48; i < 60; i++)
		{
			if(compensatedData[i] > TOUCH_THRESHOLD)
			{
				printf("\nthumb touched\n");
				
				velocityCommand[HAND_JOINTS_OFFSET] = 0.0;
				velocityCommand[HAND_JOINTS_OFFSET + 1] = 0.0;
				velocityCommand[HAND_JOINTS_OFFSET + 2] = 0.0;

				//this is just useful at the first call of the function. However it is kept for safety.
				vel->stop(HAND_JOINTS_OFFSET);
				vel->stop(HAND_JOINTS_OFFSET + 1);
				vel->stop(HAND_JOINTS_OFFSET + 2);
				remainingTouches--;
				break;
			}
		}
	}

	// the others fingers but pinky
	int sensorIndex = 0;
	for(unsigned int j = 1; j < activeFingers.size() - 2; j++)
	{
		if(activeFingers[j])
		{
			for(int i = 0; i < 12; i++)
			{
				if(compensatedData[sensorIndex] > TOUCH_THRESHOLD)
				{
					printf("\ntouched %i\n", j);
					
					velocityCommand[((j-1)*2) + HAND_JOINTS_OFFSET + 3] = 0.0;
					vel->stop(((j-1)*2) + HAND_JOINTS_OFFSET + 3);
					//printf("\nstopped joint %i\n", ((j-1)*2) + HAND_JOINTS_OFFSET + 3);
					
					velocityCommand[((j-1)*2 + 1) + HAND_JOINTS_OFFSET + 3] = 0.0;
					vel->stop(((j-1)*2 + 1) + HAND_JOINTS_OFFSET + 3);
					//printf("\nstopped joint %i\n", ((j-1)*2 + 1) + HAND_JOINTS_OFFSET + 3);
					
					remainingTouches--;
					sensorIndex += 12 - i;
					break;
				}
				sensorIndex++;
			}
		}
		else sensorIndex += 12;
	}

	// ring + pinky
	if(activeFingers[3])
	{
		while(sensorIndex < 48)
		{
			if(compensatedData[sensorIndex] > TOUCH_THRESHOLD)
			{
				printf("\ntouched ring\n");
				velocityCommand[15] = 0.0;
				vel->stop(15);

				// double, because with only a motor for two fingers it is impossible to discriminate
				remainingTouches--;
				remainingTouches--;
				break;
			}
			sensorIndex++;
		}
	}

	if(remainingTouches == 0) return true;
	 
	return false;
}

bool RobotInterface::acquireTrajectoryState()
{
	int activeJointIndex = 0;
	Vector tmpPosDimension = zeros(positionState.rows());
	Vector tmpVelDimension = zeros(velocityState.rows());

	for(int i = 0; i < positionTrajectory.cols(); i++)
	{
		if(activeJoints[i])
		{
			tmpPosDimension = positionTrajectory.getCol(i);
			tmpVelDimension = velocityTrajectory.getCol(i);
			positionState.setCol(activeJointIndex, tmpPosDimension);
			velocityState.setCol(activeJointIndex, tmpVelDimension);
			activeJointIndex++;
		}
	}

	// consistency check
	if(activeJointIndex != activeJointsNumber)
	{
		printf("ERROR: dimension mismatch occurred during position state matrix construction\n");
		return false;
	}

	return true;
}

int RobotInterface::getActiveFingers()
{
	return activeFingersNumber;
}

int RobotInterface::getJointsNumber()
{
	return jointsNumber;
}

bool RobotInterface::writeTrajectoriesToFiles()
{
	std::stringstream sstm1, sstm2, sstm3;
	FILE *fp1, *fp2, *fp3;
	string str;
	
	// print position
	sstm1 << "data/positionTrajectory" << iteration << ".txt";
	str = string(sstm1.str());
	
	if ((fp1 = fopen(str.c_str(), "w")) == NULL)
    {
        printf("ERROR: Cannot open file %s\n", str.c_str());
        return false;
    }

	for (int i = 0; i < positionState.rows(); i++)
    {
		for (int n = 0; n < positionState.cols(); n++)
        {
			fprintf(fp1, "%f\t", positionState(i, n));
        }

		fprintf(fp1, "\n");
    }

	fclose(fp1);
	
	// print velocity
	sstm2 << "data/velocityTrajectory" << iteration << ".txt";
	str = string(sstm2.str());

	if ((fp2 = fopen(str.c_str(), "w")) == NULL)
    {
        printf("ERROR: Cannot open file %s\n", str.c_str());
        return false;
    }

	for (int i = 0; i < velocityState.rows(); i++)
    {
		for (int n = 0; n < velocityState.cols(); n++)
        {
			fprintf(fp2, "%f\t", velocityState(i, n));
        }

		fprintf(fp2, "\n");
    }

	fclose(fp2);

	// print pressure
	sstm3 << "data/pressureState" << iteration << ".txt";
	str = string(sstm3.str());

	if ((fp3 = fopen(str.c_str(), "w")) == NULL)
    {
        printf("ERROR: Cannot open file %s\n", str.c_str());
        return false;
    }

	for (int i = 0; i < pressureState.rows(); i++)
    {
		for (int n = 0; n < pressureState.cols(); n++)
        {
			fprintf(fp3, "%f\t", pressureState(i, n));
        }

		fprintf(fp3, "\n");
    }

	fclose(fp3);

	return true;
}

bool RobotInterface::getHomePosition(Vector& home)
{
	if(homePosition.size() == 0) return false;

	home.resize(homePosition.size());
	for(unsigned int i = 0; i < home.size(); i++) home(i) = homePosition(i);

	return true;
}

void RobotInterface::getPressureState(yarp::sig::Matrix& outputPressure)
{
	outputPressure.resize(pressureState.rows(), pressureState.cols());
	for(int i = 0; i < outputPressure.rows(); i++)
	{
		for(int d = 0; d < outputPressure.cols(); d++) outputPressure(i, d) = pressureState(i, d);
	}
}

void RobotInterface::getPositionState(yarp::sig::Matrix& outputPosition)
{
	outputPosition.resize(positionState.rows(), positionState.cols());
	for(int i = 0; i < outputPosition.rows(); i++)
	{
		for(int d = 0; d < outputPosition.cols(); d++) outputPosition(i, d) = positionState(i, d);
	}
}

void RobotInterface::getVelocityState(yarp::sig::Matrix& outputVelocity)
{
	outputVelocity.resize(velocityState.rows(), velocityState.cols());
	for(int i = 0; i < outputVelocity.rows(); i++)
	{
		for(int d = 0; d < outputVelocity.cols(); d++) outputVelocity(i, d) = velocityState(i, d);
	}
}

void RobotInterface::getCurrentPressure(Vector& pressure)
{
	pressure.resize(activeFingersNumber);

	// read tactile sensors		
	Vector compensatedData = *(tactileDataPort->read(false));
		
	// find the max touch for each finger, taking into account the order:
	// index, middle, ring, little, thumb.
	// Actually the order in which each state pressure dimension is written into the
	// state matrix is not important.
	int stateDimIndex = 0;

	// ad hoc management for thumb
	if(activeFingers[0])
	{
		for(int i = 48; i < 60; i++)
		{
			if(compensatedData[i] > pressure[stateDimIndex]) pressure[stateDimIndex] = compensatedData[i];
		}
		stateDimIndex++;
	}

	// the others fingers
	int sensorIndex = 0;
	for(unsigned int j = 1; j < activeFingers.size(); j++)
	{
		if(activeFingers[j])
		{
			for(int i = 0; i < 12; i++)
			{
				if(compensatedData[sensorIndex] > pressure[stateDimIndex]) pressure[stateDimIndex] = compensatedData[sensorIndex];
				sensorIndex++;
			}
			stateDimIndex++;
		}
		else sensorIndex += 12;
	}

	printf("PRESSURE STATE:\t");
	for(int p = 0; p < activeFingersNumber; p++) printf("%f ", pressure[p]);
	printf("\n");
}

void RobotInterface::askForCalibration()
{
	Bottle command, response;
	bool calibrationDone = false;
	string answer;
	
	// send a command for calibration
	command.addString("calib");
	printf("Sending message... %s\n", command.toString().c_str());
    rpcSkin.write(command, response);
	printf("Got response: %s\n", response.toString().c_str());

	// check if calibration is done
	while(!calibrationDone)
	{
		Time::delay(1);
		command.clear();
		command.addString("is");
		command.addString("calibrating");
		rpcSkin.write(command, response);
		answer = response.toString();
		if(answer.compare("no") == 0) calibrationDone = true;
	}

	printf("calibration done\n");
	return;
}