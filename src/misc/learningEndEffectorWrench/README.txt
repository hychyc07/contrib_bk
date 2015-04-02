README.txt
----------

learningEndEffectorWrench is "short" for "Applying learningMachine to learn the
the output of the WBTO endEffectorWrench:o in absence of external forces, movement,
or vibration. This is, what can pretty much be considered an error that we will subtract
from the values of the port, developing a further filtered version of the original
endEffectorWrench:o port called /learningEndEffectorWrench/fc:o (force corrected)".

The goal of learningEndEffectorWrench is to achieve a fine adjusment on the WBD
"/wholeBodyDynamics/.../endEffectorWrench:o" ports. The implementation is
moderately flexible (allowing to choose between arms and robot or simulator).
The provided machine and scaler correspond to the robot left arm.

In the tools dir:
 * dataRandom - Generates some random data that is formally fit for training.
 * dataPeriodic - Periodically retrieves data for saving in a .dat to train the learningMachine.
 * dataOnClick - Retrieves data for saving in a .dat to train the learningMachine when any keyboard key is hit, so user can determine when the robot has stabilized in between movements.
 * robotJoints - Like dataOnClick, but additionally moves the robot arm in joint space after each keyboard hit.
 * robotCartesian - Like robotJoints, but moves the arm in variable size small crosses and boxes instead of the joint space after each keyboard hit. Noise is automatically added to these variable size small crosses for better training samples. The data file is stored with the date on its file name.

