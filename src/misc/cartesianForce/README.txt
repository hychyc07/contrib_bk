README.txt
----------
Note: may not be updated, best check the code!

The ultimate goal is to develop a library that enables hybrid force/velocity control with accurate contour tracking. Before achieving this, we must perform some previous steps.

* cartesianForce -> Here we just do basic force control. The law to implement is basically:

cmdTorques = G + JT_r * Kp * (fd - f_r) - Kd * dq;
    where:
cmdTorques (nx1): The final commanded joint leve torques
G (nx1): Gravity comp in joint space
jac_T (nx6): The jacobian trasposed, specific for EE config.
Kp (scalar): A proportional gain, for a scalar.
Fd (nx1): The desired EE force/torque.
Fread (nx1): The current read EE force/torque.
Kd (scalar): a damping effect, a scalar depending on joint velocity
dq (nx1): The current read joint velocities (estimate based on AW using enc vals).

To be taken into account:
1) Init position (we would like hand on top of table)
2) Direction of force (we would like parallel to table)
3) What values should be taken as constants for the control law? (Kp, Kd...)(Kd also depends if in radians or degrees).
4) Where do we get the gravity from? Remember that gravityCompensator interferes with force control.

For now we are going to specify the CLI.
* quit/start/stop -> still valid.
* set/get Kp(double)/Kd(double)/Fd(double(6)) -> 3/3/8


---------------------------------------------------------------------

A description of the tools directory.

 * icubParams -> simple program to get iDyn params on screen.
 * testApp -> In its first implementation, similar to cartesian force. Its home is now a flat position, it is made to go down (Fd 0 0 -1 0 0 0). The error is measured and outputted in all directions. Shows how just Jt does not seem good enough, and how bad force sensor offset is when away from std icub home.
 * testApp2 -> Similar to testApp.
 * testApp3 -> In this version, going down movement is no longer in init. It is called by pressing "p" or "push". You set the moving vecocity through "set Vd ... ... ...".
 * testApp4 -> Same as testApp3, but using WBC lib to compute mass matrix.


