#ifndef __TACTILE_CONSTANTS_H__
#define __TACTILE_CONSTANTS_H__
 
#include <stdio.h>

/// number of fingers
const int NUM_FINGERS = 4;

/// number of hand joints
const int NUM_HAND_JOINTS = 8;

/// movement time duration
const double MOVEMENT_DURATION = 8.0;
const double HOLD = 3.0;

/// specifies the offset between the indexes of the joints for the whole arm and those of the hand
const int HAND_JOINTS_OFFSET = 8;

/// value for the desired pressure
const double DESIRED_PRESSURE = 15.0;

/// minimum time duration (in number of steps) for a contact
const int MIN_CONTACT_DURATION = 10;

const int CONTACT_DURATION = 50;

/// the names for the fingers
const std::string FINGER_TYPES[] = {"thumb", "index", "middle", "pinky"};

/// reference speed for the finger joints (used in position control)
const double REFERENCE_SPEED = 10.0;

/// reference speed for moving the fingers during the "touch detection" phase
const double SLOW_REFERENCE_SPEED = 3.0;

/// home position vector
//const double HOME_POSITION_VECTOR[16] = {-25, 20, 16, 60, 70, 0, 0, 0, 20, 20, 20, 10, 10, 10, 10, 10};
//const double HOME_POSITION_VECTOR[16] = {-30, 0, 9, 45, 90, -22, 0, 0, 20, 20, 20, 10, 10, 10, 10, 10};
//const double HOME_POSITION_VECTOR[16] = {-30, 15, 9, 45, 80, -22, 0, 0, 20, 20, 20, 10, 10, 10, 10, 10};
//const double HOME_POSITION_VECTOR[16] = {-21, 16, 6, 60, -6, -6, 11, 2, 85, 20, 20, 20, 20, 20, 20, 20};
const double HOME_POSITION_VECTOR[16] = {-47, 20, 10, 40, -11, 0, 11, 0, 90, 10, 10, 10, 10, 15, 15, 0};

/// position positions for the arm joints (evaluation pose)
const double FINAL_POSITION_VECTOR[8] = {3, 77, -7, 56, -70, 0, 20, 9};

/// touch threshold used for touch detection
const double TOUCH_THRESHOLD = 5;

#endif