/*
 * LoomingDetector.cpp
 *
 *  Created on: Jun 5, 2012
 *      Author: Christian Dondrup
 */

#include <iostream>

#include "iCub/LoomingDetector.h"

using namespace std;
using namespace yarp::os;

void LoomingDetector::receivedSkeletonAndObject(LDData* data) {
	mutex.wait();
	receivedSkeleton(data, true);
	mutex.post();
}

void LoomingDetector::receivedObject(LDData* data) {
	mutex.wait();
//	writer->write("Object");
	mutex.post();
}

void LoomingDetector::receivedSkeleton(LDData* data, bool object) {
	mutex.wait();
	/* Creating the data bottles for both hands containing the time stamp
	 and xyz coordinates. */
	Bottle right, left;
	right.addDouble(data->getSkeletonData()->getTimestamp());
	right.addDouble(data->getSkeletonData()->getRightHandX());
	right.addDouble(data->getSkeletonData()->getRightHandY());
	right.addDouble(data->getSkeletonData()->getRightHandZNormed());
	if (object) {
		right.addVocab(VOCAB3('O','B','J'));
		right.addString(data->getObjectData()->getName().c_str());
		right.addString(data->getObjectData()->getColor().c_str());
		right.addInt(data->getObjectData()->getColorId());
	}
	left.addDouble(data->getSkeletonData()->getTimestamp());
	left.addDouble(data->getSkeletonData()->getLeftHandX());
	left.addDouble(data->getSkeletonData()->getLeftHandY());
	left.addDouble(data->getSkeletonData()->getLeftHandZNormed());
	if (object) {
		left.addVocab(VOCAB3('O','B','J'));
		left.addString(data->getObjectData()->getName().c_str());
		left.addString(data->getObjectData()->getColor().c_str());
		left.addInt(data->getObjectData()->getColorId());
	}
	/* Calling the detect function or the detectEmulator function. */
	if (emulator) {
		if (data->getSkeletonData()->isRightHand())
			detectEmulator(data->getSkeletonData()->getRightHandZNormed(), true,
					right, emulator);
		if (data->getSkeletonData()->isLeftHand())
			detectEmulator(data->getSkeletonData()->getLeftHandZNormed(), false,
					left, emulator);
	} else {
		if (data->getSkeletonData()->isRightHand())
			detect(data->getSkeletonData()->getRightHandZNormed(), true, right);
		if (data->getSkeletonData()->isLeftHand())
			detect(data->getSkeletonData()->getLeftHandZNormed(), false, left);
	}
	mutex.post();
}

void LoomingDetector::receivedAnyThing() {
	mutex.wait();
//	writer->write("Anything");
	mutex.post();
}

bool LoomingDetector::threadInit() {
	stoploop = false;
	return true;
}
void LoomingDetector::threadRelease() {
	cout << "  LoomingDetector stopped" << endl;
}
void LoomingDetector::run() {
	cout << " LoomingDetector" << endl;
	while (!stoploop)
		;
}

bool LoomingDetector::detect(double dist_hand, bool right, Bottle data) {
	/* Looming detection for the right hand. */
	if (right) {
		//Calculating distance sums and mean
		dist_sum_right += dist_hand;
		dist_squared_sum_right += pow(dist_hand, 2);
		mean_right = dist_sum_right / ++nr;

		/* Calculation for online variance taken from: http://www.mathepedia.de/Standardabweichung.aspx*/
		// Calculating the variance
		var_right = sqrt(
				(nr * dist_squared_sum_right - pow(dist_sum_right, 2))
						/ (nr * (nr - 1)));

//		cout << "Right: " << mean_right << " - Var: " << var_right
//				<< " - Distance: " << dist_hand - mean_right << endl;
		writer->writeData(data, mean_right, var_right, dist_hand - mean_right,
				true);

		// Detect looming if the hand moves closer to the camera (has to be twice the variance)
		// and the current distance has to be lower then the mean.
		if (sqrt(pow(dist_hand - mean_right, 2)) > var_right * 2
				and dist_hand - mean_right < 0) {
			/* Replace cout by writing to port */
//			cout << time << " Looming: " << var_right << " - "
//					<< sqrt(pow(dist_hand - mean_right, 2)) << " Mean: "
//					<< mean_right << " Hand: " << dist_hand << " Dist: "
//					<< dist_hand - mean_right << " Sum: " << dist_sum_right
//					<< " N: " << nr << endl;
			if (!loo_r)
				writer->setLoomingStart(data, true);
			else
				writer->sendLooming(data, true);
			loo_r = true;

			n_looming_r++;
		} else {
			if (loo_r)
				writer->setLoomingStop(data, true);
			loo_r = false;
			n_looming_r = 0;
		}

		/* Reset the looming detection if there is continuous looming for habituationThreshold detections
		 * and if the hand moves more then 10cm away from the camera.
		 */
		if (n_looming_r > habituationThreshold
				or dist_hand - mean_right > 0.1) {
			dist_sum_right = 0;
			dist_squared_sum_right = 0;
			nr = 0;
			n_looming_r = 0;
			if (loo_r)
				writer->setLoomingStop(data, true);
			loo_r = false;
			writer->sendResetFlag(data, true);
		}
	} else /* Looming detection for the left hand. */{
		//Calculating distance sums and mean
		dist_sum_left += dist_hand;
		dist_squared_sum_left += pow(dist_hand, 2);
		mean_left = dist_sum_left / ++nl;

		/* Calculation for online variance taken from: http://www.mathepedia.de/Standardabweichung.aspx*/
		// Calculating the variance
		var_left = sqrt(
				(nl * dist_squared_sum_left - pow(dist_sum_left, 2))
						/ (nl * (nl - 1)));

//		cout << "Left: " << mean_left << " - Var: " << var_left
//				<< " - Distance: " << dist_hand - mean_left << endl;
		writer->writeData(data, mean_left, var_left, dist_hand - mean_left,
				false);

		// Detect looming if the hand moves closer to the camera (has to be twice the variance)
		// and the current distance has to be lower then the mean.
		if (sqrt(pow(dist_hand - mean_left, 2)) > var_left * 2
				and dist_hand - mean_left < 0) {
			/* Replace cout by writing to port */
//			cout << time << " Looming: " << var_left << " - "
//					<< sqrt(pow(dist_hand - mean_left, 2)) << " Mean: "
//					<< mean_left << " Hand: " << dist_hand << " Dist: "
//					<< dist_hand - mean_left << " Sum: " << dist_sum_left
//					<< " N: " << nl << endl;
			if (!loo_l)
				writer->setLoomingStart(data, false);
			else
				writer->sendLooming(data, false);
			loo_l = true;

			n_looming_l++;
		} else {
			if (loo_l)
				writer->setLoomingStop(data, false);
			loo_l = false;
			n_looming_l = 0;
		}

		/* Reset the looming detection if there is continuous looming for habituationThreshold detections
		 * and if the hand moves more then 10cm away from the camera.
		 */
		if (n_looming_l > habituationThreshold or dist_hand - mean_left > 0.1) {
			dist_sum_left = 0;
			dist_squared_sum_left = 0;
			nl = 0;
			n_looming_l = 0;
			if (loo_l)
				writer->setLoomingStop(data, false);
			loo_l = false;
			writer->sendResetFlag(data, false);
		}
	}
	return true;
}

bool LoomingDetector::detectEmulator(double dist_hand, bool right, Bottle data,
		double emulator) {
	if (dist_hand < emulator) {
		if (!loo_r) {
			writer->setLoomingStart(data, right);
		} else {
			writer->sendLooming(data, right);
		}
		loo_r = true;
	} else {
		if (loo_r) {
			writer->setLoomingStop(data, right);
		}
		loo_r = false;
	}

	return true;
}
