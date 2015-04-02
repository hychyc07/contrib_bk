/***************************************************************************
 *   Copyright (C) 2007 by Zenon Mathews   *
 *   zmathews@iua.upf.edu   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef TARGETOBJECT_H
#define TARGETOBJECT_H

#include <qstring.h>
#include <vector>
#include <iostream>
#include <string>
#include <iostream>
#include "cv.h"
#include "timer.h"
#include "math.h"
#include "definitions.h"
#include <unistd.h>
#include <sstream>
#include "dataelement3d.h"
/**
	@author Zenon Mathews <zenon.mathews@gmail.com>
*/
class TargetObject{
public:

	struct high_dim_data
    	{
      		float x;
		float y;
	        float z;
		float hue;
		float weight;
    	};


	float tapx, tapy; // send the predicted position to tap

	// this is the x and y that will be send out to position clients
	float sendx, sendy, sendz;

    	TargetObject(int idd, std::string namee, float xx, float yy,  float zz, float vxx, float vyy, float vzz, int red, int green, int blue, int inactTime);

    	~TargetObject();

	// stores this event for the target
	void storeJPDAevent(float xx, float yy, float zz,  float huee, float weightt, float prob);

	// updates the state of the target using the combined innovation and the evtn probs stored earlier using storeJPDAevent
	void updateJPDAstate(CvMat* trans_mat);

	void updateJPDAstateNew(CvMat* trans_mat);

	// do the Kalman prediction
	void kalmanPredictState();
	
	// do the Kalman estimation..given the measurement (x,y, hue, weight) 
	void kalmanEstimateState(float x, float y, float z, float huee, float weightt);
	
     	// position and velocity of the target	
     	float x, y, z, hue, weight, vx, vy, vz;

	

	// previous positions (needed for avoiding track coalescence)
	float xOld, yOld, zOld, vxOld, vyOld, vzOld, hueOld, weightOld; 

	/// time for which the target has not been updated..used to delete the
	/// target
	Timer timer;

	// rgb and the hue of the target
	// rgb is generated randomly and used display the target
	// on the visualization...rgb is NOT used as a target attribute
	// hue is used as a target attribute!	
     	int red, green, blue; 

      	// name of the target
   	std::string name;	
   	
	// id if target
	int id;
	// special ID
	QString special_id;
	void computeSpecialID(); // computes the special ID
	
	// pointer to the Kalman structure storing state and matrices of this target
	CvKalman* kalman; 
	
	// pointer to a matrix for measurements..the sensor data will be read in here
	CvMat* measurement;

	// predicted measurement
	CvMat* measurement_pred;

	// JPDA: combined innovation
	CvMat* combined_innov;

	// temporary
	CvMat* tempo1, *tempo2, *tempoTrans1, *tempCov1, *tempCov2, *tempCov3, *tempCov4,*tempCov5, *tempCov6,*combInnovCov;
	
	/// NEW: error to be added to the cov. matrix
	CvMat* extra_error;

	// individual innovations......
	std::vector<CvMat*>* all_innovs;

	// the event probabilities
	std::vector<float>* event_probs;

	// resst all JPDA fields used
	void resetJPDA();
	
	void printMatrix(CvMat* mat,  QString name);

	// pritns the trace norm
	void printNormTraceMatrix(CvMat* mat,  QString name);

	

	// just to try kalman without JPDA
	void updateKalmanState(CvMat* trans_mat, float x, float y, float z);

	// used int new JPDA update function
	std::vector<high_dim_data>* all_measurements;

	void updateKalmanStateWithProb(float xxxx, float yyyy, float zzzz, float prob);

	bool hueDataAvailable; // this will be set to true when hue data was associated to this target
	float currentHueValue;
	float currentHuePosX, currentHuePosY;

        void HSV2RGB(int hueVal, int &red, int &green, int &blue);

	float minMahaDist; // minimum Mahanalobis distance to otehr targets
	float attentionalPriority;	

	// for the validation gate
	double xaxis, yaxis, growth_factor, predX, predY, predZ;
	bool notAssociated, lostTrack;

	void updateRGB();

	// timer for testing
	//Timer tempTimer;
	//bool newHue;

	bool estimationAdvanced; // a help variable for resetting covariance

	//for weight computation
	double prevWeight;
	double curWeight;
	bool weightReset;
	void resetWeight();
	void addWeight(double w);
	void updateTotalWeight();
	
	std::vector<DataElement3D> weightDataVector;
	void addToWeightData(DataElement3D data);
	void clearWeightData();

	void setTapY(double y);
};


#endif
