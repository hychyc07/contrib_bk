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
#ifndef DATAASSOCIATOR_H
#define DATAASSOCIATOR_H



// comment this if you do not need debugging output for data association (DA) 
//#define DEBUG_DA  


#include <math.h>
#include <vector>
#include <iostream>

#include "cv.h"
#include "targetobject.h"
#include "dataelement3d.h"
#include "validationmatrix.h"
///#include "datamutex.h"

#include <map>

#include "definitions.h"


// gnu scientific library
#include <gsl/gsl_math.h>
#include <gsl/gsl_rng.h> // for random numbers
#include <gsl/gsl_randist.h>

#include <qstringlist.h>

/// timer for setting dynamic transition matrix
#include "timer.h"

#include "targetcreationdeletion.h"

// Get all OS and signal processing YARP classes

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <yarp/os/BufferedPort.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;



using namespace std;

/**
	@author Zenon Mathews <zmathews@iua.upf.edu>
*/
class DataAssociator{
public:
 	
	
	/// for random number generation using gsl
	const gsl_rng_type * tGSL;
        gsl_rng * rGSL;
	

	/// struct for use in MCMC algorithm
	
	struct MCMC_Event
    	{
      		int hitCount;  // counts the nof times this event was generated using MCMC
      		float postProb; // the posterior prob, of this event
      		vector<int>* dataToTarget;  // the data to target associations..e.g. 4,3,1,2 means data 1 associated to target 4, data 2 associated to target 3 etc.
    	};
	
	
	struct FLOOR_WEIGHT
    	{
      		double x,y,weight;
		
    	};


	/// map containing all JPDA events used for MCMC computation: this map will be filled in each run by the function:  computeJPDAeventProbs
	map<int, MCMC_Event> all_Events;// the int is just the map key
	

	/// empties all events from the map
	void emptyAllEvents();

	/// prints an int vector
	void printVector(vector<int> dataToTarget);

	
	/// generates an initial feasible event using the validation matrix
	/// this is done simply by looking at the validation matrix and selecting the first feasible event possible
	MCMC_Event generateInitialFeasibleEvent();
	
	/// manages a newly generated event in MCMC
	/// if the event is really new, add it to the all_Events map
	/// else increase the hitCount for the appropriate event in the map
	void manageNewMCMCEvent(vector<int> eventVector);

	/// MCMC: returns true if accept new MC state tmpEventVectorTwo
	bool proposalGood(vector<int> tmpEventVectorOne, vector<int> tmpEventVectorTwo );


	/// for finding random feasible row and column of the validation matrix
	int randomRow;
	int randomCol;
	//return true if there is atleast one feasible elem in validation matrix
	// and sets randomRow and randomCol
	bool findRandomFeasibleElem();
	vector<int>* rowFeasVec;
	vector<int>* colFeasVec;


	/// temp matrices for the proposal function
	CvMat* tempp1; 
	CvMat* tempp2; 
	CvMat* tempp3; 
	CvMat* innovCovv;
	CvMat* curPredd;

	/// --------- matrices for computing the validation matrix
	CvMat*	data_high_dim_pos;
			
	CvMat*	innov;
		
	CvMat*	innovTmp;
		
	CvMat*	innovTmp2;
		
	CvMat*	temp1;
	CvMat*	temp2;
	CvMat*	temp3;

	CvMat*	invInnovCov;
		
	CvMat*	innovTranspose;
		
	CvMat*	innovCov;

	///-------------------------------------------

	


    DataAssociator(vector<TargetObject*>* targVec, TargetCreationDeletion * tcd);

   /** vector containing data from ALL sensors */
    vector<DataElement3D*>* allDataVector;
	
	/// the following 2 are needed to re-intitialize the validationMatrix 
	// current no.of data
	int nofData;
	// current no. of targets 
	int nofTargets;

	

	// validation matrix (for each time step it says which data is valid for which targets)
	ValidationMatrix* validationMatrix;

	/// run the whole JPDA procedure for all targets
	void runJPDA();

	/// do tha kalman prediction for all targets
	void predictAllTargetStates();

	/// do tha kalman prediction for single target
	void predictSingleTargetState(int index);
	
	/// updates the validation matrix.....threshold is the threshold for the validation gate 
	void updateValidationMatrix();

	

	/// compute JPDA event probs 
	void computeJPDAeventProbs();

	/// new: 17/11/2008
	void estimateAllTargetStatesValidationGate();

	/// to be implemented ....compute the JPDA state estimation for a given target
	void estimateAllTargetStatesJPDA();



    ~DataAssociator();
	
   // contains all the targets being tracked
    std::vector<TargetObject*>* targetVector;
 

	/// no delay nearest neighbour state estimator
	void noDelayNearestNeighEstimator();
	/// a boolean vector to make sure no two targets have the same NN data
	vector<bool> dataFreeVector;


	/// dynamic kalman state transition matrix
	CvMat* transition_mat;
	
	CvMat* control_mat;
	/// set the dynamic kalman state transition matrix
	void setTransitionMatrix();
	Timer transition_mat_timer;

	void printMatrix(CvMat* mat,  QString name);

	void avoidCoalescence();
	
	/// returns true if the input camera-data is not associated to any other target
	/// this is to avoid that one camera data is associated to more than one target
	/// because it is impossible with AnTS in XIM
	bool getAssociationOk(DataElement3D* curData, TargetObject* targetInstance);

	/// computes the min Mahalanobis distance to other targets
	void computeMinMahaDistance(TargetObject* target);

	float getMahanalobisDist(TargetObject* targOne, TargetObject* targTwo);
	CvMat* stateMatrixOne; 
	CvMat* stateMatrixTwo; 
	CvMat* stateOneCovariance;
	CvMat* resultMahaDistMatrix;
	void setAttentionPriority(TargetObject *target);
	void sendAttentionalInspection(); // sends the targets interesting for attentional inspection
	QString attentionInspecString;

	map<float, int> attPriorityMap; // sorted map with the priority as key and the index of the target as the attribute

	bool getTargetXYZ(int idd, float &curX, float &curY, float &curZ); // help function

	


	// adds Gaussian distributed data to the given data
	QString getGaussianDistributedData(QString curData);


	/// new validation gate computation 12/11/2008: returns true if inside validation gate
	bool insideValidationGate(TargetObject* targ, DataElement3D data, float scaleOne, float scaleTwo, float minAxis, float maxAxis, float predictionFactor);



	/// for Gaussian spreading of data
	const gsl_rng_type * type;
  	gsl_rng * r;
	
	void updateFinalData(TargetObject target, DataElement3D* finalData, DataElement3D currentData);
	
	/// returns true if the given data is not too close to all other targets except the given one
	bool clearNeighbourHood(TargetObject* target, DataElement3D* curData);

	double getFloorDataWeight(DataElement3D* curData);
	double getCamDataWeight(DataElement3D* curData);

	TargetCreationDeletion * targCreationDeletion;
	double totalFloorWeight;	

	void avoidCoalescenceWeight();
	void avoidCoalescenceWeightNew();

	float getDistance(DataElement3D* data, TargetObject* target);
	float getDistance(TargetObject* target1, TargetObject* target2);

	float findAngle(TargetObject* t1, TargetObject* t2, DataElement3D data);

	
	//BufferedPort<Vector> inPortLeft;
	//BufferedPort<Bottle> inPortLeft;

};

#endif
