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
#include "dataassociator.h"



DataAssociator::DataAssociator(vector<TargetObject*>* targVec,TargetCreationDeletion * tcd )
{

	
	totalFloorWeight = 0.0;

	// used to set the total weight
	targCreationDeletion = tcd;

	

	/// for the computation of Mahanalobis distance
	attentionInspecString = "";
	
	stateMatrixOne = cvCreateMat(KALMAN_SIZE,1,CV_32FC1);	
	stateMatrixTwo = cvCreateMat(1,KALMAN_SIZE, CV_32FC1);	
	stateOneCovariance = cvCreateMat(KALMAN_SIZE,KALMAN_SIZE,CV_32FC1);	
	resultMahaDistMatrix = cvCreateMat(1,1,CV_32FC1);	

	 
	targetVector = targVec;
	allDataVector = new vector<DataElement3D*>;
	//
	
	/// ---- inilialize the validation matrix -----
	nofData = 3;
	nofTargets = 2;
	validationMatrix = new ValidationMatrix(3,2);
	
	/// --------------------------------------------


	/// for validation matrix calculation
	 	data_high_dim_pos = cvCreateMat(MES_SIZE,1,CV_32FC1);
			
		innov = cvCreateMat(MES_SIZE,1,CV_32FC1);
		
		innovTmp = cvCreateMat(1,MES_SIZE,CV_32FC1);
		
		innovTmp2 = cvCreateMat(MES_SIZE,1,CV_32FC1);
		
		temp1 = cvCreateMat(MES_SIZE,KALMAN_SIZE,CV_32FC1);		
		temp2 = cvCreateMat(KALMAN_SIZE,MES_SIZE,CV_32FC1);		
		temp3 = cvCreateMat(MES_SIZE,MES_SIZE,CV_32FC1);		

		invInnovCov = cvCreateMat(MES_SIZE,MES_SIZE,CV_32FC1);
		
		innovTranspose = cvCreateMat(1,MES_SIZE,CV_32FC1);
		
		innovCov = cvCreateMat(MES_SIZE,MES_SIZE,CV_32FC1);
	/// --------------------------------------------------


	// for random number generation using gsl
	gsl_rng_env_setup();
	tGSL = gsl_rng_default;
        rGSL = gsl_rng_alloc (tGSL);
	// now ths will give a random number between 0.0 and 1.0 : double u = gsl_rng_uniform (r);

	// for computing random feasible element of validation matrix
	rowFeasVec = new vector<int>;
	colFeasVec = new vector<int>;

	// for the proposal acceptance function proposalGood
	tempp1 = cvCreateMat(MES_SIZE,KALMAN_SIZE,CV_32FC1);		
	tempp2 = cvCreateMat(KALMAN_SIZE,MES_SIZE,CV_32FC1);		
	tempp3 = cvCreateMat(MES_SIZE,MES_SIZE,CV_32FC1);		
	innovCovv = cvCreateMat(MES_SIZE,MES_SIZE,CV_32FC1);
	curPredd = cvCreateMat(1,KALMAN_SIZE,CV_32FC1);	


	/// dynamic kalman transition matrix
	transition_mat = cvCreateMat(KALMAN_SIZE,KALMAN_SIZE,CV_32FC1);		
	control_mat = cvCreateMat(KALMAN_SIZE,MES_SIZE,CV_32FC1);	

	
}


DataAssociator::~DataAssociator()
{
	

	delete allDataVector;

	// for random number generation
	gsl_rng_free (rGSL);

	delete rowFeasVec;
	delete colFeasVec;

	/// release Mahanalobis distance measurement matrices
		cvReleaseMat(&stateMatrixOne);
		cvReleaseMat(&stateMatrixTwo);
		cvReleaseMat(&stateOneCovariance);

	/// release memory calclating the validation matrix			
		cvReleaseMat(&data_high_dim_pos);
		cvReleaseMat(&innov);
		cvReleaseMat(&innovTmp);
		cvReleaseMat(&innovTmp2);
		cvReleaseMat(&invInnovCov);
		cvReleaseMat(&innovTranspose);

		cvReleaseMat(&temp1);
		cvReleaseMat(&temp2);
		cvReleaseMat(&temp3);
	/// 
	
	/// release memory for matrices in the proposalGood function
	cvReleaseMat(&tempp1); 	
	cvReleaseMat(&tempp2);
	cvReleaseMat(&tempp3);
	cvReleaseMat(&innovCovv);
	cvReleaseMat(&curPredd);
	///------------------------


	/// dynamic kalman state transition matrix
	cvReleaseMat(&transition_mat);
	cvReleaseMat(&control_mat);
}

 

void DataAssociator::runJPDA(){
	///cout << endl << endl << "***************************** new dataasso runJPDA() ***************"	 << endl;

	/// ------------ use this for JPDA with MCMC --------------
	//cout << "goign to predict all target states" << endl;
        predictAllTargetStates();
		

	/// step 1) : predict the target states
	vector<TargetObject*>::iterator targIter; 
	
	/*
	for( targIter = targetVector->begin(); targIter != targetVector->end(); targIter++){ 
		(*targIter)->kalmanPredictState();
	}
	*/

	

	/// commented 0ut 17/11/2008
	///computeJPDAeventProbs();


	/// before updating the state of the targets, we have to set the transition matrix with the time delta_t
	setTransitionMatrix();

	
	/// /// commented 0ut 17/11/2008: JPDA estiamtion of states 
	///estimateAllTargetStatesJPDA();	/// the JPDA estimator	

	/// 2/04/2009: use the weight data to separate targets too close to each other
	//cout << "call coalesc" << endl;
	//avoidCoalescenceWeight();
	avoidCoalescenceWeightNew();

	/// 17/11/2008: new method: update using ALL elements inside the validation gate always
	estimateAllTargetStatesValidationGate();
	
	//cout << "end coalesc" << endl;

	/// remove all elements in the attention priority map
	attPriorityMap.clear();

	/// now reset the JPDA states
	for( targIter = targetVector->begin(); targIter != targetVector->end(); targIter++){ 
		(*targIter)->resetJPDA();
		(*targIter)->clearWeightData();

		/// compute Mahalanobis distance to other targets to
		/// set the confidence
		computeMinMahaDistance(*targIter);

		setAttentionPriority(*targIter);
	}

	/// now send the targets for attentional inspection
	//sendAttentionalInspection();

}
float DataAssociator::getDistance(DataElement3D* data, TargetObject* target){
	float tx, ty;
	float dx,dy;
	float dist;

	tx = target->x;
	ty = target->y;
	dx = data->x;
	dy = data->y;
	
	dist = sqrt((tx-dx)*(tx-dx)+(ty-dy)*(ty-dy));

	return dist;
}

float DataAssociator::getDistance(TargetObject *target1, TargetObject *target2){
	float tx, ty;
	float dx,dy;
	float dist;

	tx = target1->x;
	ty = target1->y;
	dx = target2->x;
	dy = target2->y;
	
	dist = sqrt((tx-dx)*(tx-dx)+(ty-dy)*(ty-dy));

	return dist;
}



void DataAssociator::avoidCoalescenceWeightNew(){
	// iterate throught the targets and find 
	vector<TargetObject*>::iterator targIter1; 
	vector<TargetObject*>::iterator targIter2; 
	vector<DataElement3D>::iterator weightIter;

	//cout << "coalesc 1" << endl;
	for( targIter1 = targetVector->begin(); targIter1 != targetVector->end(); targIter1++){ 
		for( targIter2 = targetVector->begin(); targIter2 != targetVector->end(); targIter2++){ 
			//cout << "coalesc 2" << endl;
			if (targIter1 != targIter2){ // if not same target
				float dist = getDistance(*targIter1, *targIter2);
				//cout << "coalesc 3" << endl;
				if ((dist < 60.0) && (dist > 35.0)){ // if too close
					/// add floor data on the correct side
				  float x1, y1, x2,y2, z1, z2;
					x1 = (*targIter1)->x - ((*targIter2)->x - (*targIter1)->x)/5.0;
					y1 = (*targIter1)->y - ((*targIter2)->y - (*targIter1)->y)/5.0;
					z1 = (*targIter1)->z - ((*targIter2)->z - (*targIter1)->z)/5.0;
					
					
					DataElement3D* elem1 = new DataElement3D(3,x1,y1,z1, -1,-1);
					DataElement3D* elem2 = new DataElement3D(3,x1+rand()%3, y1+rand()%3, z1+rand()%3, -1,-1);
					DataElement3D* elem3 = new DataElement3D(3,x1+rand()%3, y1+rand()%3, z1+rand()%3, -1,-1);
					
					allDataVector->push_back(elem1);
					allDataVector->push_back(elem2);
					allDataVector->push_back(elem3);
				}
				else if ((dist < 35.0) && (dist > 10.0)){
					/// add floor data on the correct side
				  float x1, y1,z1,x2,y2,z2,x3,y3,z3;
					x1 = (*targIter1)->x - ((*targIter2)->x - (*targIter1)->x)/2.0;
					y1 = (*targIter1)->y - ((*targIter2)->y - (*targIter1)->y)/2.0;
					z1 = (*targIter1)->z - ((*targIter2)->z - (*targIter1)->z)/2.0;
					x2 = (*targIter1)->x - ((*targIter2)->x - (*targIter1)->x)/1.0;
					y2 = (*targIter1)->y - ((*targIter2)->y - (*targIter1)->y)/1.0;
					z2 = (*targIter1)->z - ((*targIter2)->z - (*targIter1)->z)/2.0;
					x3 = (*targIter1)->x - 2*((*targIter2)->x - (*targIter1)->x)/1.0;
					y3 = (*targIter1)->y - 2*((*targIter2)->y - (*targIter1)->y)/1.0;
					z3 = (*targIter1)->z - 2*((*targIter2)->z - (*targIter1)->z)/1.0;
					

					DataElement3D* elem1 = new DataElement3D(3,x1,y1,z1, -1,-1);
					DataElement3D* elem2 = new DataElement3D(3,x2,y2,z2, -1,-1);
					DataElement3D* elem3 = new DataElement3D(3,x3,y3,z3, -1,-1);
					
					
					allDataVector->push_back(elem1);
					allDataVector->push_back(elem2);
					allDataVector->push_back(elem3);
				}
				
				else if (dist < 10.0){
					/// add floor data on the correct side
				  float x1, y1, z1, x2,y2, z2, x3,y3, z3;
					x1 = (*targIter1)->x - 2*((*targIter2)->x - (*targIter1)->x)/1.0;
					y1 = (*targIter1)->y - 2*((*targIter2)->y - (*targIter1)->y)/1.0;
					z1 = (*targIter1)->z - 2*((*targIter2)->z - (*targIter1)->z)/1.0;
					
					x2 = (*targIter1)->x - 3*((*targIter2)->x - (*targIter1)->x)/1.0;
					y2 = (*targIter1)->y - 3*((*targIter2)->y - (*targIter1)->y)/1.0;
					z2 = (*targIter1)->z - 3*((*targIter2)->z - (*targIter1)->z)/1.0;
					

					x3 = (*targIter1)->x - 4*((*targIter2)->x - (*targIter1)->x)/1.0;
					y3 = (*targIter1)->y - 4*((*targIter2)->y - (*targIter1)->y)/1.0;
					z3 = (*targIter1)->z - 4*((*targIter2)->z - (*targIter1)->z)/1.0;
					


					DataElement3D* elem1 = new DataElement3D(3,x1,y1,z1, -1,-1);
					DataElement3D* elem2 = new DataElement3D(3,x2,y2,z2, -1,-1);
					DataElement3D* elem3 = new DataElement3D(3,x3,y3,z3, -1,-1);
					
					
					allDataVector->push_back(elem1);
					allDataVector->push_back(elem2);
					allDataVector->push_back(elem3);
				}


			}
		}
	}



}






void DataAssociator::avoidCoalescenceWeight(){
	
	// iterate throught the targets and find 
	vector<TargetObject*>::iterator targIter1; 
	vector<TargetObject*>::iterator targIter2; 
	vector<DataElement3D>::iterator weightIter;

	//cout << "coalesc 1" << endl;
	for( targIter1 = targetVector->begin(); targIter1 != targetVector->end(); targIter1++){ 
		for( targIter2 = targetVector->begin(); targIter2 != targetVector->end(); targIter2++){ 
			//cout << "coalesc 2" << endl;
			if (targIter1 != targIter2){ // if not same target
				float dist = getDistance(*targIter1, *targIter2);
				//cout << "coalesc 3" << endl;
				if (dist < 100.0){ // if too close
					float bestAngle = 0;
					int bestIndex = -1;
					int i = -1;
					// find weight data preferable on the opposite side for targ1
					for (weightIter = ((*targIter1)->weightDataVector).begin(); weightIter!= ((*targIter1)->weightDataVector).end(); weightIter++){
						//cout << "coalesc 4" << endl;
						i++;
						float curAngle = findAngle(*targIter1, *targIter2, *weightIter);
						//cout << "coalesc 5" << endl;
						if (curAngle > bestAngle){ // the higher the better
							bestAngle = curAngle;
							bestIndex = i;
						}
					}

					if (bestIndex > -1){
						//
						//cout << "coalesc 6" << endl;
						float x1 = ((*targIter1)->weightDataVector).at(bestIndex).x;
						//cout << "coalesc 7" << endl;
						float y1 = ((*targIter1)->weightDataVector).at(bestIndex).y;
						float z1 = ((*targIter1)->weightDataVector).at(bestIndex).z;
						

						float x2,y2,z2, tmpx, tmpy, tmpz;
						tmpx = ((*targIter1)->weightDataVector).at(bestIndex).x - (*targIter1)->x;
						tmpy = ((*targIter1)->weightDataVector).at(bestIndex).y - (*targIter1)->y;
						tmpz = ((*targIter1)->weightDataVector).at(bestIndex).z - (*targIter1)->z;
						
						x2 = (*targIter1)->x + tmpx;
						y2 = (*targIter1)->y + tmpy;
						z2 = (*targIter1)->z + tmpz;
						
						
						//cout << "coalesc 8" << endl;
						//cout << "x1,y1,x2,x2: " << x1 << "," << y1 << "," << x2 << ", " << x2 << endl;
						//cout << "coalesc1 updating target: " << (*targIter1)->id << endl;
						(*targIter1)->kalmanEstimateState(x1, y1,z1, (*targIter1)->hue , (*targIter1)->weight);
						//cout << "coalesc2 updating target: " << (*targIter2)->id << endl;
						(*targIter2)->kalmanEstimateState(x2, y2,z2, (*targIter2)->hue , (*targIter2)->weight);
						//cout << "coalesc 9" << endl;
					}
				}
			}
		}
	}
}

float DataAssociator::findAngle(TargetObject *t1, TargetObject *t2, DataElement3D data){
	float returnAngle = 0;
	
	float distt1data = getDistance(&data,t1);
	float distt2data = getDistance(&data,t2);
	float distt1t2 = getDistance(t1,t2);
	if ((distt1t2 > distt1data) && (distt1t2 > distt2data)){ // data in the middle of t1 and t2

		if (distt1data < distt2data){ // data closer to t1
			return distt1data;
		}
		else{
			return 0;
		}

	} 
	else if(distt1data < distt2data){ // data closer to t1 and not in the middle
		return 100.0+distt1data;
	}
	
	return returnAngle;
}


void DataAssociator::updateFinalData(TargetObject target, DataElement3D* finalData, DataElement3D curData){
	/// from the current data and the state of the final data update the finalData
	float curDataTargDist = sqrt((target.x - curData.x)*(target.x - curData.x) + (target.y - curData.y)*(target.y - curData.y) +(target.z - curData.z)*(target.z - curData.z) );

	/// see if the final data was already updated
	if (finalData->x < 0.){
		// if not then simply write the current data
		finalData->x = curData.x;
		finalData->y = curData.y;
		finalData->z = curData.z;
		finalData->hue = curData.hue;
		finalData->weight = curData.weight;
	}
	else{
		
	}
}


void DataAssociator::sendAttentionalInspection(){

	
	attentionInspecString = "";
	
	// now get the last three elements from the map 
	map<float, int>::reverse_iterator rii; 
	int total = 1;
	//std::cout << "size of att priority map = " << attPriorityMap.size() << std::endl;
	
	for(rii=attPriorityMap.rbegin(); rii != attPriorityMap.rend(); ++rii)
   	{
		if (total < 4){
		  float curX, curY, curZ; 
		  bool ok = getTargetXYZ(rii->second, curX, curY, curZ);
			if (ok){
			  attentionInspecString = attentionInspecString + QString("%1,%1,%1,%1, %1,").arg(rii->second).arg(curX).arg(curY).arg(curZ).arg(0);
			}
      			///std::cout << "priority = " << rii->first << ", id = " << rii->second << endl;
		}
		total++;

   	}

	//std::cout << "attentionInspecString = " << attentionInspecString  <<std::endl;

	// now send this string to the attention mechanism
	//attentionPositionClient->sendData(attentionInspecString);
}

bool DataAssociator::getTargetXYZ(int idd, float &curX, float &curY, float &curZ){
	vector<TargetObject*>::iterator targIter; 
	bool returnBool = false;
		
	for( targIter = targetVector->begin(); targIter != targetVector->end(); targIter++){ 
		
		if ((*targIter)->id == idd){
			returnBool = true;
			curX = (*targIter)->x;
			curY = (*targIter)->y;
			curZ = (*targIter)->z;

			// convert into XIM coordinates
			//curX = curX  - 280.0;  // x = 0...560 ---> xx = -280..280
	    		//curY = curY  - 267.0;  // y = 0...534 ---> yy = -267..267
			
	    		//curY = curY * -1;
		}

	}

	return returnBool;
}

void DataAssociator::setAttentionPriority(TargetObject *target){
	float curMinMahaDist = target->minMahaDist;
	float curHue = target->hue;

	// never been attended to
	if ((curHue < 0.) && (target->attentionalPriority < 2.)){target->attentionalPriority = 2.+ ((float)rand()/(float)RAND_MAX) ;}
	
	// attended before but now increases distance
	if ((curMinMahaDist < 100.) && (target->attentionalPriority < 50.)){
		target->attentionalPriority = target->attentionalPriority + 1. ;
	}
	
	if (target->attentionalPriority >  50.){
		target->attentionalPriority = 49. + ((float)rand()/(float)RAND_MAX);
	}

	attPriorityMap.insert(map<float, int>::value_type(target->attentionalPriority, target->id));
}

void DataAssociator::computeMinMahaDistance(TargetObject* target){
	vector<TargetObject*>::iterator targIter; 
	float curMahaDist;
	float minMahaDist = 999.;
	int totalTargets = 0;
	/// now reset the JPDA states
		
	for( targIter = targetVector->begin(); targIter != targetVector->end(); targIter++){ 
		
		if (target->id != (*targIter)->id){
			curMahaDist = getMahanalobisDist(*targIter, target);
			if (curMahaDist < minMahaDist){
				minMahaDist =  curMahaDist;
				totalTargets++;
			}
		}
	}

	if (minMahaDist < 0.1){minMahaDist = 0.1;}
	/// here 100 seems to be the crucial value
	///std::cout << "minimum mahanalobis distance of target " << target->id << " = " << minMahaDist << std::endl;
	target->minMahaDist = minMahaDist;
}

float DataAssociator::getMahanalobisDist(TargetObject* targOne, TargetObject* targTwo){
	float returnDist = 999.0;
	/// we look at the distribution of the target one and compute the Mahanalobis distance to the target two
	
	cvInvert((targOne->kalman)->error_cov_post, stateOneCovariance, CV_LU);
	
	/// state two - minus one 
	cvSub((targTwo->kalman)->state_post, (targOne->kalman)->state_post, stateMatrixOne, 0);
	
	/// transpose
	cvTranspose(stateMatrixOne, stateMatrixTwo);
	
	cvMatMul(stateMatrixTwo, stateOneCovariance, stateMatrixTwo);
	
	cvMatMul(stateMatrixTwo, stateMatrixOne, resultMahaDistMatrix);

	
	/// compute the square root 
	returnDist = sqrt(cvmGet(resultMahaDistMatrix,0,0));
	
	

	return returnDist;
}

void DataAssociator::setTransitionMatrix(){
	
	double trans_time = transition_mat_timer.getInactiveTime();
	double one_over_trans_time = 1.0 / trans_time;
	double delta_t_square_half = (trans_time * trans_time)/2.0;

	/// now write the transition matrix
// 	const float A[] = { 1.0, 	0.0,		trans_time , 	0.0, 		delta_t_square_half, 	0.0 ,
// 			0.0,	1.0,		0.0,		trans_time,	0.0,			delta_t_square_half,
// 				0.0,0.0,1.0,0.0,trans_time, 0.0,
// 				0.0,0.0,0.0,1.0,0.0,trans_time,
// 				0.0,0.0,0.0,0.0,1.0,0.0,
// 				0.0,0.0,0.0,0.0,0.0,1.0 }; // transition matrix

	const float A[] = {     1.0, 0.0, 0.0, trans_time, 0.0, 0.0, trans_time, 0.0, 0.0, 0.0, 0.0,
			    	0.0, 1.0, 0.0, 0.0, trans_time, 0.0, 0.0, trans_time, 0.0, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0, 0.0, trans_time, 0.0, 0.0, trans_time, 0.0, 0.0,
				0.0, 0.0, 0.0, 1.0, 0.0, 0.0, trans_time, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, trans_time, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, trans_time, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
			        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 1.0 
				}; // transition matrix


	                        

	memcpy( transition_mat->data.fl, A, sizeof(A));

	/// now write the control matrix
	//double delta_t_square_half = (trans_time * trans_time)/2.0;
	//const float C[] = {delta_t_square_half, 0.0, 0.0, delta_t_square_half, trans_time, 0.0, 0.0, trans_time};
	//memcpy( control_mat->data.fl, C, sizeof(C));

	transition_mat_timer.setActiveTime();
}


void DataAssociator::estimateAllTargetStatesJPDA(){
	// go thorough all the calculated events and update the targets state with the MCMC computed probability of this event
	map<int, MCMC_Event>::iterator mapIter;  
	vector<int>::iterator vecIter;  
	TargetObject* curTarget; // use targetVector->at to get a target
	DataElement3D* curData; // use allDataVector->at to get a data
	float postProb;	
	int dataID;
	int curTargID;
	vector<int> dataToTarget;  

	for( mapIter = all_Events.begin(); mapIter != all_Events.end(); mapIter++){ 
		postProb = mapIter->second.postProb; 
		if (postProb > 0){
			dataToTarget = *(mapIter->second.dataToTarget); 
			curTargID = 0;	

			//cout << "dataAssociator before for" << endl;
			for( vecIter = dataToTarget.begin(); vecIter != dataToTarget.end(); vecIter++){ 
				curTarget = targetVector->at(curTargID);
				
// 				if (curData->sensorID < 2){
// 					curData->hue = curTarget->hue;
// 					curData->weight = curTarget->weight;
// 				}
// 				else if (curData->sensorID == 2){
// 					//curData->hue = curTarget->hue;
// 					curData->weight = curTarget->weight;
// 				}
// 				else if (curData->sensorID == 3){
// 					curData->hue = curTarget->hue;
// 					//curData->weight = curTarget->weight;
// 				}
				//cout << " da " << *vecIter;
   	 			if (*vecIter != -1){

					dataID = *vecIter;
					curData = allDataVector->at(dataID);
					curTarget->storeJPDAevent(curData->x, curData->y, curData->z, curData->hue, curData->weight, postProb);	 	
					curTarget->timer.setActiveTime();
				}
				else{
					//cout << "else case " << endl;
					curTarget->storeJPDAevent(curTarget->x, curTarget->y, curData->z, curData->hue, curData->weight,postProb); 

					

				}
				curTargID++;
  			}
			//cout << endl;
		}
    		
  	}

	//cout << "before 2nd for " << endl; 
	/// now after all combined innovations have been
	/// computed, JPDA update the targets state
	vector<TargetObject*>::iterator tarIter;
	for (tarIter = targetVector->begin();tarIter != targetVector->end();tarIter++){
		//(*tarIter)->updateJPDAstate(transition_mat); /// original jpda call
		//std::cout << "dataassoc: before updateJPDAstatenew: targ: " << (*tarIter)->id << ", " << (*tarIter)->x << ", " << (*tarIter)->y << ", " << (*tarIter)->hue << " , " << (*tarIter)->weight << std::endl;
		(*tarIter)->updateJPDAstateNew(transition_mat); /// new jpda call
		//std::cout << "dataassoc: after updateJPDAstatenew: targ: " << (*tarIter)->id << ", " << (*tarIter)->x << ", " << (*tarIter)->y << ", " << (*tarIter)->hue << " , " << (*tarIter)->weight << std::endl;
		//(*tarIter)->updateKalmanState(transition_mat,(allDataVector->at(0))->x, (allDataVector->at(0))->y); /// just to  try kalman without JPDA
	}

	//cout << "after 2nd for " << endl;
}



// prints out a CvMat given the pointer to its data and a string
void DataAssociator::printMatrix(CvMat* mat,  QString name){

	CvSize size = cvGetSize( mat );
	int rows = size.height;
	int cols = size.width; 
	

	float *invdata = mat->data.fl;
	int invcovn = mat->cols;
	std::cout << "-------" << name << "-------- " << std::endl; 
	std::cout << "size = " << rows << ", " << cols << std::endl;

	for (int i = 0; i < rows; i ++){
		for (int j =0; j < cols; j++){
			std:: cout << invdata[i * invcovn + j] << ", ";
		}
		std::cout << std::endl;
	} 
// 	std:: cout << invdata[0 * invcovn + 0] << ", " << invdata[0 * invcovn + 1] << ", " << invdata[0 * invcovn + 2] << std::endl << invdata[1 * invcovn + 0] << ", " << invdata[1 * invcovn + 1] << ", " << invdata[1 * invcovn + 2] << std::endl << invdata[2 * invcovn + 0] << ", " << invdata[2 * invcovn + 1] << ", " << invdata[2 * invcovn + 2] << std::endl;

	
}




// this function should use the validationmatrix and 
// use the MCMC algorithm to fill all_Events map
void DataAssociator::computeJPDAeventProbs(){


	//cout << "noftargets = " << nofTargets << ", nofData = " << nofData << endl;

	/// empty all events map
	emptyAllEvents();

	/// generate an initial feasible event
	MCMC_Event initial_event = generateInitialFeasibleEvent();
	
	/// insert this event into all_events map
	all_Events.insert( make_pair( 1, initial_event ) ); 

	/// now perform MCMC for (nofData * nofData) times
	vector<int> tmpEventVectorOne = *(initial_event.dataToTarget);
	vector<int> tmpEventVectorTwo;
	bool foundFeasElem = false;


	if ((nofData > 0) && (nofTargets > 0)){
	int maxSteps = MONTE_CARLO_ITERATIONS;
	if ((nofData * nofData) > maxSteps){maxSteps = nofData * nofData;}
	for (int mcmcStep = 0; mcmcStep < maxSteps; mcmcStep++){
		
		//cout << "ok 1, nofData = " << nofData << ", nofTargets = " << nofTargets << endl;

		// sample randNr Unif[0,1]
		double randNr = gsl_rng_uniform (rGSL);
		//std::cout << "randNr = " << randNr << std::endl;
		if (randNr < 0.5){
			tmpEventVectorTwo = tmpEventVectorOne;
			foundFeasElem = true;			
		}
		else{
			// choose e = (u,v) elem E uniformly at random 	
	
			//cout << "in first else..." << endl;		 	

			
			foundFeasElem = findRandomFeasibleElem();
			if (foundFeasElem){
				int rowNr = randomRow; //0..nofdata-1
				int colNr = randomCol;
		
				//cout << "rownNr = " << rowNr << ", colNr = " << colNr << endl;
			

			
				//-------  now we found an e elem E 	-------------------  //
				// ------- now compute a new event -------------------- //
				// first compute if e is contained in current event
				//cout << "found edge e..." << endl;
				vector<int>::iterator iter;  
				bool contains = false;
				for( iter = tmpEventVectorOne.begin(); iter != tmpEventVectorOne.end(); iter++){ 
    					if (*iter == rowNr) {
						contains =true;	
					
					}
  				}
 
				//cout << "computed contains .." << endl;
				if (tmpEventVectorOne.at(colNr) == rowNr){
					// e is element of current event

					// w' = w - e
					tmpEventVectorTwo = tmpEventVectorOne;
					tmpEventVectorTwo.at(colNr) = -1;
				}
				else{
					if (tmpEventVectorOne.at(colNr) == -1){
					// u and v unmatched in w
					// w' = w + e
					tmpEventVectorTwo = tmpEventVectorOne;
					tmpEventVectorTwo.at(colNr) = rowNr;

		 
					}
					else if (!contains || 	(tmpEventVectorOne.at(colNr) == -1)){
						// w' = w + e - e'
						tmpEventVectorTwo = tmpEventVectorOne;
						tmpEventVectorTwo.at(colNr) = rowNr;
					
					}
					else{
						tmpEventVectorTwo = tmpEventVectorOne;
					}
				}
				//cout << "computed new event .." << endl;

				// -------------- finished computing a new event ----------- //
			} // if foundFeasElem
			else{
				break;
			}
			
		} // if else (randNr < 0.5)


		/// now set the new event as current event with prob A(w, w')
		if (!foundFeasElem){ break;}
		else{		
			if (proposalGood(tmpEventVectorOne, tmpEventVectorTwo )){
				///cout << " -------------> new event " << endl;
				manageNewMCMCEvent(tmpEventVectorTwo);
				tmpEventVectorOne = tmpEventVectorTwo;
			}
			else{
				manageNewMCMCEvent(tmpEventVectorOne);
			}
			//cout << "computed proposalGood .." << endl;
		}
		
	} // while MCMC loop
	

	//cout << "ok 2" << endl;

	//if (foundFeasElem){
		/// now we have generated many events using MCMC
		/// so look at the hitcounts and compute the prob of events
		//std::cout << "events size = " << all_Events.size() << std::endl;
		float totalHits = 0;
		map<int, MCMC_Event>::iterator iter;  
		for( iter = all_Events.begin(); iter != all_Events.end(); iter++){ 

    			totalHits = totalHits +  iter->second.hitCount; 
  		}
		//cout << "ok 3, totalHits = " << totalHits <<  endl;	
		for( iter = all_Events.begin(); iter != all_Events.end(); iter++){ 
			if (totalHits > 0){
    				iter->second.postProb =  iter->second.hitCount/totalHits; 
			}
			else{
				iter->second.postProb = 0;
			}
  		}
		//cout << "ok 4" << endl;

	//}

	
	/// ----------- just a print out of events and probs -------- //
	
	///cout << "------------ printing events and their probs ---------" << endl;
	//map<int, MCMC_Event>::iterator iter;
	
	//for( iter = all_Events.begin(); iter != all_Events.end(); iter++){ 
    			///cout << "event : " <<  iter->first << ", prob = " << iter->second.postProb << endl;
			///cout << "....... " ; printVector(*(iter->second.dataToTarget));
  		}

	///cout << "------------------------------------------------------" << endl;
	
	/// ---------------------------------------------------------- //	

	///} // if ..nofData > 0 && nofTargets > 0

}

void DataAssociator::printVector(vector<int> dataToTarget){
	vector<int>::iterator iter;
	for( iter = dataToTarget.begin(); iter != dataToTarget.end(); iter++){ 
		cout << *iter << ", ";
	}
	cout << endl;
}

bool DataAssociator::proposalGood(vector<int> tmpEventVectorOne, vector<int> tmpEventVectorTwo ){

	// compute Gaussian disttribution prob N(v) of v, with
	// mean u and covariance B_k_u

	// this is equivalent to: compute Gaussian disttribution prob N(v) of 
	// v - u, with mean (0,0) and covariance B_k_u

	
	int sizeOne =  tmpEventVectorOne.size();
	int sizeTwo =  tmpEventVectorTwo.size();
	double probOne = 1.0;
	double probTwo = 1.0;
	
	//cout << "in proposalGood: sizeOne = " << sizeOne << endl;
	//cout << "in proposalGood: sizeTwo = " << sizeTwo << endl;
	// printing the two events
	//cout << "event 1: tmpEventVectorOne = " << endl;
	//for (int i = 0; i < sizeOne; i ++){
	//	cout << tmpEventVectorOne.at(i) << ", ";
	//}
	//cout << endl;
	//cout << "event 2: tmpEventVectorTwo = " << endl;
	//for (int i = 0; i < sizeTwo; i ++){
	//	cout << tmpEventVectorTwo.at(i) << ", ";
	//	}
	//cout << endl;

	/// to be used for calculations of both event One and two probs
	TargetObject* curTarget;
	DataElement3D* curData; 
		

	/// false alarm rate: now it is the same for all sensors
	double lambda = 0.1;
	/// target detection rate: now it is the same for all sensors
	double p_detect = 0.9;
	/// target miss rate: now it is the same for all sensors
	double p_miss = 1 - p_detect;
	/// the coeffcient of the proposal distribution
	double coeff;
	/// number of matchings for event one and two
	int cardOne = 0; int cardTwo = 0;
	/// for the bivariate Gaussian distribution
	double rho, sigma_x, sigma_y, sigma_z;
	double curProb;
	double xGauss, yGauss, zGauss;
	/// the proposal ratio
	double ratio;
	
	/// ------------- compute P for event One ---------------- //
	for  (int i = 0;i < sizeOne; i++)
	{
		//cout << "i = " << i << endl;
		// update cardinality for this event
		if (!(tmpEventVectorOne.at(i)== -1)){
			cardOne++;

			//cout << "inside if 1 " << endl;
			// first compute the covariance of innovation for this target 
			// data association
			curTarget = targetVector->at(i);
			
			//cout << "size allDataVector 1 = " << allDataVector-> size() << endl;
			curData = allDataVector->at(tmpEventVectorOne.at(i));
			cvMatMul(curTarget->kalman->measurement_matrix,curTarget->kalman->error_cov_pre, tempp1);
			cvTranspose(curTarget->kalman->measurement_matrix, tempp2);
			cvMatMul(tempp1, tempp2, tempp3);
			cvAdd(curTarget->kalman->measurement_noise_cov, tempp3, innovCovv);
		
			//cout << "computed innovCovv1" << endl;

			sigma_x = sqrt(cvmGet(innovCovv,0,0)); 
			sigma_y = sqrt(cvmGet(innovCovv,1,1)); 
			rho = cvmGet(innovCovv,0,1)/(sigma_x * sigma_y);

			curPredd = curTarget->kalman->state_pre;
			xGauss = curData->x - cvmGet(curPredd,0,0);
			yGauss = curData->y - cvmGet(curPredd,1,0);
			zGauss = curData->z - cvmGet(curPredd,2,0);

			//cout << "xGauss, yGauss, sigma_x, sigma_y, rho = " << xGauss << ", " << yGauss << ", " <<  sigma_x << ", " <<  sigma_y << ", " <<  rho << endl;

			curProb = gsl_ran_bivariate_gaussian_pdf (xGauss, yGauss, sigma_x, sigma_y, rho);


			/// 13/11/2008: cam data 4 time smore probabale
			if (curData->sensorID == 1){
				curProb = curProb * FLOOR_DATA_WEIGHT_INSIDE;// * 0.25;
			}
			/// 14/11/2008: also look at the distance from the prediction to the current data
			float curPredDataDist = sqrt(xGauss*xGauss + yGauss * yGauss + zGauss * zGauss );
			///curProb = curProb * (1./curPredDataDist);

			//cout << "computed curProb1 = " << curProb << endl;
			probOne = probOne * curProb;	
		}
	}
	//cout << "computed prob 1 = " << probOne << endl;
	// compute the coefficient for the proposal distribution
	coeff = pow(lambda, (nofData - cardOne)) * pow(p_detect, cardOne) * pow(p_miss, (nofTargets - cardOne));	
	probOne = probOne * coeff;
	//cout << "computed coeff1 = " << coeff << endl;
	/// ------------------------------------------------------- ///

	/// ------------- compute P for event Two ---------------- //
	for  (int i = 0;i < sizeTwo; i++)
	{

		//cout << "i = " << i << endl;
		// update cardinality for this event
		if (!(tmpEventVectorTwo.at(i)== -1)){cardTwo++;


			//cout << "gugu1" << endl;		
			// first compute the covariance of innovation for this target 
		// data association
			curTarget = targetVector->at(i);
		
			//cout << "size allDataVector 2= " << allDataVector-> size() << endl;
			//cout << "tmpEventVectorTwo.at(i) = " << tmpEventVectorTwo.at(i) << endl;

			curData = allDataVector->at(tmpEventVectorTwo.at(i));

			//cout << "gugu3" << endl;
			cvMatMul(curTarget->kalman->measurement_matrix,curTarget->kalman->error_cov_pre, tempp1);
			//cout << "gugu4" << endl;
			cvTranspose(curTarget->kalman->measurement_matrix, tempp2);
			//cout << "gugu5" << endl;
			cvMatMul(tempp1, tempp2, tempp3);
			//cout << "gugu6" << endl;
			cvAdd(curTarget->kalman->measurement_noise_cov, tempp3, innovCovv);
			//cout << "computed innovCovv2" << endl;
		
			sigma_x = sqrt(cvmGet(innovCovv,0,0)); 
			sigma_y = sqrt(cvmGet(innovCovv,1,1)); 
			sigma_z = sqrt(cvmGet(innovCovv,2,2)); 
			rho = cvmGet(innovCovv,0,1)/(sigma_x * sigma_y * sigma_z);

		
			curPredd = curTarget->kalman->state_pre;
			xGauss = curData->x - cvmGet(curPredd,0,0);
			yGauss = curData->y - cvmGet(curPredd,1,0);
			zGauss = curData->z - cvmGet(curPredd,2,0);
			//cout << "computed curPredd, xGauss and yGauss 2" << endl;

			curProb = gsl_ran_bivariate_gaussian_pdf (xGauss, yGauss, sigma_x, sigma_y, rho);
			//cout << "computed curProb2 =  " <<  curProb << endl;

			/// 13/11/2008: cam data 4 time smore probabale
			if (curData->sensorID == 1){
				curProb = curProb * FLOOR_DATA_WEIGHT_INSIDE;//* 0.25;
			}
			/// 14/11/2008: also look at the distance from the prediction to the current data
			float curPredDataDist = sqrt(xGauss*xGauss + yGauss * yGauss + zGauss * zGauss);
			///curProb = curProb * (1./curPredDataDist);


			probTwo = probTwo * curProb;	
		}
	}
	//cout << "computed prob 2 = " << probTwo << endl;
	// compute the coefficient for the proposal distribution
	double pow1 = pow(lambda, (nofData - cardTwo));
	double pow2 = pow(p_detect, cardTwo);
	double pow3 = pow(p_miss, (nofTargets - cardTwo));
	//cout << "lambda = " << lambda << ", nofData = " << nofData << ", cardTwo = " << cardTwo << ", p_detect = " << p_detect << ", p_miss = " << p_miss << ", nofTargets = " << nofTargets << ", pow1 = " << pow1 << ", pow 2 = " << pow2 << ", pow 3 = " << pow3 << endl;
	coeff = pow(lambda, (nofData - cardTwo)) * pow(p_detect, cardTwo) * pow(p_miss, (nofTargets - cardTwo));	
	probTwo = probTwo * coeff;
	/// ------------------------------------------------------- ///
	//cout << "computed coeff 2 = " << coeff << endl;
	//cout << "probOne, probTwo = " << probOne << ", " << probTwo << endl;

	ratio = probTwo/ probOne;
	/// 14/11/2008: /3.0 added, if not acceptance row is too low
	double acceptance = GSL_MIN (1.0, ratio) / 3.0;
	// now generate a random number between 0 and 1
	double randNr = gsl_rng_uniform (rGSL);

	
	if (randNr >= acceptance ){ return true;}
	else {return false;}

	
}


void DataAssociator::manageNewMCMCEvent(vector<int> eventVector){
	int key = -1;
	bool exists = false;
	/// now see if this event already exists
	map<int, MCMC_Event>::iterator iter;  
	vector<int>::iterator vectorIt;
	int sameCounter = 0;
	for( iter = all_Events.begin(); iter != all_Events.end(); iter++){ 
    		///if (*(iter->second.dataToTarget) == eventVector) {
		///	exists =true;
		///	key = iter->first;
		///}
		int tmpCounter = 0;
		bool done = false;
		for (vectorIt = (*(iter->second.dataToTarget)).begin(); vectorIt != (*(iter->second.dataToTarget)).end(); vectorIt++){
			if (*vectorIt != eventVector.at(tmpCounter)){
				done = true;
			}
			tmpCounter++;
			
		}	
		// now if done==false both the vectors are the same
		if (!done){ 
			exists =true;
			key = iter->first;
		}
  	}

	if (!exists){
		//cout << endl << "+++++++++++++++ new unique event found.." << endl << endl;	

		vector<int>* dtoTarg = new vector<int>;
		*dtoTarg = eventVector;
		MCMC_Event event;
		event.hitCount = 1;
		event.postProb = 0.0;
		event.dataToTarget = dtoTarg;
		
		// get size of all_events map
		int sizeMap = all_Events.size();
		//cout << "size of all_events before insertion = " << sizeMap << endl;
		all_Events.insert( make_pair( sizeMap + 1, event ) ); 
		//cout << "size of all_events after insertion = " <<  all_Events.size() << endl << endl;
	}
	else{
		  map<int, MCMC_Event>::iterator iter = all_Events.find(key);
		iter->second.hitCount++;
	}
}
// release memory for the vectors inthe map and empty the map
void DataAssociator::emptyAllEvents(){
	map<int, MCMC_Event>::iterator iter;  
	for( iter = all_Events.begin(); iter != all_Events.end(); iter++){ 
    		if (iter->second.dataToTarget) {delete iter->second.dataToTarget;}
  	}

	all_Events.clear();

}


// generates a feasible JPDA event using the validatin matrix
DataAssociator::MCMC_Event DataAssociator::generateInitialFeasibleEvent(){
	vector<int>* dtoTarg = new vector<int>;
	MCMC_Event event;
	
	// boolean to set which data was used
	bool usedData[nofData];
	for (int i =0; i < nofData; i++){
		usedData[i] = false;
	}

	// now go through all targets and set a the data for each
	for (int j = 0; j < nofTargets; j++){
		int curDataNr = -1;
		for (int i =0; i < nofData; i++){
			if ((validationMatrix->isTrue(i,j)) && (!usedData[i])){
				dtoTarg->push_back(i);
				usedData[i] = true;
				curDataNr = i;
				break;
			}
		}
		if (curDataNr == -1){dtoTarg->push_back(-1);}
	}

	// write this into the event struct
	event.hitCount = 0;
	event.postProb = 0.0; 
	event.dataToTarget = dtoTarg;

	return event;
}

void DataAssociator::predictSingleTargetState(int index){
	TargetObject* targ;
	targ = targetVector->at(index);
	targ->kalmanPredictState();
}

void::DataAssociator::predictAllTargetStates(){
	TargetObject* targ;
	vector<TargetObject*>::iterator iter;
	for (iter = targetVector->begin(); iter != targetVector->end();iter++){
		targ = *iter;
		targ->kalmanPredictState();
	}
}


void DataAssociator::updateValidationMatrix(){
	
	//cout << "--------------> in updateVaidation 1 = " << endl;

	if ((targetVector != NULL) && (allDataVector != NULL)){
		//cout << "testing 1 " << endl;
		if (!(targetVector->empty())){
			//cout << " targetVector not empty..." << endl;
			nofTargets = targetVector->size();
		}
		else{
			nofTargets = 0;
		}
		//cout << "kuku.." << endl;
		if (!(allDataVector->empty())){
			//cout << " allDataVector not empty..." << endl;
			nofData = allDataVector->size();
		}
		else{
			nofData = 0;
			
		}
		//cout << "hoho.." << endl;

		if (nofData * nofTargets > 0){
		//cout << "testing 1.5 " << endl;
	//	cout << "--------------> in updateVaidation 3 = " << endl;
		/// ---- RE - inilialize the validation matrix -----
		// so delete it first
		
		if (validationMatrix != NULL) {delete validationMatrix;}
		//cout << "testing 2 " << endl;
		
		
		//cout << "testing 3 " << endl;
	//	cout << "--------------> in updateVaidation 2 = " << endl;

		

		//cout << "testing 4 " << endl;
		// then allocate again
		validationMatrix = new ValidationMatrix(nofData, nofTargets);
		//cout << "testing 5 " << endl;
		

		float tmp_arr[] = {0,0,0};
		float tmpMat[] = { 1,  0,  0,
		0,  1,  0,
		0, 0, 1 };
	
			
		CvMat* mes_prediction = cvCreateMat(MES_SIZE,1,CV_32FC1);
		DataElement3D* curData; 
		TargetObject* target;
		
		
			
		double value = 100000;
		double gateParam1, gateParam2 ;
		float minAxis, maxAxis;
		float predFactor;

	//	cout << "--------------> in updateVaidation 7 = " << endl;	
			
		for (int row = 0; row < nofData; row ++){
			// create a vector with this data (just x, y, hue, weight )
			curData = allDataVector->at(row);
	
			gateParam1 = GATE_PARAM1;
			gateParam2 = GATE_PARAM2; 
			minAxis = MIN_AXIS; 
			maxAxis = MAX_AXIS; 
			predFactor = PRED_FACTOR; 
				
				
		
			
			// for each row (measurements) go through all the targets (colums)
			//cout << "--------------> in updateVaidation 9 = " << endl;
			for (int column = 0; column < nofTargets; column++){
				// get the target
				target = targetVector->at(column);
				
				
				//if (insideValidationGate(target, *curData, gateParam1, gateParam2, minAxis, maxAxis) && (getAssociationOk(curData, target)) ){
				if (insideValidationGate(target, *curData, gateParam1, gateParam2, minAxis, maxAxis, predFactor) ){	
					validationMatrix->setTrue(row,column);
					curData->cam_data_to_target = target->id; 

				
					//std::cout << "setting true val mat: " << row << ", " << column << std::endl;
					validationMatrix->setTrue(row,column);

					
				}
				else{
					validationMatrix->setFalse(row,column);
				}
				
			}
		}
			
	
		
		} //if targetVector->size() > 0 && allDataVector->size() > 0
	} // if targetvector != NULL && allDataVector != NULL

	//cout << "65" << endl;
}

bool DataAssociator::getAssociationOk(DataElement3D* curData, TargetObject* targetInstance){
  return true;

}


QString DataAssociator::getGaussianDistributedData(QString curData){

	QString returnString = "";
	const char TARGET_SEP = ';';	// the individual datas are separated by this..has nothing to do with targets!
   	const char DATA_SEP = ','; // the elements inside one data are
	bool ok;

	QStringList fields = QStringList::split( TARGET_SEP, curData );
		
	// read nr of data
	//int nofData = fields[0].toInt(&ok, 10 );
	//std::cout << "nofData = " << nofData << endl;
	// read individual data
	QStringList::Iterator itt = fields.begin();
	int nData = (*itt).toInt(&ok, 10 );
	itt++;
	
	int totalData = 0;
	if (nData > 0){

		/// for Gaussian spreading of data
	gsl_rng_env_setup();
  	type = gsl_rng_default;
  	r = gsl_rng_alloc (type);
		

		for ( QStringList::Iterator it = itt; it != fields.end(); ++it ) {
			
			//cout << "hehe 1" << endl;	

			QString curTargData = *it;
			///cout << "curData QString= " << curData << endl;
			QStringList dataFields = QStringList::split( DATA_SEP, curTargData );	

			
			
			//cout << "hehe 2" << endl;
			QStringList::Iterator iter = dataFields.begin();
			double x = (*iter).toDouble( &ok );
			iter++;
			double y = (*iter).toDouble( &ok );	
			iter++;
			double z = (*iter).toDouble( &ok );	
			
			/// create new points for this point and add to the string
// 			float distArg = 15.0;//CAMERA_THRESHOLD_2 + 2.0;
// 			returnString = returnString + QString(";%1,%1").arg(x).arg(y);
// 			totalData++;
// 			returnString = returnString + QString(";%1,%1").arg(x-distArg).arg(y);
// 			totalData++;
// 			returnString = returnString + QString(";%1,%1").arg(x+distArg).arg(y);
// 			totalData++;
// 			returnString = returnString + QString(";%1,%1").arg(x).arg(y+distArg);
// 			totalData++;
// 			returnString = returnString + QString(";%1,%1").arg(x).arg(y-distArg);
// 			totalData++;


			/// see how far this point is from the center
			float distToCentre = sqrt((x - X_LENGTH/2.)*(x - X_LENGTH/2.) + (y - Y_LENGTH/2.)*(y - Y_LENGTH/2.));
			float minD = 0.0;
			float maxD = 680.0;
			float maxDataDist = 25.0;

			

			if ((distToCentre > minD) && (distToCentre < maxD)){
				float deltaX = fabs(x - X_LENGTH/2.);
				float deltaY = fabs(y - Y_LENGTH/2.);
				

				/// now get some Gaussian data around the data
				for (int i = 1; i < NOF_SPREAD_DATA; i++){
					
					float randx = gsl_ran_gaussian (r, GAUSSIAN_SIGMA_DATA_SPREAD);
					float randy = gsl_ran_gaussian (r, GAUSSIAN_SIGMA_DATA_SPREAD);
					

					returnString = returnString + QString(";%1,%1").arg(x + randx).arg(y + randy);
 					totalData++;
					//std::cout << "rand: " << randx << ", " << randy << std::endl;
				}
// 				if ((x + maxDataDist < X_LENGTH) && (y + maxDataDist < Y_LENGTH)){
// 					returnString = returnString + QString(";%1,%1").arg(x + maxDataDist).arg(y + maxDataDist);
//  					totalData++;
// 				}
// 				if ((x + maxDataDist < X_LENGTH) && (y - maxDataDist > 0)){
// 					returnString = returnString + QString(";%1,%1").arg(x + maxDataDist).arg(y - maxDataDist);
//  					totalData++;
// 				}
// 				if ((x - maxDataDist > 0.) && (y - maxDataDist > 0)){
// 					returnString = returnString + QString(";%1,%1").arg(x - maxDataDist).arg(y - maxDataDist);
//  					totalData++;
// 				}
// 
// 				if ((x - maxDataDist > 0.) && (y + maxDataDist < Y_LENGTH)){
// 					returnString = returnString + QString(";%1,%1").arg(x - maxDataDist).arg(y + maxDataDist);
//  					totalData++;
// 				}
// 
// 				if ((x + maxDataDist < X_LENGTH) ){
// 					returnString = returnString + QString(";%1,%1").arg(x + maxDataDist).arg(y);
//  					totalData++;
// 				}
// 
// 				if ((x - maxDataDist > 0.) ){
// 					returnString = returnString + QString(";%1,%1").arg(x - maxDataDist).arg(y);
//  					totalData++;
// 				}
// 				if ((y - maxDataDist > 0.) ){
// 					returnString = returnString + QString(";%1,%1").arg(x).arg(y - maxDataDist);
// 	 				totalData++;
// 				}
// 
// 				if ((y + maxDataDist < Y_LENGTH) ){
// 					returnString = returnString + QString(";%1,%1").arg(x).arg(y + maxDataDist);
//  					totalData++;
// 				}
				
				
				returnString = returnString + QString(";%1,%1").arg(x).arg(y);
 				totalData++;
			}
			else{ // if close enough to center
				returnString = returnString + QString(";%1,%1").arg(x).arg(y);
 				totalData++;
			}

		}
	}
	returnString = QString("%1").arg(totalData) + returnString;
	return returnString;
	
}
bool DataAssociator::findRandomFeasibleElem(){
	// set randomRow and randomCol so that this represents 
	// a feasible (i.e. > 0) element 
	//cout << "in findRandomFeasibleElem" << endl;
	bool retVal = false;

	rowFeasVec->clear();
	colFeasVec->clear();
	//std::cout << "val mat = " << validationMatrix->nofData << ", " <<  validationMatrix->nofTargets << std::endl;
	if ((validationMatrix->nofData > 0) && (validationMatrix->nofTargets > 0)){
		for (int i = 0; i < validationMatrix->nofData; i ++){
			for (int j = 0; j < validationMatrix->nofTargets; j ++){
				//cout << "i, j = " << i << ", " << j << endl;
				if (validationMatrix->isTrue(i,j)) {
					//cout << "is true i, j = " << i << ", " << j << endl;
					rowFeasVec->push_back(i);
					colFeasVec->push_back(j);
					retVal = true;
				}
			}
		
		}

		// now generate a random number between 0 and size of feasibleMap 
		srand ( time(NULL) );
		if (retVal){
			int iSecret = rand() % rowFeasVec->size();
			//std::cout << "size = " << rowFeasVec->size() << ", iSecret = " << iSecret << std::endl;
			randomRow = rowFeasVec->at(iSecret);
			randomCol = colFeasVec->at(iSecret);
		}
	}

	//cout << "finished findRandomFeasibleElem" << endl;
	return retVal;
	
}


void DataAssociator::avoidCoalescence(){
		bool noCollision;
	float curCollDist =1000000;;
	float trgX1 , trgY1 ,trgX2 , trgY2 ;
	TargetObject* trgObj1;  
	TargetObject* trgObj2;
	// go through all other targets and see if any other target too close
	vector<TargetObject*>::iterator tarIter;
	for (tarIter = targetVector->begin();tarIter != targetVector->end();tarIter++){
		noCollision = true;
		trgObj1 = *tarIter;//targetVector->at(ii);
	  	trgX1 = trgObj1->x;
		trgY1 = trgObj1->y;
		
		

		vector<TargetObject*>::iterator tarIter2;
		for (tarIter2 = targetVector->begin();tarIter2 != targetVector->end();tarIter2++){
		///for (int jj = 0; jj < targetVector->size(); jj++){
			///if (ii != jj){
			if (tarIter != tarIter2){	
				///trgObj2 = targetVector->at(jj);
				trgObj2  = *tarIter2;
	  			trgX2 = trgObj2->x;
				trgY2 = trgObj2->y;
	

				float tmpDist = sqrt((trgX1 - trgX2)*(trgX1 - trgX2) + (trgY1 - trgY2)*(trgY1 - trgY2));

				if (tmpDist < 50){
				// move these two targets apart
					// move target 1
					// this is too hard movement correction
					// need something slower
/*
					float deltaX1 = trgObj1->xOld - trgX1;
					float deltaY1 = trgObj1->yOld - trgY1;
					if (deltaX1 < 0) deltaX1 = deltaX1 * -1;
					if (deltaY1 < 0) deltaY1 = deltaY1 * -1;

					float tmpX;
					float tmpY;
		
					if (deltaX1 > deltaY1){
						tmpX = trgObj1->xOld; 
						tmpY = trgObj1->y;
					}
					else{
						tmpX = trgObj1->x;
						tmpY = trgObj1->yOld;// + (trgObj1->yOld - trgY1)/2;
					}
			*/

					//updateTargetState(trgObj1->id, tmpX, tmpY);
				  trgObj1->kalmanEstimateState(trgObj1->xOld, trgObj1->yOld, trgObj1->zOld, trgObj1->hueOld, trgObj1->weightOld);
					
					
					
					cout << "----------collision avoidance performed-------" << endl;

				}
			}
		}

		

				
	}
}


double DataAssociator::getFloorDataWeight(DataElement3D* curData){
	double retWeight;
	/// if data is on the border then return a high weight
	if (curData->x > (X_LENGTH - 120.0)){
		retWeight = FLOOR_DATA_WEIGHT_BORDER;
	}
	else if (curData->x < ( 120.0)){
		retWeight = FLOOR_DATA_WEIGHT_BORDER;
	}
	else if (curData->y > (Y_LENGTH-110.0)){
		retWeight = FLOOR_DATA_WEIGHT_BORDER;
	}
	else if (curData->y < (110.0)){
		retWeight = FLOOR_DATA_WEIGHT_BORDER;	
	}
	else{
		retWeight = FLOOR_DATA_WEIGHT_INSIDE;
	}
	return retWeight;
}


double DataAssociator::getCamDataWeight(DataElement3D* curData){
	double retWeight;
	/// if data is on the border then return a low weight
	if (curData->x > (X_LENGTH - 100.0)){
		retWeight = CAM_DATA_WEIGHT_BORDER;
	}
	else if (curData->x < ( 100.0)){
		retWeight = CAM_DATA_WEIGHT_BORDER;
	}
	else if (curData->y > (Y_LENGTH-97.0)){
		retWeight = CAM_DATA_WEIGHT_BORDER;
	}
	else if (curData->y < (97.0)){
		retWeight = CAM_DATA_WEIGHT_BORDER;	
	}
	else{
		retWeight = CAM_DATA_WEIGHT_INSIDE;
	}
	return retWeight;
}
void DataAssociator::estimateAllTargetStatesValidationGate(){

	DataElement3D* curData;
	
	TargetObject* target;
	
	/// go through all targets
	vector<TargetObject*>::iterator tarIter;
	for (tarIter = targetVector->begin();tarIter != targetVector->end();tarIter++){

		float totalDist = 0.0;
		double gateParam1, gateParam2 ;
		float minAxis, maxAxis;
		float predFactor;
		float dist;
		map<float, DataElement3D > tempDataMap;	
		map<float, DataElement3D > finalDataMap;	
		map<float, DataElement3D > finalProbMap;

		float predX, predY, predZ;
		float dataWeight;

		bool done;

		target = *tarIter;
		DataElement3D* finalData = new DataElement3D(0,0.0,0.0,0.0,0.0, 0.0);
		done = false;
		
		predX = cvmGet((target->kalman)->state_pre,0,0);
		predY = cvmGet((target->kalman)->state_pre,1,0);
		predZ = cvmGet((target->kalman)->state_pre,2,0);
		
		bool hueFound = false;
		bool weightFound = false;
		double newHue;
		double newWeight;
		//std::cout << "-----------target pred = " << predX << ", " << predY << std::endl;
		
		

		/// now go through all data
		 vector<DataElement3D*>::iterator allDataIter;
		for (allDataIter = allDataVector->begin(); allDataIter != allDataVector->end();allDataIter++){

			curData = *allDataIter;
			
		
			/// ---assumtion: maximum one cam data for a person!!!
			/// see 
				/// - if this data is inside the validation gate of this target
				/// - if the target is already associated to a camdata then avoid another association to a cam data
				/// - if the current data is cam data, then make sure it is only associated to one target
				/// - and finally if the data is not too close to another existing target

			
			
			  if ( /*!(target->associated_to_cam_data && (curData->sensorID == 0)) && 
				 (curData->dataUsed != true) && */ 
			     (insideValidationGate(target, *curData, GATE_PARAM1, GATE_PARAM2, MIN_AXIS, MAX_AXIS, predFactor) ) && (clearNeighbourHood(target, curData))){	

				if (curData->sensorID == 2){
					hueFound = true;
					newHue = curData->hue;
				}

				//std::cout << "data inside validation gate" << std::endl;
				/// if yes then update the current data
				dist = 0.01 + (1.0/dataWeight)*sqrt((curData->x - predX)*(curData->x - predX)+(curData->y - predY)*(curData->y - predY) );
				
				totalDist = totalDist + dist;
				tempDataMap.insert( make_pair( dist,  *curData));
				curData->dataUsed = true;

				if (curData->sensorID == 0){
				  //target->associated_to_cam_data = true;
				}
				
				
				
				}
		}


		//std::cout << "totalDist= " << totalDist << std::endl;
		//std::cout << "tempDatamap size = " << tempDataMap.size() << std::endl;
		/// now compute the correct probabilities using the totalDist	
		float totalProb = 0.0;
		
		map<float, DataElement3D>::iterator iter;
		for( iter = tempDataMap.begin(); iter != tempDataMap.end(); iter++ ) {
			float newProb;
			
			if ((iter->first / totalDist) < 1.0){
				newProb = 1.0 - (iter->first / totalDist);
			}
			else{
				newProb = 1.0;
			}
			totalProb = totalProb + newProb;
			finalDataMap.insert( make_pair( newProb,  iter->second));
			//std::cout << "temp data = " <<   (iter->first) << ", " << (iter->second).x << ", " << (iter->second).y << std::endl;
		}
		//std::cout << "totalProb = " << 	totalProb << std::endl;
	
		for( iter = finalDataMap.begin(); iter != finalDataMap.end(); iter++ ) {
			float newProb = (iter->first / totalProb);
			finalProbMap.insert( make_pair( newProb,  iter->second));
			//std::cout << "new prob, data = " << newProb << ", " << (iter->second).x << ", " << (iter->second).y << std::endl;
			
		}
		
	
		/// now from the final map compute the final measurement
		for( iter = finalProbMap.begin(); iter != finalProbMap.end(); iter++ ) {
			finalData->x = (finalData->x) + iter->first * ((iter->second).x);
			finalData->y = (finalData->y) + iter->first * ((iter->second).y);
			finalData->z = (finalData->z) + iter->first * ((iter->second).z);
			if (!hueFound){
				finalData->hue = (finalData->hue) + iter->first * ((iter->second).hue);
			}
			else{
				finalData->hue = newHue;
				
			}
			//finalData->weight = (finalData->weight) + iter->first * ((iter->second).weight);
			finalData->weight = target->weight;
			done = true;
		}
	
		/// set the new validation gate for the target using the current prediction and the data inside the validation gate
		

		
		if (done){
			//std::cout << "final data = " << finalData->x << ", " << finalData->y << std::endl;
			target->kalmanEstimateState(finalData->x, finalData->y, finalData->z, finalData->hue, finalData->weight);
			target->notAssociated = false;
			target->timer.setActiveTime();
			
		}
		else{
			if (allDataVector->size() > 0){
				target->notAssociated = true;
			}
			else{
				target->notAssociated = false;
			}
		}
		delete finalData;

		
		target->updateRGB();
		target->updateTotalWeight();
		
	} /// for all targets

	
}

bool DataAssociator::clearNeighbourHood(TargetObject* target, DataElement3D* curData){
	bool returnBool = true;
	double MIN_INTER_TARGET_DIST;
	if (curData->sensorID == 0){
		MIN_INTER_TARGET_DIST = MIN_INTER_TARGET_DIST_CAM;
	}
	else if (curData->sensorID == 1){
		MIN_INTER_TARGET_DIST = MIN_INTER_TARGET_DIST_FLOOR;
	}
	else{
		MIN_INTER_TARGET_DIST = MIN_INTER_TARGET_DIST_GAZER;
	}

	vector<TargetObject*>::iterator tarIter;
	TargetObject* curTarget;
	for (tarIter = targetVector->begin();tarIter != targetVector->end();tarIter++){
		curTarget = *tarIter;
		if (curTarget != target){
			float curDist = sqrt((curTarget->x - curData->x)*(curTarget->x - curData->x)+(curTarget->y - curData->y)*(curTarget->y - curData->y));

			if (curDist < MIN_INTER_TARGET_DIST){
				returnBool = false;
				break;
			}
		}

	}

	return returnBool;
}

bool DataAssociator::insideValidationGate(TargetObject* target, DataElement3D data, float scaleOne, float scaleTwo, float minAxis, float maxAxis, float predictionFactor){
	bool returnBool = false;
	float predX, predY, predZ;
	float aa;

	/// if the target was not associated to data in the last step do not trust the prediction (because we grew the validation gate the prediction will be too much)
	if (target->lostTrack){
		aa = 0.1;
	}
	else{
		aa = 0.0;
	}
	target->lostTrack = false;

	predX = aa * cvmGet(target->kalman->state_pre,0,0) + (1.0 - aa) * target->x;
	predY = aa * cvmGet(target->kalman->state_pre,1,0) + (1.0 - aa) * target->y;
	predZ = aa * cvmGet(target->kalman->state_pre,2,0) + (1.0 - aa) * target->z;
	if ((predX <= 0.0) || (predY <= 0.0)){
		predX = target->x;
		predY = target->y;
		predZ = target->z;
	}

	

	/// necessary for plotting (fuserthread.cpp)
	target->predX = predX;
	target->predY = predY;
	target->predZ = predZ;

	CvPoint pt1;
	CvPoint pt2;
	pt1.x = target->x;
	pt1.y = target->y;
	//pt1.z = target->z;	
	pt2.x = pt1.x + (predX - pt1.x) * predictionFactor;
	pt2.y = pt1.y + (predY - pt1.y) * predictionFactor;
	//pt2.z = pt1.z + (predY - pt1.z) * predictionFactor;

	//std::cout << "prediction = " << predX << ", " << predY << std::endl;
	
	//double xaxis = abs(pt1.x - pt2.x);// + scaleTwo;
	//double yaxis = abs(pt1.y - pt2.y);// + scaleTwo;
	double xaxis;
	double yaxis;

 	xaxis = exp( fabs(pt1.x - pt2.x) + scaleOne);
	float NARROWNESS = 0.1;
 	yaxis = fabs(pt1.y - pt2.y) -  (log(xaxis) * NARROWNESS)  + scaleTwo;
// 
 	if (xaxis < minAxis){xaxis = minAxis;}
 	if (yaxis < minAxis){yaxis = minAxis;}
// 	
// 	// make sure xaxis is the bigger one
	if (xaxis < yaxis){
 		double tmp = xaxis;
 		xaxis = yaxis;
 		yaxis = tmp;
 	}
// 	
 	//std::cout << "1...xaxis = " << xaxis << ", yaxis = " << yaxis << std::endl;
// 	
// 		
// 	
 	if (xaxis > maxAxis){
 		xaxis = maxAxis;
 	}
 	if (yaxis > maxAxis){
 		yaxis = maxAxis;
 	}
// 	}

	/// now see if the target was not associated to data
	if ((target->notAssociated) ){
		//std::cout << "lost track..growing" << std::endl;
		target->xaxis = target->xaxis + 50.0;
		target->yaxis = target->xaxis + 50.0;
		if ((target->xaxis < MAX_GROWTH_SIZE_VAL_GATE) && (target->yaxis < MAX_GROWTH_SIZE_VAL_GATE)){
			xaxis = target->xaxis;
			yaxis = target->yaxis;

			target->lostTrack = true;
		}
		else{
			target->xaxis = MAX_GROWTH_SIZE_VAL_GATE;
			target->yaxis = MAX_GROWTH_SIZE_VAL_GATE;
			xaxis = target->xaxis;
			yaxis = target->yaxis;

			target->lostTrack = true;
		}
		
	}
	else{
		/// this will be used by fuserthread to plot
		//std::cout << "on track" << std::endl;
		target->xaxis = xaxis;
		target->yaxis = yaxis;
	}

	

	//std::cout << "xaxis, yaxis,  = " << xaxis << ", " << yaxis << ", " << target->growth_factor << std::endl;
	
	

	

	double angleRadian = atan2((pt2.y - pt1.y), (pt2.x - pt1.x) );
	
	double a = 3.14;
	double angle = (((angleRadian + a)*360./(2.*a))) - 180.;
	
	
	if (angle < 0.){angle = angle * -1.;}
	else {angle = 360. - angle;}
	
	
	
	// compute foci
	double c;
	c = sqrt( xaxis*xaxis - yaxis*yaxis);
	
	double b = 2* 3.14;
	double angleRadian2 = (angle / 360.)*b;
	
	double deltax1 = c * cos(angleRadian2);
	double deltay1 = c * sin(angleRadian2);

	CvPoint brennP1;
	CvPoint brennP2;
	if ((angle > 0.) && (angle < 90.0)){
		brennP1.x = pt2.x + deltax1; 
		brennP1.y = pt2.y - deltay1; 
		brennP2.x = pt2.x - deltax1; 
		brennP2.y = pt2.y + deltay1; 
	}	
	else if ((angle > 90.) && (angle < 180.0)){
		brennP1.x = pt2.x - deltax1; 
		brennP1.y = pt2.y + deltay1; 
		brennP2.x = pt2.x + deltax1; 
		brennP2.y = pt2.y - deltay1; 
	}
	else if ((angle > 180.) && (angle < 270.0)){
		brennP1.x = pt2.x - deltax1; 
		brennP1.y = pt2.y + deltay1; 
		brennP2.x = pt2.x + deltax1; 
		brennP2.y = pt2.y - deltay1; 
	}
	else {
		brennP1.x = pt2.x + deltax1; 
		brennP1.y = pt2.y - deltay1; 
		brennP2.x = pt2.x - deltax1; 
		brennP2.y = pt2.y + deltay1; 
	}


	// check if point inside ellipse
	double curD1 = sqrt((data.x - brennP1.x)*(data.x - brennP1.x) + (data.y - brennP1.y)*(data.y - brennP1.y));
	double curD2 = sqrt((data.x - brennP2.x)*(data.x - brennP2.x) + (data.y - brennP2.y)*(data.y - brennP2.y));

	if ( (curD1 + curD2) <= 2.*xaxis){

		returnBool = true;		
	}

	return returnBool;
}
