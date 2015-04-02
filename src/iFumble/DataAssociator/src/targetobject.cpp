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
#include "targetobject.h"

TargetObject::TargetObject(int idd, std::string namee, float xx, float yy, float zz, float vxx, float vyy, float vzz, int r, int g, int b, int inacTime):
	id(idd),
	name(namee),
	x(xx),
	y(yy),
	z (zz),
	vx(vxx),
	vz(vzz),
	vy(vyy),
	red(r),
	green(g),
	blue(b),
	//associated_to_cam_data(false),
	//associated_to_any_data(false),
	hue(NOF_HUEBINS/2.0),
	weight(-1),
	hueDataAvailable(false),
	currentHueValue(1.0),
	minMahaDist(999.),
	attentionalPriority(20. + ((float)rand()/(float)RAND_MAX)),
	sendx(xx),
	sendy(yy),
	sendz(zz),
	tapx(-0.5),
	tapy(0.0)
{
	weightReset = false;
	curWeight = 0.0;
	prevWeight = 0.0;

	computeSpecialID();

	//tempTimer.setActiveTime(); // this is testing timer..delete later
	//newHue = false;
	
	// set timer
	timer.setActiveTime();
	//std::std::cout << "targets previousActive time at creation = " << timer->previousActiveTime << std::std::endl;

	// initialize measurement matrtimer.setActiveTime();ix
	measurement = cvCreateMat( MES_SIZE, 1, CV_32FC1 );

	// initialize measurement_prediction matrix
	measurement_pred = cvCreateMat( MES_SIZE, 1, CV_32FC1 );

	// temporary matrices
	combined_innov = cvCreateMat( MES_SIZE, 1, CV_32FC1 );
	tempo1 = cvCreateMat( MES_SIZE, 1, CV_32FC1 );
	tempo2 = cvCreateMat( KALMAN_SIZE, 1, CV_32FC1 );
	tempoTrans1 = cvCreateMat( 1, MES_SIZE, CV_32FC1 );
	tempCov1 = cvCreateMat(MES_SIZE,KALMAN_SIZE,CV_32FC1);		
	tempCov2 = cvCreateMat(KALMAN_SIZE,MES_SIZE,CV_32FC1);		
	tempCov3 = cvCreateMat(MES_SIZE,MES_SIZE,CV_32FC1);
	tempCov4 = cvCreateMat(KALMAN_SIZE,KALMAN_SIZE,CV_32FC1);
	tempCov5 = cvCreateMat(KALMAN_SIZE,KALMAN_SIZE,CV_32FC1);
	tempCov6 = cvCreateMat(MES_SIZE,MES_SIZE,CV_32FC1);
	combInnovCov = cvCreateMat(MES_SIZE,MES_SIZE,CV_32FC1);

	/// extra error
	extra_error = cvCreateMat(KALMAN_SIZE,KALMAN_SIZE,CV_32FC1);

	// JPDA event probabilities
	event_probs = new std::vector<float>;

	all_innovs = new std::vector<CvMat*>; 


	all_measurements = new std::vector<high_dim_data>;
	// CvKalman* cvCreateKalman( int dynam_params, int measure_params, int control_params=0 );
	kalman  = cvCreateKalman( KALMAN_SIZE, MES_SIZE, CONTROL_SIZE );    // state is x,y,vx,vy and control vector is ax,ay (=acceleration)
	
	// measurement is x,y,z

	//std::cout << "kalman state size = " << kalman->DP << std::endl; 
	//std::cout << "kalman mes size = " << kalman->MP << std::endl;

	// set the current state
	CvMat* M = kalman->state_post;
	int   step  = M->step/sizeof(float);
	float *data = M->data.fl;

	(data+0*step)[0] = xx;
	(data+1*step)[0] = yy;
	(data+2*step)[0] = zz;
	(data+3*step)[0] = vxx;
	(data+4*step)[0] = vyy;
	(data+5*step)[0] = vzz;
	(data+6*step)[0] = 0.0; // acceleration
	(data+7*step)[0] = 0.0;
	(data+8*step)[0] = hue; // hue
	(data+9*step)[0] = weight; // weight




	/// set the prediction to the same as x, y (for validation gate)
	cvmSet(kalman->state_pre,0,0, 300.);
	cvmSet(kalman->state_pre,1,0, 300.);
	notAssociated = false;
	lostTrack = false;

	//cvSetIdentity( extra_error, cvRealScalar(60) ); /// 3500 until april 2008

	// initialize the Kalman vectors and matrices
	//const float A[] = { 1.0, 0.0, 0.0, 0.0, 1.0 ,0.0,0.0,0.0,1.0 }; // transition matrix
// 	const float A[] = { 	1.0, 0.0, 1.0, 0.0, 1.0, 0.0 ,
// 			    	0.0, 1.0, 0.0, 1.0, 0.0, 1.0,
// 				0.0, 0.0, 1.0, 0.0, 1.0, 0.0,
// 				0.0, 0.0, 0.0, 1.0, 0.0, 1.0,
// 				0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
// 				0.0, 0.0, 0.0, 0.0, 0.0, 1.0 }; // transition matrix

	//                      X,   Y,   Z,   VX,  VY,  VZ,  AX,  AY,  AZ,  HUE  WEI

	const float A[] = { 	1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
			    	0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 1.0,
				}; // transition matrix 

	
	memcpy( kalman->transition_matrix->data.fl, A, sizeof(A));
	//printMatrix(kalman->transition_matrix, "transition matrix");
	///std::cout << "ok 1" << std::endl;
	// H = measurement matrix 
	//const float MES[] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,
	//		      0.0, 1.0, 0.0, 0.0, 0.0, 0.0 }; //measurement matrix

	const float MES[] = { 
	1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0
 				}; //


	memcpy( kalman->measurement_matrix->data.fl, MES, sizeof(MES));
        //printMatrix(kalman->measurement_matrix, "measurement matrix");

	

	/// process noice covariance matrix 

	const float B[] = {     0.001, 0.0, 0.0, 0.001, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
			    	0.0, 0.001, 0.0, 0.0, 0.001, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.001, 0.0, 0.0, 0.001, 0.0, 0.0, 0.001, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.001, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.001	 
				}; 

	///memcpy( kalman->process_noise_cov->data.fl, B, sizeof(B));
  	
	cvSetIdentity( kalman->process_noise_cov, cvRealScalar(5) );
        cvSetIdentity( kalman->measurement_noise_cov, cvRealScalar(1) );

	/// meaurement noice covariance
	///cvSetIdentity( kalman->measurement_noise_cov, cvRealScalar(1.0));
       
	
	/// estimation error covariance (P-Schlange)
	cvSetIdentity( kalman->error_cov_post, cvRealScalar(1.0)); 
	estimationAdvanced = false;

	/// testing
	//std::cout << "target created: id, x,y,hue,weight = " << id << ", " << x << ", " << y << ", " << hue << ", " << weight << std::endl;

	/// for computation of the validation gate
	xaxis = 100.;
	yaxis = 100.;
	growth_factor = 0.1;

	
}



TargetObject::~TargetObject()
{	
	cvReleaseKalman(&kalman);


	resetJPDA();
	cvReleaseMat(&measurement_pred);
	cvReleaseMat(&tempo1);
	cvReleaseMat(&tempo2);
	cvReleaseMat(&tempoTrans1);
	cvReleaseMat(&tempCov1);
	cvReleaseMat(&tempCov2);
	cvReleaseMat(&tempCov3);
	cvReleaseMat(&tempCov4);
	cvReleaseMat(&tempCov5);
	cvReleaseMat(&tempCov6);
	cvReleaseMat(&combInnovCov);
	cvReleaseMat(&combined_innov);
	

	event_probs->clear();
	delete event_probs;
}

void TargetObject::computeSpecialID(){
	// should set QString special_id;
	time_t seconds = time (NULL);
	 int iPID = int(getpid());
	const char *pcOS = "L";
	std::stringstream oss;
	srand( time(NULL) );
	int randNr = rand();
	oss << pcOS << "-" << iPID << "-" << int(seconds) + id << "-" << randNr;
	special_id = oss.str();
	//std::cout << "special ID = " << special_id  << std::endl;
}

void TargetObject::resetJPDA(){

  //associated_to_any_data = false;

	/// reset the covariance matrix if the trace is too high
	///printMatrix(kalman->error_cov_post, "cov");
	CvSize size = cvGetSize( kalman->error_cov_post );
	int rows = size.height;
	int cols = size.width; 
	float totalSum = 0.0;
	if (rows == cols){ // check for square matrix 

		float *invdata = kalman->error_cov_post->data.fl;
		int invcovn = kalman->error_cov_post->cols;
		
		
		for (int i = 0; i < rows; i ++){
			for (int j =0; j < cols; j++){
			if (i==j){totalSum = totalSum + (invdata[i * invcovn + j] * invdata[i * invcovn + j]);}
			}
		} 
	}
	
	//std::cout << "trace norm: " << name << ": " << sqrt(totalSum) << std::endl; 
	if (sqrt(totalSum) > 1000.0){ // trace norm too big
		// rest the covariance to identity
		///cvSetIdentity( kalman->error_cov_post, cvRealScalar(1.0));

		/// meaurement noice covariance
		///cvSetIdentity( kalman->measurement_noise_cov, cvRealScalar(1.0));
       
	}


	/// ---------- end resetting covariance

	//printMatrix(kalman->error_cov_post, "cov");
	//std:: cout << "in reset JPDA " << std:: endl;

	/// update the RGB value from the new hue value
	if (hueDataAvailable){
	  int hueVal = hue ; //360. * hue / NOF_HUEBINS;
		//std::cout << "setting rgb from hue: " << hue << ", " << hueVal << std::endl;
		NOF_HUEBINS;
		HSV2RGB(hueVal, red, green, blue);

		//std::cout << "set rgb to:  " << red << ", " << green << ", " << blue << std::endl;
		
	}
	hueDataAvailable = false;
	
	event_probs->clear();
	//std:: cout << "in reset JPDA 1 " << std:: endl;
	cvSet(combined_innov,cvScalarAll(0),0);
 	//std:: cout << "all_innovs size =  " << all_innovs->size() << std:: endl;
	// reset all innovs
	for (int i = 0; i < all_innovs->size(); i++){
		cvReleaseMat(&(all_innovs->at(i)));
	}
	//std:: cout << "in reset JPDA 3 " << std:: endl;
	all_innovs->clear();

	all_measurements->clear();
	//std:: cout << "exiting reset JPDA...all_innovs size = " << all_innovs->size() << std:: endl;
	//associated_to_cam_data = false;
}

// stores this event for the target
void TargetObject::storeJPDAevent(float xx, float yy, float zz,  float huee, float weightt, float prob){
	
	//std::cout << "in store JPDA event: xx, yy, prob = " << xx << ", " << yy << ", " << huee << ", " << weightt << ", " << prob <<  std::endl;
	
	high_dim_data newData;
	newData.x = xx;
	newData.y = yy;
	newData.z = zz;

	

	if (!hueDataAvailable){
		newData.hue = hue;
		
	}
	else{

		if ((abs(currentHueValue - hue) < 4.5) || (hue <= 0.0)){
			newData.hue = currentHueValue;
			// reduce attentional prioriy
			attentionalPriority = attentionalPriority / 2.0; //0. +
				// ((float)rand()/(float)RAND_MAX);
			if (attentionalPriority < 1.0){attentionalPriority = ((float)rand()/(float)RAND_MAX); }
		}
		else{
			std::cout << "------------------------> hue diff = " << abs(currentHueValue - hue) << ", hue = " << hue <<  std::endl;
			newData.hue = hue + (currentHueValue - hue)/2.0;
			
			hueDataAvailable = false;
		}
		
	}
	newData.weight = weightt;
	all_measurements->push_back(newData);
	
	
	event_probs->push_back(prob);

	/// compute the innovation
	CvMat* tempo3 = cvCreateMat( MES_SIZE, 1, CV_32FC1 );
	cvmSet( tempo1, 0, 0, xx);
	cvmSet( tempo1, 1, 0, yy);
	cvmSet( tempo1, 2, 0, zz);
	cvmSet( tempo1, 3, 0, huee);
	cvmSet( tempo1, 4, 0, weightt);

	cvmSet( measurement_pred, 0, 0,  cvmGet(kalman->state_pre,0,0));
	cvmSet( measurement_pred, 1, 0,  cvmGet(kalman->state_pre,1,0));
	cvmSet( measurement_pred, 2, 0,  cvmGet(kalman->state_pre,6,0));
	cvmSet( measurement_pred, 3, 0,  cvmGet(kalman->state_pre,7,0));

	cvSub(tempo1, measurement_pred, tempo3, 0);
	
	///printMatrix(tempo1, "target tempo 1: measurement");
	

	///printMatrix(kalman->state_pre, "kalman->state_pre");

	/// now scale this with the prob 
	///cvConvertScale( tempo3, tempo3, prob, 0 );
	///printMatrix(tempo3, "temppo 3 innovation");

	/// insert this innovation into all_innovs
	all_innovs->push_back(tempo3);

	/// update the combined innovation
	//std::cout << "------------------------> prob = " << prob << std::endl;
	CvScalar prob_scal = cvScalarAll(prob);
// 	std::cout << "targobj: before cvScaleAdd" << std::endl;
// 	std::cout << "tempo3: " << tempo3->rows << "," << tempo3->cols << std::endl;
// 	
// 	std::cout << "combined_innov: " << combined_innov->rows << "," << combined_innov->cols << std::endl;
// 	std::cout << "tempo1: " << tempo1-> rows << "," << tempo1->cols << std::endl;

	cvScaleAdd( tempo3, prob_scal, combined_innov, tempo1);
	//std::cout << "targobj: after cvScaleAdd" << std::endl;
	cvCopy(tempo1, combined_innov, 0);	

	///printMatrix(combined_innov, "combined_innov...1");
}


void TargetObject::updateKalmanState(CvMat* trans_mat, float xxxx, float yyyy, float zzzz){
	cvCopy(trans_mat, kalman->transition_matrix,0);
	///printMatrix(kalman->transition_matrix, "transition matrix");

	///std::cout << "measurement = " <<  xxxx << ", " << yyyy << std::endl;
	kalmanEstimateState(xxxx, yyyy, zzzz, hue, weight);
	///printMatrix(kalman->gain, "kalman->gain");
	///printMatrix(kalman->state_post, "state_post");
	
}


void TargetObject::updateKalmanStateWithProb(float xxxx, float yyyy, float zzzz, float prob){
	
  kalmanEstimateState(x, y, z, hue, weight);

	CvScalar prob_scal = cvScalarAll(prob);
	cvScaleAdd( kalman->state_post, prob_scal, 0, kalman->state_post);
	cvScaleAdd( kalman->error_cov_post, prob_scal, 0,kalman->error_cov_post);
		
}

void TargetObject::updateJPDAstateNew(CvMat* trans_mat){

	///if (associated_to_any_data){

		//std::cout << "targ ID: " << id << ", x = " << x << ", y = " << y << ", hue = " << hue << ", weight = " << weight << std::endl;
		///printMatrix(kalman->error_cov_post, "error cov post");
		/// now see if hueDataAvailable (if yes change the hue value of all data)
		if (hueDataAvailable){
			//std::cout << "################################################################## current hue value = " << currentHueValue << std::endl;
			for (int i = 0; i < event_probs->size(); i ++){
	
				/// only if current hue pos is close enough
				double distToHuePos = sqrt(pow(all_measurements->at(i).x - currentHuePosX,2)+pow(all_measurements->at(i).y - currentHuePosY,2));
		
				if (distToHuePos < MAX_DIST_TO_HUE_POS){
					(all_measurements->at(i)).hue = currentHueValue;
				}
				
			}
		}
	
	
		///printMatrix(kalman->state_pre, "state pre");
		//printMatrix(trans_mat, "trans_mat");	
	
		/// set the transition matrix
		
		cvCopy(trans_mat, kalman->transition_matrix,0);
	
		high_dim_data final_measurement;
		final_measurement.x = 0;
		final_measurement.y = 0;
		final_measurement.z = 0;
		final_measurement.hue = 0;
		final_measurement.weight = 0;	
		bool donne = false;
	
		float cur_mes_x;
		float cur_mes_y;
		float cur_mes_z;
		float cur_mes_hue;
		float cur_mes_weight;
		//std::cout << "-----------------------------number of associations: " << 	event_probs->size() << std::endl;

		
		
			for (int i = 0; i < event_probs->size(); i ++){
				donne = true;
				float curProb = event_probs->at(i);
		
				cur_mes_x = (all_measurements->at(i)).x;
				cur_mes_y = (all_measurements->at(i)).y;
				cur_mes_z = (all_measurements->at(i)).z;
				cur_mes_hue = (all_measurements->at(i)).hue;
				cur_mes_weight = (all_measurements->at(i)).weight;
	
				
				//std::cout << "target " << id << ", mes_x, mes_y, mes_hue, mes_weight, prob = " << cur_mes_x << ", " << cur_mes_y << ", " << cur_mes_hue << ", " << cur_mes_weight << ", " << curProb  << std::endl;
	
			
				final_measurement.x = final_measurement.x + (curProb * cur_mes_x );
				final_measurement.y = final_measurement.y + (curProb * cur_mes_y);
				final_measurement.z = final_measurement.z + (curProb * cur_mes_z);
				final_measurement.hue = final_measurement.hue + (curProb * cur_mes_hue );
				final_measurement.weight = final_measurement.weight + (curProb * cur_mes_weight );
			}
		
			//std::cout << "final_mes = " << final_measurement.x << ", " << final_measurement.y << std::endl;
		
			if (donne){
		
				//std::cout << "before, state_pre: " << cvmGet(kalman->state_pre,0,0) << ", " << cvmGet(kalman->state_pre,1,0) << ", " << cvmGet(kalman->state_pre,2,0) << ", " << cvmGet(kalman->state_pre,3,0) << std::endl;
		
				kalmanEstimateState(final_measurement.x, final_measurement.y,final_measurement.z, final_measurement.hue, final_measurement.weight);
				//std::cout << "++++++++++++ updateJPDAstateNew: target " << id << ", final mes: " << final_measurement.x << ", " << final_measurement.y << ", " << final_measurement.hue << ", " <<  final_measurement.weight << std::endl << "after state_pre: " << cvmGet(kalman->state_pre,0,0) << ", " << cvmGet(kalman->state_pre,1,0) << ", " << cvmGet(kalman->state_pre,6,0) << ", " << cvmGet(kalman->state_pre,7,0) << std::endl;
		// 		
		// 		
			}
			///printMatrix(kalman->state_post, "state post");
		
	
		// clear all measurements
		all_measurements->clear();

	///} /// if associated_to_any_data
	
}




void TargetObject::updateJPDAstate(CvMat* trans_mat){

	/// set the transition matrix
	
	cvCopy(trans_mat, kalman->transition_matrix,0);
	///printMatrix(kalman->transition_matrix, "transition matrix");
	//cvCopy(control_mat, kalman->control_matrix,0);
	

	/// as in Shashtry paper


	//std::cout <<  " shastry ok 1 " << std::endl;

	
	/// compute the event probs total sum
	float allProbSum = 0;
	//std::cout << "Nr of events = " << event_probs->size() << std::endl;
	for (int i = 0; i < event_probs->size(); i ++){
		
		//std::cout << "event Nr = " << i << " prob = " << event_probs->at(i) << std::endl;
		allProbSum = allProbSum + event_probs->at(i);

		
	}

if (allProbSum > 0){	

	//std::cout <<  " ok 2 " << std::endl;combined_innov
	
	///printMatrix(kalman->measurement_noise_cov, "mes-noice-cov");
	///printMatrix(kalman->error_cov_pre, "error-cov-pre");

	/// compute the covariance of combined innovation
	//std::cout << "targobj pos 1" << std::endl;
	//printMatrix(kalman->error_cov_pre, "error cov pre");
	
	cvMatMul(kalman->measurement_matrix,kalman->error_cov_pre, tempCov1);
	//std::cout << "targobj pos 2" << std::endl;
	cvTranspose(kalman->measurement_matrix, tempCov2);
	//std::cout << "targobj pos 3" << std::endl;
	cvMatMul(tempCov1, tempCov2, tempCov3);
	//std::cout << "targobj pos 4" << std::endl;
	cvAdd(kalman->measurement_noise_cov, tempCov3, combInnovCov);
	//std::cout << "targobj pos 5" << std::endl;
	///printMatrix(combInnovCov, "comb innov cov");

	/// compute the combined Kalman Gain
	cvTranspose(kalman->measurement_matrix, tempCov2);
	///printMatrix(tempCov2, "tempCov2 1");
	///printMatrix(kalman->error_cov_post, "kalman->error_cov_post");
	//std::cout << "targobj pos 5.5" << std::endl;
	cvMatMul(kalman->error_cov_post, tempCov2, tempCov2);
	///printMatrix(tempCov2, "tempCov2 2");
	//std::cout << "targobj pos 6" << std::endl;
	cvInvert(combInnovCov, tempCov3, CV_LU);
	///printMatrix(tempCov3, "tempCov3");
	//std::cout << "targobj pos 7" << std::endl;
	cvMatMul(tempCov2, tempCov3, kalman->gain);
	//std::cout << "targobj pos 8" << std::endl;
	///printMatrix(kalman->gain, "comb kalman gain");
	
	///---------------- upate the state estimate-------------------------	
	
	///printMatrix(combined_innov, "targ: update state: combined_innov");
	
	cvMatMulAdd( kalman->gain, combined_innov, 0, tempo2 );
	//std::cout <<  " ok 4.1 " << std::endl;
	cvAdd( tempo2, kalman->state_post, tempo2, 0 );
	//std::cout <<  " ok 4.2 " << std::endl;
	cvCopy(tempo2, kalman->state_post, 0);
	///printMatrix(kalman->state_post, "kalman->state_post");
	//std::cout <<  " ok 5 " << std::endl;


	/// --------------- compute the state estimation covariance  ---------
	cvTranspose(kalman->gain,tempCov1 );
	//std::cout <<  " ok 5.1 " << std::endl;
	cvMatMul(kalman->gain, combInnovCov, tempCov2);

	//std::cout <<  " ok 5.2 " << std::endl;
	cvMatMul(tempCov2, tempCov1, tempCov4);
	//std::cout <<  " ok 5.3 " << std::endl;
	
	cvSet(tempCov5,cvScalarAll(0),0);
	//std::cout <<  " ok 5.4 " << std::endl;
	CvScalar prob_scal = cvScalarAll(allProbSum);
	//std::cout <<  " targobj: ok 5.5 " << std::endl;
	cvScaleAdd( tempCov4, prob_scal, tempCov5, tempCov5);
	//std::cout <<  " targobj: ok 5.6 " << std::endl;
	cvSub(kalman->error_cov_post, tempCov5, tempCov4, 0);
	// so until now tempCov4 has the result
	
	//std::cout <<  " ok 6 " << std::endl;

	// now compute the middle bracket
	
	cvSet(tempCov2,cvScalarAll(0),0);
	cvSet(tempCov3,cvScalarAll(0),0); // just temp
	cvSet(tempCov6,cvScalarAll(0),0); 
	//std::cout <<  " ok 6.1..events_prob size =  " << event_probs->size() << std::endl;

	for (int i = 0; i < event_probs->size(); i ++){
		//std::cout <<  " ok 6.n2 " << std::endl;
		CvScalar tmpSca = cvScalarAll(event_probs->at(i));
		//std::cout <<  " ok 6.n3 " << std::endl;
		cvTranspose(all_innovs->at(i), tempoTrans1);
		//std::cout <<  " ok 6.n4 " << std::endl;
		cvMatMul(all_innovs->at(i), tempoTrans1, tempCov3);
		//std::cout <<  " ok 6.n5 " << std::endl;
		cvScaleAdd( tempCov3, tmpSca, tempCov6, tempCov6);
		//std::cout <<  " ok 6.n6 " << std::endl;
		//cvCopy(tempCov1, tempCov2);
		//std::cout <<  " ok 6.n7 " << std::endl;
	}
	/// tempCov6 has the result (which is the sum in the middle bracket)
	//std::cout <<  " ok 6.2 " << std::endl;
	cvTranspose(combined_innov, tempoTrans1);
	//std::cout <<  " ok 6.3 " << std::endl;
	cvMatMul(combined_innov, tempoTrans1, tempCov3);
	//std::cout <<  " ok 6.4 " << std::endl;
	cvSub(tempCov6, tempCov3, tempCov6);
	//std::cout <<  " ok 7 " << std::endl;

	// the last sum in the equation in paper
	cvTranspose(kalman->gain, tempCov1);
	cvMatMul(kalman->gain, tempCov6, tempCov2);
	cvMatMul(tempCov2, tempCov1, tempCov5);

	//std::cout <<  " ok 8 " << std::endl;
	// now add with tempCov4
	cvAdd( tempCov5, tempCov4, kalman->error_cov_post, 0 ); 
	//std::cout <<  " ok 9 " << std::endl;


	

	//printMatrix(kalman->gain, "gain");
	
	//printMatrix(combInnovCov, "combInnovCov");

	//printMatrix(kalman->error_cov_post, "estimate cov ");


	//printMatrix(kalman->state_post, "kalman->state_post");
	
	/// set x,y,z
	CvMat* Me = kalman->state_post;
	int   stepMe  = Me->step/sizeof(float);
	float *dataMe = Me->data.fl;

	

	/// save old state
	xOld = x;
	yOld = y;
	zOld = z;
	vxOld = vx;
	vyOld = vy;
	vzOld = vz;
	hueOld = hue;
	weightOld = weight;

	x = (dataMe+0*stepMe)[0];
	y = (dataMe+1*stepMe)[0];
	z = (dataMe+2*stepMe)[0];
	vx = (dataMe+3*stepMe)[0];
	vy = (dataMe+4*stepMe)[0];
	vz = (dataMe+5*stepMe)[0];
	hue = (dataMe+9*stepMe)[0];
	weight = (dataMe+10*stepMe)[0];

	if (isnan(x)){
		x = xOld;
	}
	if (isnan(y)){
		y = yOld;
	}
	if (isnan(z)){
		z = zOld;
	}
	if (isnan(vx)){
		vx = vxOld;
	}
	if (isnan(vy)){
		vy = vyOld;
	}
	if (isnan(vz)){
		vz = vzOld;
	}
		
	if (isnan(hue)){
		std::cout << "hue is nan in updateJPDAstate" << std::endl;
		hue = hueOld;
	}
	if (isnan(weight)){
		std::cout << "weight is nan in updateJPDAstate" << std::endl;
		weight = weightOld;
	}
	/// ------------------------------------------------------------------

	
	timer.setActiveTime();
	
	/// error_cov_post is increased manually (24 July 2007 ZM)
	
	//cvAdd( kalman->error_cov_post, extra_error , kalman->error_cov_post);
	
	//std::cout << "x,y,vx,vy = " << x << ", " << y << ", " << vx << ", " << vy << std::endl;
	
	
}
	
	
}	

void TargetObject::printNormTraceMatrix(CvMat* mat,  QString name){

	CvSize size = cvGetSize( mat );
	int rows = size.height;
	int cols = size.width; 
	float totalSum = 0.0;
	if (rows == cols){ // check for square matrix 

		float *invdata = mat->data.fl;
		int invcovn = mat->cols;
		
		
		for (int i = 0; i < rows; i ++){
			for (int j =0; j < cols; j++){
			if (i==j){totalSum = totalSum + (invdata[i * invcovn + j] * invdata[i * invcovn + j]);}
			}
		} 
	}
	
	std::cout << "trace norm: " << name << ": " << sqrt(totalSum) << std::endl; 
}

// prints out a CvMat given the pointer to its data and a string
void TargetObject::printMatrix(CvMat* mat,  QString name){

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


// do state prediction 
void TargetObject::kalmanPredictState(){

	
	

	// compute the measurement prediction
	///printMatrix(kalman->measurement_matrix, "kalman mes matrix");
	
	

	///cvMatMulAdd(kalman->measurement_matrix, kalman->state_pre,0, measurement_pred);

	///printMatrix(measurement_pred, "mes prediction");


	cvKalmanPredict( kalman, 0 );
	//printMatrix(kalman->state_pre, "state prediction");
	//printNormTraceMatrix(kalman->error_cov_post, QString("id:%1 trace-norm post error covariance").arg(id));
	
}


void TargetObject::setTapY(double y){
  tapy = 0.9 * tapy + 0.1 *y;
}

// do the Kalman estimation..given the measurement (x,y) 
void TargetObject::kalmanEstimateState(float xx, float yy, float zz, float huee, float weightt){

	/// attention: make sure that the covariance min and max are kept (otherwise we run into numerical instability problems)

	//std::cout << " --------------kalmanEstimateState with " << xx << ", " << yy << ", " << huee << ", " << weightt << std::endl;
	/// set timer 
	//timer.setActiveTime();	
	
	// save old state
	xOld = x;
	yOld = y;
	zOld = z;
	vxOld = vx;
	vyOld = vy;
	vzOld = vz;
	hueOld = hue;
	weightOld = weight;


	// pointer to the state data
	CvMat* M = kalman->state_post;
	int   step  = M->step/sizeof(float);
	float *data = M->data.fl;


	// measurement 
	 /* x,y is measured */
	// pointer to the measurement data
	float* mesdata = measurement->data.fl;
	int   stepm  = measurement->step/sizeof(float);
	(mesdata+0*stepm)[0] = xx;
	(mesdata+1*stepm)[0] = yy;
	(mesdata+2*stepm)[0] = zz;
	(mesdata+3*stepm)[0] = huee;
	(mesdata+4*stepm)[0] = weightt;

	cvKalmanCorrect( kalman, measurement );
	
	float* postdata = kalman->state_post->data.fl;
	x = (postdata+0*step)[0];
	y = (postdata+1*step)[0];
	z = (postdata+2*step)[0];
	vx = (postdata+3*step)[0];
	vy = (postdata+4*step)[0];
	vz = (postdata+5*step)[0];
	hue = (postdata+9*step)[0];
	weight = (postdata+10*step)[0];


	if (isnan(x)){
		x = xOld;
		std::cout << "kalmanUpdateState: x is nan" << std::endl;
	}
	if (isnan(y)){
		y = yOld;
		std::cout << "kalmanUpdateState: y is nan" << std::endl;
	}
	if (isnan(z)){
		z = zOld;
		std::cout << "kalmanUpdateState: z is nan" << std::endl;
	}
	if (isnan(vx)){
		vx = vxOld;
		std::cout << "kalmanUpdateState: vx is nan" << std::endl;
	}
	if (isnan(vy)){
		vy = vyOld;
		std::cout << "kalmanUpdateState: vy is nan" << std::endl;
	}
	if (isnan(vz)){
		vz = vzOld;
		std::cout << "kalmanUpdateState: vz is nan" << std::endl;
	}
	
	if (isnan(hue)){
		hue = hueOld;
		std::cout << "kalmanUpdateState: hue is nan" << std::endl;
	}
	if (isnan(weight)){
		std::cout << "kalmanUpdateState: weight is nan" << std::endl;
		weight = weightOld;
	}

	//std::cout << "id: " << id << ", " << x << ", " << y <<  ", " << hue << ", " << weight << std::endl; 
	//std::cout << "id = " << id << ", norm cov = " << cvNorm(kalman->error_cov_post) << std::endl;
	double curNorm = cvNorm(kalman->error_cov_post);
	if (curNorm > 17.0){
		estimationAdvanced = true;
	}


	// 22.0 and 17.0 before
	if ((curNorm > 22.0 ) || ((estimationAdvanced) && (curNorm < 18.0)))
	{

		estimationAdvanced = false;
		//std::cout << std::endl << std::endl << "************************************ resetting  " << std::endl << std::endl; 

		/// set the new x and y to the data
		// set the current state
		CvMat* M = kalman->state_post;
		int   step  = M->step/sizeof(float);
		float *data = M->data.fl;

		(data+0*step)[0] = xx;
		(data+1*step)[0] = yy;
		(data+2*step)[0] = zz;
		(data+3*step)[0] = zz;
		(data+4*step)[0] = 1.0;
		(data+5*step)[0] = 1.0;
		(data+6*step)[0] = 1.0;
		(data+7*step)[0] = 1.0; 
		(data+8*step)[0] = 1.0;
		(data+9*step)[0] = huee; 
		(data+10*step)[0] = weightt; 


		cvSetIdentity( kalman->error_cov_post, cvRealScalar(1.0));

		cvCopy(kalman->state_post,kalman->state_pre );

		cvSetIdentity( kalman->process_noise_cov, cvRealScalar(5) );
        	cvSetIdentity( kalman->measurement_noise_cov, cvRealScalar(1) );
		

		/// for computation of the validation gate
		xaxis = 100.;
		yaxis = 100.;
		growth_factor = 0.1;

			
	}



	/// testing ID from hues
// 	if ((hue < 12.0) && (newHue) && (id ==0)){
// 		std::cout << "recovery green = " << tempTimer.getInactiveTime() << std::endl;
// 		 tempTimer.setActiveTime();
// 		newHue = false;
// 	}
// 	if ((hue > 15.0) && (newHue) && (id ==1)){
// 		std::cout << "recovery red = " << tempTimer.getInactiveTime() << std::endl;
// 		 tempTimer.setActiveTime();
// 		newHue = false;
// 	}

}

void TargetObject::addToWeightData(DataElement3D data){
	weightDataVector.push_back(data);
}
void TargetObject::clearWeightData(){
	weightDataVector.clear();
}


void TargetObject::updateRGB(){
  //std::cout << "before : hue = " << hue << ", red = " << red << ", green = " << green << ", blue = " << blue << std::endl;
  int hueVal = hue; //360. * hue / NOF_HUEBINS;
	HSV2RGB(hueVal, red, green, blue);
	//std::cout << " after : hue = " << hue << ", red = " << red << ", green = " << green << ", blue = " << blue << std::endl;
}

void TargetObject::resetWeight(){
	if (!weightReset){
		prevWeight = weight;
		curWeight = 0.0;
		weightReset = true;
	}
}
void TargetObject::addWeight(double w){
	curWeight += w;
}
void TargetObject::updateTotalWeight(){
	if (curWeight > prevWeight){ /// update to higher weight
		weight = 0.9*prevWeight + 0.1*curWeight;
	}
	else{	///decay
		weight = weight * 0.9990;
	}
	weightReset = false;
}


 void TargetObject::HSV2RGB(int hueVal, int &red, int &green, int &blue)  // Point conversion
 {
       	/// done considering: en.wikipedia.org/wiki/Image:HSV-RGB-comparison.svg


	if (hueVal <= 60){
		red = 255;
		green = (int)((255./60.)*(float)(hueVal));
		blue = 0;
	}	
	else if (hueVal <= 120 ){
		red = (int)(-(255./60.)*(float)hueVal + (255. * 2.));
		green = 255;
		blue = 0;
	}
	else if (hueVal <= 180){
		red = 0;
		green = 255;
		blue = (int)((255./60.)*(float)(hueVal) - (255. * 2));
	}
	else if (hueVal <= 240 ){
		red = 0;
		green = (int)(-(255./60.)*(float)hueVal + (255. * 4.));
		blue = 255;
	}
	else if (hueVal <= 300 ){
		red = (int)((255./60.)*(float)hueVal - (255. * 4));
		green = 0;
		blue = 255;
	}
	else if (hueVal <= 360 ){
		red = 255;
		green = 0;
		blue = (int)(-(255./60.)*(float)hueVal + (255. * 6.));
	}

	//std::cout << "hue, r,g,b = " << hueVal << ", " << red << ", " << green << ", " << ", " << blue << std::endl;
} 

