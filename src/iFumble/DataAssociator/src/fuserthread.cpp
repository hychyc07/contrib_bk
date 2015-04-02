
#include "fuserthread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;


/// define callback function or mouse click updating world model
void mouseClicked(int event, int x, int y, int flags , void* param  );
TargetCreationDeletion* targCreationCopy;
vector<TargetObject*>* targetVectorCopy;

// constructor
//FuserThread::FuserThread(QObject* frameObjectt,  Frame* frameC)
FuserThread::FuserThread(int period)
        : RateThread(period) /*QThread(),*/
	/*run_flag(true)*/

{	
	
 
	
}

bool FuserThread::threadInit(){

	inPortLeft.open("/iFumble/traza/left/in");
	inPortRight.open("/iFumble/traza/right/in");
	inPortEyeToWorld.open("/iFumble/traza/eye2world/in");

	inPortHue.open("/iFumble/traza/hue/in");

	sendPort.open("/iFumble/traza/3dpos/out");
	sendPortFumble.open("/iFumble/iCStuff/ObjectOut");
	

       floorImage = cvCreateImage(cvSize(X_LENGTH,Y_LENGTH ),IPL_DEPTH_8U,3);
  	displayImage = cvCreateImage(cvSize(X_LENGTH ,Y_LENGTH ),IPL_DEPTH_8U,3);
	scaledImage = cvCreateImage(cvSize(X_LENGTH ,Y_LENGTH ),IPL_DEPTH_8U,3);

	//frameObjectCopy = frameObject; // for editing world model
	//frameCopy = frameC;
	
	imageTimeFile.open("imagetime.txt");
	
     logFile.open("logfile.txt");			
     logData = false;
     logImage = false;
	
     

      /// initialize data vectors for targets and data
     
     targetVector = new vector<TargetObject*>;
	
	targCreationDeletion = new TargetCreationDeletion();

	targetVectorCopy = targetVector; // copy pointer for mouse click handling
	targCreationCopy = targCreationDeletion; // copy pointer for mouse click handling



  	//targPosSendThread = new TargetPositionSenderThread();
  	

	//targPosSendThread->addClientPosFan("192.168.1.11", 42345); // gazer hue extrac
	//targPosSendThread->addClientPosFan("192.168.1.30",4252);  // torque server
	//targPosSendThread->addClientPosFan("192.168.1.33",4252);  // torque server
	

	
/** -------------------------------------------------------------------------- */
	// here somehow initialize he sensors..now sing yarp (before it was udp)
	
	

/** -------------------------------------------------------------------------- */


    /// initialize and set the target vector for the data associator	
    dataAssociator = new DataAssociator(targetVector, targCreationDeletion);	
    ///dataAssociator->setTargetVector(targetVector);





	cout << "hello" << endl;
	//floorImage = cvLoadImage("icub-robot.jpg", 1);  
	cvZero(floorImage);
	// draw lines for coordinate system
	// draw a green line of width 1 between (100,100) and (200,200)
	cvLine(floorImage, cvPoint(ROBOT_X,0), cvPoint(ROBOT_X, Y_LENGTH), cvScalar(0,255,0), 1);
	cvLine(floorImage, cvPoint(0, ROBOT_Y), cvPoint(X_LENGTH, ROBOT_Y), cvScalar(0,255,0), 1);

	
	//cvNamedWindow("traza", CV_WINDOW_AUTOSIZE); 
	cvNamedWindow("traza", 1); 
	
	/// Mouse click updating world model
	cvSetMouseCallback("traza", mouseClicked, this);

	//cout << "finished cvNamedWIndow" << endl;
	cvShowImage("traza", displayImage );

	cout << "initialized opencv window...." << endl;

}

void FuserThread::threadRelease(){
  cvDestroyWindow( "traza" );
}

void FuserThread::stop( )
{
    // set flag to request thread to exit
    // REMEMBER: The thread needs to be woken up once
    // after calling this method to actually exit!
    run_flag = false;
    ///cvReleaseImage(&displayImage);

    cvReleaseImage(&scaledImage);
    logFile.close();
	
	


}

void FuserThread::destroyDisplay(){
	//std::cout << "destroying display..." << std::endl;
	//targetDisplay = false;
	//cvDestroyWindow("TraZa");
	cvReleaseImage(&displayImage);
        cvDestroyWindow("traza");
	
}


void FuserThread::createDisplay(){

    
}

void mouseClicked(int event, int x, int y, int flags , void* param  )
{

	//std::cout << "mouse clicked" << std::endl;
 	switch( event )
    	{
		case CV_EVENT_LBUTTONDOWN:
		{
			
			//std::cout << "inserting target" << std::endl;

						
			targCreationCopy->targVecMutex.lock();
			int nextID = targCreationCopy->nextID;
			srand ( time(NULL) );
			int red = 255;//rand() % 255 + 1;
			int green = 255;//rand() % 255 + 1;
			int blue = 255;//rand() % 255 + 1;
			std::stringstream ss;
			ss << "Target " << nextID;
		
		
			targCreationCopy->addTarget( targetVectorCopy, nextID, ss.str(), x,y,0, red, green, blue);
			targCreationCopy->nextID++;


			targCreationCopy->targVecMutex.unlock();
			//}
			
			
			//targMutex.unlock();
		
		}
		break;
		case CV_EVENT_RBUTTONDOWN:
		{
			//std::cout << "deleting target" << std::endl;
		  if (targetVectorCopy->size() > 0){
			
		    ((TargetObject*)targetVectorCopy->at(0))->x = x;
		    ((TargetObject*)targetVectorCopy->at(0))->y = y;

		  }
		
		}
		break;
			case CV_EVENT_MBUTTONDOWN:
		{
			//std::cout << "deleting target" << std::endl;
			targCreationCopy->targVecMutex.lock();
			
			targCreationCopy->removeClosestTarget(targetVectorCopy, x, y);
			targCreationCopy->targVecMutex.unlock();
		
		}
	}
}




void FuserThread::estimate3DsensoriData(){
	
	for (int i = 0; i < leftDataVec.size(); i++){
		TwoDdata curLeftData = leftDataVec.at(i);
		if (curLeftData.f1 == 1){ //i.e. if a human
			for (int j = 0; j < rightDataVec.size(); j++){
				TwoDdata curRightData = rightDataVec.at(j);

				double xDist = curLeftData.x - curRightData.x;
				double yDist = curLeftData.y - curRightData.y;
				double dist = sqrt(xDist * xDist + yDist * yDist);
			
				if (dist < 10.0){

					// ask iKinGazeCtrl to look at this position in space
					//Vector& attOut = gazeCommandPort.prepare();
        
       					//attOut.resize(4);
        				//attOut[0] = curLeftData.x;
        				//attOut[1] = curLeftData.y;
        				//attOut[2] = curRightData.x;
        				//attOut[3] = curRightData.y;
	

				
					//gazeCommandPort.write();
					gazingTowardsObject = true;

				
				}

			}
		}
		//else if (curLeftData.f1 == 0){  // if recognized as a ball
			// use eyeToWorld to get 3D pos using just one cam

		//	Bottle& output = eye2WorldOut.prepare();
            	//	output.clear();
            	//	output.addDouble(curLeftData.x);
		//	output.addDouble(curLeftData.y);
		//	eye2WorldOut.write();
		//}
	}
}

bool FuserThread::gazingConverged(){
	bool retBool = false;


	if (retBool) {
		gazingTowardsObject = true;
	}
	return retBool;
}

void FuserThread::transformOperational2Display(double tx, double ty, double tz, double &dx, double &dy, double &dz){
	
  dx = ROBOT_X +  ty * 1000; // 0.1 m translate to 100 pixels
  dy = ROBOT_Y + tx * 200; 
  dz = tz;	

	cout << "oper: " << tx << ", " << ty << ", " << tz << ", disp: " << dx << ", " << dy << ", " << dz << endl;
}


void FuserThread::transformDisplay2Operational(double dx, double dy, double dz, double &tx, double &ty, double &tz){
  tx = (dy -  ROBOT_Y)/200;
  ty = (dx - ROBOT_X)/1000;
	tz = dz; 

}

void FuserThread::run( )
{

	// clear data
	dataAssociator->allDataVector->clear();
	leftDataVec.clear();
	rightDataVec.clear();

	// format of arriving 2D data: x1 y1 f1 g1 x2 y2 f2 g2 ....
	// x and y are 2D positions, f ang g are feature values
	Bottle* inBotLeftVec = inPortLeft.read(false);
	if ((inBotLeftVec!= NULL)){
		int j = 0;
		TwoDdata curData;
		while (j < inBotLeftVec->size()-5){
			
				curData.x = (inBotLeftVec->get(j + 0)).asDouble();
				curData.y = (inBotLeftVec->get(j + 1)).asDouble();
				curData.f1 = (inBotLeftVec->get(j + 2)).asDouble();
				curData.f2 = (inBotLeftVec->get(j + 3)).asDouble();
					
				leftDataVec.push_back(curData);

				j = j + 4;
		}
	}

	Bottle* inBotRightVec = inPortRight.read(false);
	if ((inBotRightVec!= NULL)){
		int j = 0;
		TwoDdata curData;
		while (j < inBotRightVec->size()-5){
			
				curData.x = (inBotRightVec->get(j + 0)).asDouble();
				curData.y = (inBotRightVec->get(j + 1)).asDouble();
				curData.f1 = (inBotRightVec->get(j + 2)).asDouble();
				curData.f2 = (inBotRightVec->get(j + 3)).asDouble();
					
				rightDataVec.push_back(curData);

				j = j + 4;
		}
	}
	
	

	// now get all the 3D data from this
	if ((rightDataVec.size() > 0) && (leftDataVec.size() > 0)){
		cout << "estimating 3d data" << endl;
		estimate3DsensoriData();
		
	}	

	// check eye to world to see if things found on table
	//cout << "pppp" << endl;
	Bottle* inBotEye2World = inPortEyeToWorld.read(false);
	if ((inBotEye2World!= NULL)){
		cout << "received eye to world data: " <<  (inBotEye2World->get(0)).asDouble() << ", " << (inBotEye2World->get(1)).asDouble() << ", " << (inBotEye2World->get(2)).asDouble() << endl;

		double curx, cury,curz;
		double tx = (inBotEye2World->get(0)).asDouble();
		double ty = (inBotEye2World->get(1)).asDouble();
		double tz = (inBotEye2World->get(2)).asDouble();
	
		transformOperational2Display(tx, ty, tz, curx, cury, curz);
		cout << "data elem = " << curx << ", " << cury << ", " << curz << endl;
		DataElement3D curData(0, curx, cury, curz, 0, 1); 
		(dataAssociator->allDataVector)->push_back(&curData);
	}
	//cout << "tttt" << endl;
	//targPosSendThread->start();
	//targPosSendThread->wait();


	// get hue directly from blob detector or from TPC
	Bottle* inBotHue = inPortHue.read(false);
	if ((inBotHue!= NULL)){
	  double huee = (inBotHue->get(0)).asDouble();
	  if (targetVector->size() > 0){
	    ((TargetObject*)targetVector->at(0))->hue = huee * 360.0;
	    //((TargetObject*)targetVector->at(0))->hue = huee * 360;
	 
	    //cout << "hue for target received: " << huee << ", target hue: " << ((TargetObject*)targetVector->at(0))->hue << endl; 
	  }
	}


	// send 3D position data....to test for tapping
	if (targetVector->size() > 0){
	 Bottle& bot = sendPort.prepare();
         bot.clear();
	 bot.addString("tap");
	 double sendX, sendY, sendZ; 
	 transformDisplay2Operational(((TargetObject*)targetVector->at(0))->tapx, ((TargetObject*)targetVector->at(0))->tapy, ((TargetObject*)targetVector->at(0))->z, sendX, sendY, sendZ);
	 
	  bot.addDouble(sendX);
	  bot.addDouble(sendY);
	  bot.addDouble(sendZ);
	  sendPort.write();


	  // send to fumble
	  Bottle& bott = sendPortFumble.prepare();
	  bott.clear();
	  if (((TargetObject*)targetVector->at(0))->hue > 0.5){
	     bott.addString("ball");
	  }
	  else{
	    bott.addString("piece");
	  }
	 double sendXX, sendYY, sendZZ; 
	 transformDisplay2Operational(((TargetObject*)targetVector->at(0))->x, ((TargetObject*)targetVector->at(0))->y, ((TargetObject*)targetVector->at(0))->z, sendXX, sendYY, sendZZ);
	 
	  bott.addDouble(sendXX);
	  bott.addDouble(sendYY);
	  bott.addDouble(sendZZ);
	  sendPortFumble.write();
	 }
	
	//cout << "fuserthread: going into loop" << endl;
	
	//sleep(1);
	double cur_frame_rate;
	bool slept = false;
	int not_slept = 1;

	int image_index = 0;

	if (logImage){
		image_timer.setActiveTime();
	}
	
	

      //cout << "running" << endl;
	///  put real frame rate on the GUI
	double inac_time_before = speed_timer.getInactiveTime(); // seconds
	pre_frame_rate = cur_frame_rate;
	cur_frame_rate = (1.0 / inac_time_before);

	/*
	if (slept){
	    std::cout << "not_slept = " << not_slept << std::endl;
	    frame_rate = cur_frame_rate * not_slept;
	    slept = false;
	    not_slept  =1;
	}
	else{
		not_slept++;
	}*/

	frame_rate = (min(cur_frame_rate, APP_SPEED) + min(pre_frame_rate, APP_SPEED)) / 2.0;
	

	/// first set speed_timer
	speed_timer.setActiveTime();
	

	// log data
	if (logData){writeLogData();}

	// send target postions to position clients
	
	///targPosSendThread->sendAllTargetsPosToAll(*targetVector);
	///targCreationDeletion->clusterAllData(dataAssociator->allDataVector);

	///targCreationDeletion->setFloorClusterTotalNrPeople(dataAssociator->allDataVector);
	//targCreationDeletion->tryDeleteTargetVote(dataAssociator->allDataVector, targetVector);
	targCreationDeletion->tryDeleteTargetNew(dataAssociator->allDataVector, targetVector);
	//cout << "fuserthread: going to call create target " << endl;
	targCreationDeletion->tryCreateNewTarget(dataAssociator->allDataVector, targetVector);
	//targCreationDeletion->tryCreateNewTargetVote(dataAssociator->allDataVector, targetVector);
	
	dataAssociator->runJPDA();
	
	/// update display
	updateTargetDisplay();

	/// get processing time
        double proc_time = speed_timer.getInactiveTime(); // seconds
	
// 	if (APP_SPEED  < cur_frame_rate  ){
// 		double diff_time =  (1.0 / (double) APP_SPEED) - proc_time;
// 		///std::cout << "diff_time = " <<  diff_time   << std::endl;
// 		if (diff_time > 0.0){
// 			///std::cout << "sleeping" << std::endl;
// 	    		usleep(diff_time * 1000000.0);
// 			slept = true;
// 			
// 		}
// 		
// 	}
	
	if ((logImage) && (displayImage)){		
		writeDisplayImage(image_index);
		image_index++;
	}
	
	if( cvWaitKey(10) >= 0 ){}// break;

        if( !cvGetWindowHandle("traza") ) {std::exit(0);}
	
	//

}

void FuserThread::updateTargetDisplay(){

	//cvSet(displayImage, cvScalar(0));
	//cout << "before cvCopy.." << endl;
	cvResize(floorImage, displayImage, NULL);
	

	/// show targets
	vector<TargetObject*>::iterator targIter; 
	TargetObject* targObj;
	float targX, targY, targZ;
	for( targIter = targetVector->begin(); targIter != targetVector->end(); targIter++){ 
		targObj = *targIter;
		targX = targObj->x;
		targY = targObj->y;
		//targZ = targObj->z; 

			//displayTarget(targX, targY, 0, 60, 0, 0,  targObj->red,  targObj->green,  targObj->blue, QString("%1").arg(targObj->hue));
			//displayTarget(targX, targY, 0, 60, 0, 0,  targObj->red,  targObj->green,  targObj->blue, QString("%1/%1").arg(targObj->id).arg(targObj->hue));
			displayTarget(targX, targY, 0, 60, 0, 0,  targObj->red,  targObj->green,  targObj->blue, QString("%1,%1").arg(targObj->id).arg((int)targObj->weight));
			#ifdef DRAW_VAL_GATE
				//drawValidationGate(targObj);
				drawPredictedTrajectory(targObj);
				//drawValidationGate(targObj);
			#endif
	
	}

	/// show data

		// now display the data 
	/** vector containing data from ALL sensors */
	vector<DataElement3D*>* allDataVector = dataAssociator->allDataVector;

	
	int nofData = 0;
	DataElement3D* curData; 
	if (allDataVector != NULL){
		//cout << "kuku.." << endl;
		if (!(allDataVector->empty())){
			//cout << " allDataVector not empty..." << endl;
			nofData = allDataVector->size();
		}
		else{
			nofData = 0;
			
		}
		
		for (int row = 0; row < nofData; row ++){
			
			curData = allDataVector->at(row);
	//		
			// draw the data point 
			///if (createdDisplay){
				if (curData->sensorID == 0){
				cvEllipse( displayImage, cvPoint(curData->x,curData->y), cvSize(3, 3), 
				0, 0, 360, CV_RGB(255, 255, 255), 0, 1, 0);
				}
				else if (curData->sensorID == 1){
				cvEllipse( displayImage, cvPoint(curData->x,curData->y), cvSize(3, 3), 
				0, 0, 360, CV_RGB(255, 0, 0), CV_FILLED, 1, 0);

				}
				else if (curData->sensorID == 3){
				
				cvEllipse( displayImage, cvPoint(curData->x,curData->y), cvSize(4, 4), 
				0, 0, 360, CV_RGB(0, 255, 0), CV_FILLED, 1, 0);

				}
				
			///}
			//cvEllipse( targetImage, cvPoint(curData->x,curData->y), cvSize(5, 5), 
			//0, 0, 360, CV_RGB(255, 255, 255), CV_FILLED, 1, 0);

		}
		
	}

	/// show frame rate
	CvFont font;
  	cvInitFont(&font,CV_FONT_VECTOR0,0.5,0.5,0.0,0.8);
    	cvPutText( displayImage, QString("%1").arg((int)frame_rate ), cvPoint( 10, 50), &font,CV_RGB(255, 255, 255));

	int totalCamNrPeople = targCreationDeletion->camTotalNrPeople;
	int totalNrPeople = targCreationDeletion->floorTotalNrPeople;
	int totalClusters = targCreationDeletion->floorClusterTotalNrPeople;
	int voteTargetNr = targCreationDeletion->voteTargetNr;

	//cvPutText( displayImage, QString("%1,%1,%1,%1").arg(totalCamNrPeople).arg(totalNrPeople).arg(totalClusters).arg(voteTargetNr), cvPoint( X_LENGTH-70, Y_LENGTH-40), &font,CV_RGB(255, 255, 255));

	///------------------------------------------------------//

	if (displayImage != NULL){
		cvvShowImage("traza", displayImage);	
	}
	//cout << "finished target display" << endl;

}

void FuserThread::drawPredictedTrajectory(TargetObject* target){

	bool showFloor = false;
	double scaleOne;
	double scaleTwo;
	double minAxis;
	double maxAxis;
	double predFactor;

	if (target != NULL){
		
	  scaleOne = 2.0; 
	  scaleTwo = 3.0;
	  minAxis = 2.0 ;
	  maxAxis = 3.0; 
	  predFactor = 5.0;
		
		//cvKalmanPredict(  target->kalman, 0 );
		float predX, predY;
		predX = target->predX;//cvmGet(target->kalman->state_pre,0,0);
		predY = target->predY;//cvmGet(target->kalman->state_pre,1,0);

		/// draw predicted point
		cvEllipse( displayImage, cvPoint(cvmGet(target->kalman->state_pre,0,0), cvmGet(target->kalman->state_pre,1,0)), cvSize(5, 5), 
					0, 0, 360, CV_RGB(100, 255, 255), CV_FILLED, 1, 0);
	

		// draw a line through current state and prediction
		int x =  target->x;
		int y =  target->y;
		int predXX = (predX + (x - predX)*20);
		int predYY = (predY + (y - predY)*20);
		
		cvLine(displayImage, cvPoint(predX,predY), cvPoint(predXX,predYY), cvScalar(0,180,150), 1);
		

		// set where the robot should tap (ideally table border)
		float slope = (predYY - predY)/(predXX - predX);
		float offset = predY - slope * predX;
	       
		// if moving rightwards
		if (predXX > predX + 10.0){
		  target->tapx =  TAP_RIGHT_X;
		  target->setTapY(slope * TAP_RIGHT_X + offset);
		  //		  target->tapy = slope * TAP_RIGHT_X + offset;
		  
		  
		}
		// or move left
		else if (predX > predXX + 10.0){
		  target->setTapY(slope * TAP_LEFT_X + offset);
		  //target->tapy = slope * TAP_LEFT_X + offset;
		  target->tapx = TAP_LEFT_X;
		  
		}
		//target->tapx = predXX;
		//target->tapy = predYY;

		cvEllipse( displayImage, cvPoint(target->tapx,target->tapy), cvSize(5, 5), 
					0, 0, 360, CV_RGB(255, 100, 255), CV_FILLED, 1, 0);

	}

}



void FuserThread::drawPrediction(TargetObject* target){

	bool showFloor = false;
	double scaleOne;
	double scaleTwo;
	double minAxis;
	double maxAxis;
	double predFactor;

	if (target != NULL){
		
	  scaleOne = 2.0; 
	  scaleTwo = 3.0;
	  minAxis = 2.0 ;
	  maxAxis = 3.0; 
	  predFactor = 5.0;
		
		//cvKalmanPredict(  target->kalman, 0 );
		float predX, predY;
		predX = target->predX;//cvmGet(target->kalman->state_pre,0,0);
		predY = target->predY;//cvmGet(target->kalman->state_pre,1,0);

		/// draw predicted point
		cvEllipse( displayImage, cvPoint(cvmGet(target->kalman->state_pre,0,0), cvmGet(target->kalman->state_pre,1,0)), cvSize(5, 5), 
					0, 0, 360, CV_RGB(100, 255, 255), CV_FILLED, 1, 0);
	
		//std::cout <<"out 2" << std::endl;
		CvPoint pt1;
		CvPoint pt2;
		
		pt1.x = target->x;
		pt1.y = target->y;
		pt2.x = pt1.x + (predX - pt1.x) * predFactor;
		pt2.y = pt1.y + (predY - pt1.y) * predFactor;

		
		double xaxis = target->xaxis;
		double yaxis = target->yaxis;
		///std::cout << "xaxis, yaxis = " << xaxis << ", " << yaxis << std::endl;
		
		
		//std::cout <<"out 3.0" << std::endl;
		double angleRadian = atan2((pt2.y - pt1.y), (pt2.x - pt1.x) );
		//std::cout <<"out 3.1" << std::endl;
		double a = 3.14;
		double angle = (((angleRadian + a)*360./(2.*a))) - 180.;
		
		
		if (angle < 0.){angle = angle * -1.;}
		else {angle = 360. - angle;}
		
		//std::cout <<"out 3.2" << std::endl;
		//std::cout << "xaxis, yaxis, angle = " << xaxis << ", " << yaxis << ", " << angle << std::endl;
		
		// brennpunkte bestimmen
		double c;
		c = sqrt( xaxis*xaxis - yaxis*yaxis);
		
		
		// now we have angles from 0..360
		// transform this into radians 0..2*PI
		double b = 2* 3.14;
		double angleRadian2 = (angle / 360.)*b;
		
		double deltax1 = c * cos(angleRadian2);
		double deltay1 = c * sin(angleRadian2);
		

		///std::cout << " px, py, angle, angleRadian2, deltax1, deltay1 " << pt2.x << ", " << pt2.y << ", " << angle  << ", " << angleRadian2 << ", " <<  deltax1 << ", " <<  deltay1 << std::endl;

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
		//std::cout << "brennP1 = " << brennP1.x << ", " << brennP1.y << std::endl;
		//std::cout << "brennP2 = " << brennP2.x << ", " << brennP2.y << std::endl;
		///if (xaxis < 5.){xaxis = 5.;}
		///if (yaxis < 5.){yaxis = 5.;}
		//std::cout <<"out 3.3" << std::endl;
		
		//std::cout <<"out 4" << std::endl;
		
		//std::cout << "xaxiss, yaxiss, anglee = " << xaxiss << ", " << yaxiss << ", " << anglee << std::endl;
		//std::cout <<"out 5" << std::endl;
		if ((pt2.x > 0) && (pt2.y > 0)){
			
			/// validation gate
			cvEllipse( displayImage, pt2, cvSize(xaxis , yaxis), angle, 0, 360, CV_RGB(100, 255, 255), 0, 1, 0);

			/// midpoint of ellipse
			//cvEllipse( displayImage, pt2, cvSize(3 , 3), 0, 0, 360, CV_RGB(100, 255, 255), CV_FILLED, 1, 0);

			/// brennpunkt 1
			//cvEllipse( displayImage, brennP1, cvSize(2, 2), 0, 0, 360, CV_RGB(255, 0, 0), 0, 1, 0);

			/// brennpunkt 2
			//cvEllipse( displayImage, brennP2, cvSize(2, 2), 0, 0, 360, CV_RGB(255, 0, 0), 0, 1, 0);
			
		}

		/// now draw all the points that fall inside the ellipse
		/*
		for (int i =0; i < X_LENGTH; i = i+12){
			for (int j =0; j < Y_LENGTH; j = j+12){
				// see if this point is inside the gate
				
				double curD1 = sqrt((i - brennP1.x)*(i - brennP1.x) + (j - brennP1.y)*(j - brennP1.y));
				double curD2 = sqrt((i - brennP2.x)*(i - brennP2.x) + (j - brennP2.y)*(j - brennP2.y));

				if ( (curD1 + curD2) <= 2.*xaxis){

					cvEllipse( displayImage, cvPoint(i,j), cvSize(2, 2), 
					0, 0, 360, CV_RGB(255, 0, 255), 1.0, 1, 0);
				}
			}
		}
		*/
	}
}

void FuserThread::drawValidationGate(TargetObject* target){
	
	//cvKalmanPredict(  target->kalman, 0 );

	///target->printMatrix(target->kalman->error_cov_pre, "error_cov_pre");
	//target->printMatrix(target->kalman->error_cov_post, "error_cov_post");

	CvMat* mes_prediction = cvCreateMat(MES_SIZE,1,CV_32FC1);
	cvmSet( mes_prediction, 0, 0,  cvmGet(target->kalman->state_pre,0,0));
	cvmSet( mes_prediction, 1, 0,  cvmGet(target->kalman->state_pre,1,0));
	cvmSet( mes_prediction, 2, 0,  cvmGet(target->kalman->state_pre,2,0));
	cvmSet( mes_prediction, 3, 0,  cvmGet(target->kalman->state_pre,3,0));


	CvMat* data_high_dim_pos = cvCreateMat(MES_SIZE,1,CV_32FC1);
	CvMat* innov = cvCreateMat(MES_SIZE,1,CV_32FC1);

	CvMat* temp1 = cvCreateMat(MES_SIZE,KALMAN_SIZE,CV_32FC1);		
	CvMat* temp2 = cvCreateMat(KALMAN_SIZE,MES_SIZE,CV_32FC1);		
	CvMat* temp3 = cvCreateMat(MES_SIZE,MES_SIZE,CV_32FC1);
	CvMat* innovCov = cvCreateMat(MES_SIZE,MES_SIZE,CV_32FC1);
	CvMat* invInnovCov = cvCreateMat(MES_SIZE,MES_SIZE,CV_32FC1);
	CvMat* innovTranspose = cvCreateMat(1,MES_SIZE,CV_32FC1);
	CvMat* innovTmp2 = cvCreateMat(MES_SIZE,1,CV_32FC1);
	CvMat* innovTmp = cvCreateMat(1,MES_SIZE,CV_32FC1);

	for (int i =0; i < X_LENGTH; i = i+12){
		for (int j =0; j < Y_LENGTH; j = j+12){
			cvmSet( data_high_dim_pos, 0, 0, i);
			cvmSet( data_high_dim_pos, 1, 0, j);	
			cvmSet( data_high_dim_pos, 2, 0, target->hue);
			cvmSet( data_high_dim_pos, 3, 0, target->weight);

			// compute innnovation
			cvSub(data_high_dim_pos, mes_prediction, innov);

			cvMatMul(target->kalman->measurement_matrix,target->kalman->error_cov_pre, temp1);
			cvTranspose(target->kalman->measurement_matrix, temp2);
			cvMatMul(temp1, temp2, temp3);
			cvAdd(target->kalman->measurement_noise_cov, temp3, innovCov);	
			
			//target->printMatrix(target->kalman->error_cov_pre, "error cov pre");
			//target->printMatrix(innovCov, "innovCov");

			cvInvert(innovCov, invInnovCov, CV_LU);
				
			
			

				
				
				//cout << "ok21" << endl;	
				//printMatrix(innov, "valid_mat: innov");
			cvTranspose(innov, innovTranspose);
				//cout << "ok22" << endl;			
			cvMatMul(innovTranspose, invInnovCov, innovTmp);
				//printMatrix(innovTmp, "valid_mat: innovTmp");
				//cout << "ok23" << endl;	
			cvTranspose(innovTmp, innovTmp2);
				
		
			//printMatrix(innov, "valid_mat: innov");
			//printMatrix(innovTmp2, "valid_mat: innovTmp2");
				//cout << "ok24" << endl;
			//std::cout << "rows, cols = " <<  innovTmp2->rows << ", " << innovTmp2->cols << ", " << innov->rows << ", " << innov->cols << std::endl;	
			
			float value = cvDotProduct(innov, innovTmp2);
			if (value < 0){value = value * -1;}
			if (value < 2.0){
				//cvEllipse( displayImage, cvPoint(i,j), cvSize(2, 2), 0, 0, 360, CV_RGB(0, 255, 0), 1.0, 1, 0);
			}
			if (value < 2.0){
				cvEllipse( displayImage, cvPoint(i,j), cvSize(2, 2), 
				0, 0, 360, CV_RGB(255, 0, 255), 1.0, 1, 0);
			}
		}
	}
	
}

// write display image
void FuserThread::writeDisplayImage(int index){
		// capturing jpegs
		//cout << "writing image ..............." << endl;
			
     				
    		imageTimeFile << index << " " <<  image_timer.getInactiveTime()  << endl;
		//imageTimeFile.close();
		
		
		sprintf(tempfilename, "../../ImageData/logImage%d.jpg", index);
		// now resize the displayImage to 388, 284 scaledImage
		cvResize(displayImage, scaledImage, NULL);
		cvSaveImage(tempfilename, scaledImage);
}


// write log data
void FuserThread::writeLogData(){


  
}

// display graphically the target 
void FuserThread::displayTarget(float x, float y, float angle, int gain, float sig_x, float sig_y, int rgb1, int rgb2, int rgb3, string name){

	if ((x > -10) && (x < 620) && (y > -10) && (y < 620)){
		///std::cout << "display target at: " << x << ", " << y << std::endl;
	
		// draw the ellipse	
		int thickness = 3;
		//cout << "rgb = " << rgb1 << ", " << rgb2 << ", " << rgb3 << endl;
		cvEllipse( displayImage, cvPoint(x,y), cvSize(10, 10), 
			angle, 0, 360, CV_RGB(rgb1, rgb2, rgb3), thickness, 1, 0);


		///cvEllipse( displayImage, cvPoint(x,y), cvSize(10, 10), 
		///	angle, 0, 360, CV_RGB(200, 10, 10), CV_FILLED, 1, 0);
		CvFont font;
  		//cvInitFont(&font,CV_FONT_VECTOR0,1.5,1.5,0.0,3.2);
		cvInitFont(&font,CV_FONT_VECTOR0,0.5,0.5,0.0,1.0);
    		cvPutText( displayImage, QString(name), cvPoint(x - 20, y - 10), &font,CV_RGB(255, 255, 255));
  		//char str[255] = name;
  	
  	
		/// Repercurso
		///cvPutText( displayImage, "Afrika", cvPoint(x - 20, y - 10), &font,CV_RGB(255, 255, 255));
		/**
		obey convention: gotten from sensors as:
		x: 0..............n
	
		y: 0
	   	.
	   	.
	   	.
	   	.
	   	n
	
		**/

	}





}




// adds a new target object
void FuserThread::addTarget( int id, std::string name, double x, double y, double z, double vx, double vy, double vz, int red, int green, int blue){
	///targWaitCond.wait();
	targetMutex.lock();
	cout << "targMutex locked from add target" << endl;
	
	TargetObject* target = new TargetObject(id, name, x, y,z, vx, vy, vz, red, green, blue, 0);
	targetVector->push_back(target);
	//freeIDs[targetVector->size()- 1] = false;
	cout << "--------------------------> added target" << endl;
	
	///targWaitCond.wakeAll();
	cout << "targcond wake from add target" << endl;
	targetMutex.unlock();
	cout << "targMutex unlocked from add target" << endl;
	
} 

// removes target object with given  id
void FuserThread::removeTarget( int id){

	///targWaitCond.wait();
	targetMutex.lock();
	vector<TargetObject*>::iterator targIter; 

	
	cout << "targMutex locked from remove target" << endl;
	int atPos = 0;

	for( targIter = targetVector->begin(); targIter != targetVector->end(); targIter++){ 	
	  
	  	if ( (*targIter)->id == id){

			cout << "going to erase id " << id << endl;
			targetVector->erase(targetVector->begin() + atPos);
			cout << "--------------------------> erased target" << endl;	
			///cout << "erased id: " << id << endl;
			//freeIDs[ii] = true;
			targIter = targetVector->end();
			break;
		
		}
		atPos++;
		
	  }

	///targWaitCond.wakeAll();	
	cout << "targcond wakeAll from remove target" << endl;
	targetMutex.unlock();
	cout << "targMutex unlocked from remove target" << endl;
	
} 	


