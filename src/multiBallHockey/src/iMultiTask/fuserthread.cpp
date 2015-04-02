
#include "fuserthread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;


/// define callback function or mouse click updating world model
void mouseClicked(int event, int x, int y, int flags , void* param  );
TargetCreationDeletion* targCreationCopy;
vector<TargetObject*>* targetVectorCopy;

double hitxold = 0.0;
double hitx = X_LENGTH / 2;;
double absVel;

int countNoPoints = 0;
 double maxWorldX = 5.0; // maximum minus x axis value (abs)
  double maxWorldY = 2.0;





// constructor
//FuserThread::FuserThread(QObject* frameObjectt,  Frame* frameC)
FuserThread::FuserThread(int period, vector<TaskThread*> taskV)
        : RateThread(period) /*QThread(),*/
	/*run_flag(true)*/

{	
  
	taskVec = taskV;  
}

bool FuserThread::threadInit(){

  
  debugPort.open("/traza/debug/out");
  inPortBlobs.open("/iMultiTask/blobs/in");
  
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
	

    /// initialize and set the target vector for the data associator	
    dataAssociator = new DataAssociator(targetVector, targCreationDeletion);	
    ///dataAssociator->setTargetVector(targetVector);





    //cout << "hello" << endl;
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

	//cout << "initialized opencv window...." << endl;

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
		  //if (targetVectorCopy->size() > 0){
			
		  //((TargetObject*)targetVectorCopy->at(0))->x = x;
		  //((TargetObject*)targetVectorCopy->at(0))->y = y;

		  //}

		  
		
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

  
  dx = (X_LENGTH/(maxWorldY*2)) * (ty + maxWorldY); //(Y_LENGTH/5)*(ty+5); //ROBOT_X +  ty * 1000; // 0.1 m translate to 100 pixels
  dy = (Y_LENGTH/maxWorldX) * (tx + maxWorldX);//ROBOT_Y + tx * 200; 

  dz = tz;	

  
}


void FuserThread::transformDisplay2Operational(double dx, double dy, double dz, double &tx, double &ty, double &tz){

  //dx = (X_LENGTH/(maxWorldY*2)) * (ty + maxWorldY); 
  ty = (dx/(X_LENGTH/(maxWorldY*2))) - maxWorldY; 		
  //dy = (Y_LENGTH/maxWorldX) * (tx + maxWorldX);//ROBOT_Y + tx * 200; 
  tx = (dy / (Y_LENGTH/maxWorldX) ) - maxWorldX;
  tz = dz;	

}

void FuserThread::run( )
{

//std::cout << "running fuser.." << std::endl;

  // clear data
  dataAssociator->allDataVector->clear();
  
  // check to see if things found on table
  //cout << "running fusethread..." << endl;
  Bottle* inBotBlobs = inPortBlobs.read(false);
  if ((inBotBlobs!= NULL)){
    
    int xy[6];
    xy[0] = -1;
    xy[1] = -1;
    xy[2] = -1;
    xy[3] = -1;
    xy[4] = -1;
    xy[5] = -1;
    //cout << "received blobs: ";
    for (int uu = 0; uu < inBotBlobs->size(); uu++){
      //cout << (inBotBlobs->get(uu)).asInt() << ", ";
      xy[uu] =  (inBotBlobs->get(uu)).asInt();
    }
    //cout << endl;
    
    
    yarp::sig::Vector xyworld1;
    xyworld1.resize(2);
    yarp::sig::Vector xyworld2;
    xyworld2.resize(2);
    yarp::sig::Vector plane;
    plane.resize(4);
    plane[0] = 0.0;
    plane[1] = 0.0;
    plane[2] = 1.0;
    plane[3] = 0.12;
    yarp::sig::Vector px2D1;
    px2D1.resize(2);
    int camsel = 0; // left camera   
	  
    // one 2d point
    if (xy[0] > -1){
      
      
      px2D1[0] = xy[0];
      px2D1[1] = xy[1];
      
      //cout << "from cam xy = " << xy[0] << ", " << xy[1] << endl;
      //fprintf(stdout, "compute 3d with: %.1f, %.1f", px2D1[0], px2D1[1]);
      bool done = ((HeadThread*)(taskVec.at(0)))->igaze->get3DPointOnPlane(camsel, px2D1, plane, xyworld1);
	//bool done = igaze->get3DPointOnPlane(camsel, px2D1, plane, xyworld1);
      
      
      DataElement3D curData(0,xyworld1[0] , xyworld1[1], xyworld1[2], 0, 1); 
      (dataAssociator->allDataVector)->push_back(&curData);

	(taskVec.at(0))->setNextTaskPosition(xyworld1); // head

	
	  }

	else{ // if no 2d point
		if (countNoPoints > 10){
			(taskVec.at(0))->reset(); // head
		}
		if (countNoPoints > 50){
			(taskVec.at(1))->reset(); // arms
			countNoPoints = 0;
		}
		countNoPoints++;

	}
	
	}

	
	
	
	double cur_frame_rate;
	bool slept = false;
	int not_slept = 1;

	int image_index = 0;

	
    
	///  put real frame rate on the GUI
	double inac_time_before = speed_timer.getInactiveTime(); // seconds
	pre_frame_rate = cur_frame_rate;
	cur_frame_rate = (1.0 / inac_time_before);

	
	frame_rate = (min(cur_frame_rate, APP_SPEED) + min(pre_frame_rate, APP_SPEED)) / 2.0;
	

	/// first set speed_timer
	speed_timer.setActiveTime();
	
	// try delete and create target
	targCreationDeletion->tryDeleteTargetNew(dataAssociator->allDataVector, targetVector);
	targCreationDeletion->tryCreateNewTarget(dataAssociator->allDataVector, targetVector);
	

	dataAssociator->runJPDA();
	
	/// update display
	updateTargetDisplay();

	/// get processing time
        double proc_time = speed_timer.getInactiveTime(); // seconds
	
	

	if( cvWaitKey(10) >= 0 ){}// break;
        if( !cvGetWindowHandle("traza") ) {std::exit(0);}
	
	

}

void FuserThread::updateTargetDisplay(){

	//cvSet(displayImage, cvScalar(0));
	//cout << "updating targets display..." << endl;
	cvResize(floorImage, displayImage, NULL);
	

	/// show targets
	vector<TargetObject*>::iterator targIter; 
	TargetObject* targObj;
	float targX, targY, targZ;
	absVel = 0.0;
	for( targIter = targetVector->begin(); targIter != targetVector->end(); targIter++){ 
		targObj = *targIter;
		targX = targObj->x;
		targY = targObj->y;
		//targZ = targObj->z; 

		
		//std::cout << "mean vel = " << meanVel << std::endl;
		//std::cout << "target velocity = " << velx << ", " << vely << ", abs = " << absVel << std::endl;

		double dispx, dispy, dispz;
		transformOperational2Display(targX, targY, 0.0, dispx, dispy, dispz);
		
	        displayTarget(dispx, dispy, 0, 60, 0, 0,  targObj->red,  targObj->green,  targObj->blue, QString("%1,%1").arg(targObj->id).arg((int)targObj->weight));
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

			double dispx, dispy, dispz;
			//dispx = curData->x;
			//dispy= curData->y;
			//dispz= curData->z;
			
			transformOperational2Display(curData->x, curData->y, 0.0, dispx, dispy, dispz);
			//fprintf(stdout, "data world coord: %.1f, %.1f, disp coord: %.1f, %.1f %\n", curData->x, curData->y, dispx, dispy);

			// draw the data point 
			///if (createdDisplay){
				if (curData->sensorID == 0){
				cvEllipse( displayImage, cvPoint(dispx,dispy), cvSize(3, 3), 
				0, 0, 360, CV_RGB(255, 255, 255), 0, 1, 0);
				}
				else if (curData->sensorID == 1){
				cvEllipse( displayImage, cvPoint(dispx,dispy), cvSize(3, 3), 
				0, 0, 360, CV_RGB(255, 0, 0), CV_FILLED, 1, 0);

				}
				else if (curData->sensorID == 3){
				
				cvEllipse( displayImage, cvPoint(dispx,dispy), cvSize(4, 4), 
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

		// transform to display
		double curxx = predX; //cvmGet(target->kalman->state_pre,0,0);
		double curyy = predY; //cvmGet(target->kalman->state_pre,1,0);
		double dispx, dispy, dispz;
				
		transformOperational2Display(curxx, curyy, 0.0, dispx, dispy, dispz);
	
		/// predicted point
		cvEllipse( displayImage,  cvPoint(dispx, dispy), cvSize(5, 5), 0, 0, 360, CV_RGB(100, 255, 255), CV_FILLED, 1, 0);
	

		// draw a line through current state and prediction
		double curx =  target->x;
		double cury =  target->y;
		
		double dispxx, dispyy, dispzz;
		transformOperational2Display(curx, cury, 0.0, dispxx, dispyy, dispzz);
		

		cvLine(displayImage, cvPoint(dispxx,dispyy), cvPoint(dispx,dispy), cvScalar(0,180,150), 1);
		
 
		// compute to which side the line is going
		// compute inclination
		double incli = (dispy - dispyy)/(dispx - dispxx);
		//std::cout << "inclination = " << incli << std::endl;
		double offset = dispy - incli * dispx;
		
		// see where it hits the border of robot side
		double tmo = (Y_LENGTH - offset)/incli;
		if ((tmo > 0.0 && tmo < X_LENGTH) && (target->meanSpeed > 0.01)){ 
		  hitx = 0.9 * hitxold + 0.1 * (Y_LENGTH - offset)/incli;
		  hitxold = hitx;
		}
		///std::cout << "mean speed x, y = " << target->meanSpeedx << ", " << target->meanSpeedy << std::endl;
		if ((target->meanSpeedx > 0.001) && (curx > -2.5) && (curx < -0.2) && (cury > -1.0) && (cury < 1.0)){ // if speed high enough

			// get the root frame 3d point
			double tapx, tapy, tapz;
			//transformDisplay2Operational(hitx, Y_LENGTH - 50, 0.0, tapx, tapy, tapz);

			tapx = -0.2;
			tapy = cury;
			tapz = 0.1;
			//std::cout << "hit at: " << tapx << ", " << tapy << ", " << tapz << std::endl;
				// put some boundary conditions
				//if (tapy < -0.4) {tapy = -0.4; }
				//if (tapy > 0.4) {tapy = 0.4; }

				//std::cout << "hit at: " << tapx << ", " << tapy << ", " << tapz << std::endl;
				
			// debug output
			Bottle bot;  
  			bool done = false;
 			 	bot.addDouble(tapx);
     				bot.addDouble(tapy);	
				bot.addDouble(tapz); 
    				debugPort.write(bot);

			yarp::sig::Vector tapxy;
    			tapxy.resize(3);
			tapxy[0] = tapx;
			tapxy[1] = tapy;
			tapxy[2] = tapz;
			(taskVec.at(1))->setNextTaskPosition(tapxy); // arms

			// display the hitting point in green and talk to the cartesian
			if (hitx > (X_LENGTH/2)){
				cvEllipse( displayImage,  cvPoint(hitx, Y_LENGTH - 50), cvSize(30, 30), 0, 0, 360, CV_RGB(0, 255, 20), 0, 1, 0);
			}
			else{
				cvEllipse( displayImage,  cvPoint(hitx, Y_LENGTH - 50), cvSize(30, 30), 0, 0, 360, CV_RGB(255, 20, 20), 0, 1, 0);
			}
			// reset hitx
			hitx = X_LENGTH / 2;
		}
		else{
			cvEllipse( displayImage,  cvPoint(hitx, Y_LENGTH - 50), cvSize(30, 30), 0, 0, 360, CV_RGB(255, 255, 255), 0, 1, 0);
		}	

		
		// set where the robot should tap (ideally table border)
		//float slope = (predYY - predY)/(predXX - predX);
		//float offset = predY - slope * predX;
	       
// 		// if moving rightwards
// 		if (predXX > predX + 10.0){
// 		  target->tapx =  TAP_RIGHT_X;
// 		  target->setTapY(slope * TAP_RIGHT_X + offset);
// 		  //		  target->tapy = slope * TAP_RIGHT_X + offset;
// 		  
// 		  
// 		}
// 		// or move left
// 		else if (predX > predXX + 10.0){
// 		  target->setTapY(slope * TAP_LEFT_X + offset);
// 		  //target->tapy = slope * TAP_LEFT_X + offset;
// 		  target->tapx = TAP_LEFT_X;
// 		  
// 		}
		//target->tapx = predXX;
		//target->tapy = predYY;

		//cvEllipse( displayImage, cvPoint(target->tapx,target->tapy), cvSize(5, 5), 
			//		0, 0, 360, CV_RGB(255, 100, 255), CV_FILLED, 1, 0);

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
		//cvEllipse( displayImage, cvPoint(cvmGet(target->kalman->state_pre,0,0), cvmGet(target->kalman->state_pre,1,0)), cvSize(5, 5), 
					//0, 0, 360, CV_RGB(100, 255, 255), CV_FILLED, 1, 0);
	

		cvEllipse( displayImage, cvPoint(predX, predY), cvSize(5, 5), 
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
		//std::cout << "display target at: " << x << ", " << y << std::endl;
	
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
  	
  	
		

	}





}




// adds a new target object
void FuserThread::addTarget( int id, std::string name, double x, double y, double z, double vx, double vy, double vz, int red, int green, int blue){
	///targWaitCond.wait();
	targetMutex.lock();
	//cout << "targMutex locked from add target" << endl;
	
	TargetObject* target = new TargetObject(id, name, x, y,z, vx, vy, vz, red, green, blue, 0);
	targetVector->push_back(target);
	//freeIDs[targetVector->size()- 1] = false;
	//cout << "--------------------------> added target" << endl;
	
	///targWaitCond.wakeAll();
	cout << "targcond wake from add target" << endl;
	targetMutex.unlock();
	//cout << "targMutex unlocked from add target" << endl;
	
} 

// removes target object with given  id
void FuserThread::removeTarget( int id){

	///targWaitCond.wait();
	targetMutex.lock();
	vector<TargetObject*>::iterator targIter; 

	
	//cout << "targMutex locked from remove target" << endl;
	int atPos = 0;

	for( targIter = targetVector->begin(); targIter != targetVector->end(); targIter++){ 	
	  
	  	if ( (*targIter)->id == id){

			//cout << "going to erase id " << id << endl;
			targetVector->erase(targetVector->begin() + atPos);
			//cout << "--------------------------> erased target" << endl;	
			///cout << "erased id: " << id << endl;
			//freeIDs[ii] = true;
			targIter = targetVector->end();
			break;
		
		}
		atPos++;
		
	  }

	///targWaitCond.wakeAll();	
	//cout << "targcond wakeAll from remove target" << endl;
	targetMutex.unlock();
	//cout << "targMutex unlocked from remove target" << endl;
	
} 	



