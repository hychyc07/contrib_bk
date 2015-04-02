// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 # Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: 
 * email:   
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
  
/**
 * @file grasper.cpp
 * @brief main code for the grasper module
 */

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include "Grasper.h"
#include "PoseGenerator.h"

using namespace yarp::sig;
using namespace yarp::os;
using namespace std;


const string Grasper::JOINTS_LEFT="/icubSim/left_arm/rpc:i";
const string Grasper::TOUCH_LEFT = "/icubSim/skin/left_hand";
const string Grasper::JOINTS_RIGHT="/icubSim/right_arm/rpc:i";
const string Grasper::TOUCH_RIGHT = "/icubSim/skin/right_hand";

const string Grasper::CONFIG="config.txt";
const string Grasper::OBJ_FOLD=".\\objects\\";
const string Grasper::CUBOID_FOLD="cuboid\\";

const string Grasper::U_FILE="u.txt";
const string Grasper::S_FILE="s.txt";
const string Grasper::V_FILE="vpred.txt";

const unsigned int Grasper::NUM_FINGS=5;
const int Grasper::NUM_SENS=12;
const double Grasper::FING_THREAS=0.6;
const double Grasper::INCREMENT=1.0;

void Grasper::move_fingers(const int firstJoint, const int lastJoint, const Vector& data, Port& hand){

		//assert(data.length()==(lastJoint-firstJoint));

		ofstream myfile("grasp_output.txt",ios_base::app);

		Bottle reply;

		int i=0;
		for(int j=firstJoint;j<lastJoint;j++){
			Bottle request("set pos ");
			request.addInt(j);
			request.addDouble(data(i));
			hand.write(request,reply);
			i++;
			myfile << data(i) << " ";
		}
		
		myfile << "\n-------" << endl;
		

		Time::delay(0.5);

}

Vector Grasper::readConfig(const string& path){
	ifstream configFile( (path+CONFIG).c_str());

	Vector conf;

	while(!configFile.eof()){
		string line;

		getline(configFile,line);

		if(line.at(0)!='#'){
			istringstream iss(line);
			double val=-1.0;
			iss >> val;
			conf.push_back(val);
		}
	}

	//assert(conf.length()==3);
	return conf;
}

Matrix Grasper::readMatrix(const string& filename,int cols){
  // ifstream myfile(filename); #Rea correction 9/01/13
    ifstream myfile(filename.c_str(), ifstream::in  );


	Matrix res(2,cols);		//MAGIC NUMBER

	string line;
	int colCount=0;
	for(int i=0;i<2;i++){		//MAGIC NUMBER

		getline(myfile,line);

		istringstream iss(line);

		//for(int j=0;j<res.cols();j++){
		int j=0;
		while(!iss.eof()){
			iss >> res(i,j);
			colCount=j;
			j++;

		}
		//cout << endl;

	}//for k
	cout << endl;

	if(colCount<res.cols()){
		//res.resize(2,colCount);
		res=res.submatrix(0,1,0,colCount-1);
	}

	return res;
}//func
//25

void Grasper::squeeze(Port& touch, Port& hand){
	
	ofstream myfile("grasp_output.txt",ios_base::app);
	myfile << "###########" << endl;
	

	Bottle touch_reply;
	Bottle reply;


	bool squeezed=false;
	while(!squeezed){

		double totActive=0.0;

		touch.read(touch_reply);
		
		//scanning all the fingertips, from thumb to pinky - to count number of active sensors
		Vector jointsMask(JointsNum-thumb_oppose);
		jointsMask.zero();
		
		for(int i=0;i<(NUM_FINGS*NUM_SENS);i++){
	
			if(touch_reply.get(i).asDouble() != 0.0){
				totActive+=1.0;
			}
		}

		//if more than 60% sensors is active, stop squeezing
		if(totActive>=(NUM_FINGS*NUM_SENS*FING_THREAS)){
			squeezed=true;
			cout << "Gsp> DONE! :) "<< endl;
		}
		else{

			//----Squeezing
			cout << endl;	
			cout << "Gsp> Squeezing..." << endl;
			Vector data;
			
			//scanning all hand's joints to get encoders' value
			//for(int j= Joints::thumb_oppose;j<Joints::JointsNum;j++){   #Rea corrected
			for(int j= thumb_oppose;j<JointsNum;j++){			// - 8: thumb flexion, 15: ring+pinky flexion - joint 9: suggest to jump it (flex the thumb to the fingers)
			
				Bottle request("get enc ");
				request.addInt(j);
				hand.write(request,reply);
				double val=reply.get(2).asDouble();

				reply.clear();
				Bottle request_lims("get llim ");
				request_lims.addInt(j);
				hand.write(request_lims,reply);
				double max=reply.get(3).asDouble();

                // Rea change 10/1/13 if(j!=Joints::thumb_proximal && val<(max-INCREMENT)){
				if(j!=thumb_proximal && val<(max-INCREMENT)){				// - 9
					val+=INCREMENT;
				}else{
					jointsMask[j-thumb_oppose]=1.0;		//joint has reached limit
					cout << "Gsp> Joint "<< j << " has reached its limit" << endl;
				}
				data.push_back(val);
			}

			move_fingers(thumb_oppose,JointsNum,data,hand);
			
		}

		double totLimit=0.0;
		for(unsigned int i=0;i<jointsMask.length();i++){
			totLimit+=jointsMask[i];
		}
		if(totLimit>=(JointsNum-thumb_oppose)){
			squeezed=true;
			cout << "Gsp> All joints have reached thier limit"<<endl;
		}

		//--------------------------------

		cout << "Gsp> Total Number activated sensors: " << totActive << endl;
		cout << "Gsp> Total Number of joints locked: " << totLimit << endl;
		
	}
}




void Grasper::grasp(PoseGenerator::Hands domHand,PoseGenerator::ObjectClass objType){

	Network yarp;

	Port hand;
	hand.open("/out/hand");

	Port touch;
	touch.open("/in/touch");

	string path;

	//setting hand and object class
	switch(domHand){

		case PoseGenerator::Left:
			Network::connect("/out/hand",JOINTS_LEFT.c_str());
			Network::connect(TOUCH_LEFT.c_str(),"/in/touch");
		break;

		case PoseGenerator::Right:
			Network::connect("/out/hand",JOINTS_RIGHT.c_str());
			Network::connect(TOUCH_RIGHT.c_str(),"/in/touch");
		break;
	
		default:	//should never happen
			Network::connect("/out/hand",JOINTS_LEFT.c_str());
			Network::connect(TOUCH_LEFT.c_str(),"/in/touch");
		break;
	}

	switch(objType){	//one day there will be object classes like cuboid, cylinder etc.
		
	case PoseGenerator::Tall:
		path=OBJ_FOLD+CUBOID_FOLD;//".\\objects\\cuboid\\";
	break;
	
	default:
		path=OBJ_FOLD+CUBOID_FOLD;//".\\objects\\cuboid\\";
	break;
	}

	//reading data from files
	Vector conf=readConfig(path);

	Matrix u=Grasper::readMatrix(path+U_FILE,conf[0]);//12);	//"u.txt"
	cout << "Gsp> U read" << endl;
	Matrix s=Grasper::readMatrix(path+S_FILE,conf[1]);//12);	//"s.txt"
	cout << "Gsp> S read" << endl;
	//Matrix cents=Grasper::readMatrix(path+"cents.txt",5);
	//cout << "cents read" << endl;


	Matrix vPred=Grasper::readMatrix(path+V_FILE,conf[2]);//40);//25);	//"vpred.txt"
	cout << "Gsp> vpred read" << endl;

	Matrix joints(vPred.cols(),u.cols());
	joints.zero();

	//reconstructiong joint matrix via usv multiplication
	for(int i=0;i<joints.cols();i++){
		for(int j=0;j<joints.rows();j++){
			joints(j,i)=u(0,i)*s(0,0)*vPred(0,j)+u(1,i)*s(1,1)*vPred(1,j);
			if(i==1 || i==6 || i==11){
				joints(j,i)=-1*joints(j,i);
			}
		}
	}

	//1: -1 / 6: +1

	ofstream myfile("grasp_output.txt");
	cout << "\nGsp> Grasping";// << endl;
	for(int i=0;i<joints.rows();i++){
        //Rea change 10/1/13 move_fingers(Joints::wrist_prosup,JointsNum,joints.getRow(i),hand);		//everything before 4 is the arm
		move_fingers(wrist_prosup,JointsNum,joints.getRow(i),hand);		//everything before 4 is the arm
		cout << ".";
	}
	cout << endl;

	squeeze(touch,hand);
	myfile.close();

	cout<< "Press enter to continue..." << endl;
	cin.ignore();
}



