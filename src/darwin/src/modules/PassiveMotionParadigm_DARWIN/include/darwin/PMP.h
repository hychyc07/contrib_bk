// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2011 Dalia De Santis, Jacopo Zenzeri
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/

// $Id: Matrix.h,v 1.16 2008-10-27 19:00:32 natta Exp $ 


#ifndef PMP_H
#define PMP_H

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include "darwin/tbg.h"

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

/**
* \file PMP.h contains the definition of a PMP type 
*/

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;

using namespace iCub::iKin;

namespace Darwin{

namespace pmp{

class PassiveMotionParadigm
{
	private:
		// class variables
		yarp::sig::Vector q_rightArm;
		yarp::sig::Vector q_ref_right;
		yarp::sig::Vector qdot_rightArm;

		yarp::sig::Vector q_leftArm;
		yarp::sig::Vector q_ref_left;
		yarp::sig::Vector qdot_leftArm;

		yarp::sig::Matrix K_right;
		yarp::sig::Matrix K_left;
		yarp::sig::Matrix K_biman;
		yarp::sig::Matrix Kint_right;
		yarp::sig::Matrix Kint_left;

		yarp::sig::Matrix A_rightArm; // Admittance matrices
		yarp::sig::Matrix A_leftArm;
		yarp::sig::Matrix A_biman;
		yarp::sig::Matrix A_torso;

		yarp::sig::Vector F_right;
		yarp::sig::Vector F_left;

		yarp::sig::Vector x_0R;
		yarp::sig::Vector x_0L;	

		yarp::sig::Vector Tool_R;
		yarp::sig::Vector Tool_L;

		int ref_right_active;
		int ref_left_active;

		unsigned int time; //iteration number
		int scale; // conversion factor mm -> m
		bool useTool_R;
		bool useTool_L;

		string side;
		Property *opt_right, *opt_left, *opt_bimanual, *opt_tbg;

		// ikin chain:
		iCubArm * Rarm;
		iCubArm * Larm;
		iKinChain *Rchain;
		iKinChain *Lchain;

		// class methods
		yarp::sig::Vector calculate_F      (Vector x_tg, Vector x, Matrix K);
		void			  calculate_qdot   (Vector &F_r, Vector &F_l, string s = "null");
		yarp::sig::Vector calculate_Tau_int(string side);
		yarp::sig::Matrix get_Jacobian	   (string side);
		
		//yarp::sig::Matrix get_CompMatrix(Vector F, string side);

		bool initializeChain(string side);
		bool initIkinChain(string side);
		bool setFromProperty(Property *options, string key, Vector &v);
		bool setFromProperty(Property *options, string key, Matrix &m);
		bool setEyeMatrixFromVector(Vector v, Matrix &m);

		void connectTool(Vector &target, const string & _side);

		//ofstream tgx,posx,tgx1,posx1,Fr,Fl;

	public:		
		
		//class variables
		yarp::sig::Matrix TRo_r;
		yarp::sig::Matrix TRo_l;
		yarp::sig::Vector x_tgR;
		yarp::sig::Vector x_tgL;
		yarp::sig::Vector q_0R;
		yarp::sig::Vector q_0L;
		yarp::sig::Vector q_homeR;
		yarp::sig::Vector q_homeL;

		TimeBaseGenerator tbg;

		// class methods
		PassiveMotionParadigm(Property *opt_right, Property *opt_left, Property *opt_bimanual, Property *opt_tbg);
		~PassiveMotionParadigm();

		yarp::sig::Vector get_q_rightArm();
		yarp::sig::Vector get_q_leftArm();
        yarp::sig::Matrix get_A_rightArm();
		yarp::sig::Matrix get_A_leftArm();
		yarp::sig::Matrix get_A_biman();
		yarp::sig::Matrix get_A_torso();
		yarp::sig::Vector get_F_right();
		yarp::sig::Vector get_F_left();
		yarp::sig::Matrix get_K_right();
		yarp::sig::Matrix get_K_left();
		yarp::sig::Matrix get_K_biman();
		yarp::sig::Matrix get_Kint_right();
		yarp::sig::Matrix get_Kint_left();
	
		yarp::sig::Vector get_q_ref_right();
		yarp::sig::Vector get_q_ref_left();

		void set_q_rightArm  (const yarp::sig::Vector &_q_rightArm);
		void set_q_leftArm   (const yarp::sig::Vector &_q_leftArm);
        void set_A_rightArm  (const yarp::sig::Matrix &_A_rightArm);
		void set_A_rightArm  (const yarp::sig::Vector &_A_rightArm);
		void set_A_leftArm   (const yarp::sig::Matrix &_A_leftArm);
		void set_A_leftArm   (const yarp::sig::Vector &_A_leftArm);
		void set_A_biman     (const yarp::sig::Matrix &_A_biman);
		void set_A_biman     (const yarp::sig::Vector &_A_biman);
		void set_A_torso     (const yarp::sig::Matrix &_A_torso);
		void set_A_torso     (const yarp::sig::Vector &_A_torso);
		void set_F_right     (const yarp::sig::Vector &_F_right);
		void set_F_left      (const yarp::sig::Vector &_F_left);
		void set_K_right     (const yarp::sig::Matrix &_K_right);
		void set_K_right     (const yarp::sig::Vector &_K_right);
		void set_K_left      (const yarp::sig::Matrix &_K_left);
		void set_K_left      (const yarp::sig::Vector &_K_left);
		void set_K_biman     (const yarp::sig::Matrix &_K_biman);
		void set_K_biman     (const yarp::sig::Vector &_K_biman);
		void set_Kint_right  (const yarp::sig::Matrix &_Kint_right);
		void set_Kint_right  (const yarp::sig::Vector &_Kint_right);
		void set_Kint_left   (const yarp::sig::Matrix &_Kint_left);
		void set_Kint_left   (const yarp::sig::Vector &_Kint_left);
		void set_q_ref_right (const yarp::sig::Vector &_q_ref_right);		
		void set_q_ref_left  (const yarp::sig::Vector &_q_ref_left);

		// tools handling:
		void set_Tool		 (const yarp::sig::Vector &_ToolEE, const string &_side, bool connect=false);
		void use_Tool		 (const string &_side);
		void leave_Tool		 (const string &_side);
		void delete_Tool	 (const string &_side);
		Vector get_ToolPose	 (const string &_side);


		string getActiveChain();
		bool setActiveChain	 (const string &_side);
		Vector getPose		 (const string &_side);
		Vector get_EEPose	 (const string &_side);

		bool run(Vector par, const Vector *xR_target = NULL, const Vector *xL_target = NULL);		

		yarp::sig::Vector pmp2Sim(Vector xp);
		yarp::sig::Vector Sim2pmp(Vector xs);

		//void Sim_reference(char final_ref);

		friend class PMPthread;
};


}// end namespace pmp
}// end namespace Darwin

#endif
