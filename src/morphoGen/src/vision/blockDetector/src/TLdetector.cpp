// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
 * TLdetector.hpp
 * Texture-less object detection class.
 *
 * before run it, you may edit the yml file first for parameter setting
 * Example:
	string test_im_file = "data_cmp8toys/test_01.jpg";
	string para_yml_file = "data_cmp8toys/para_cmp8toys.yml";
	Mat im = imread(test_im_file);

	CTLdetector detector;
	detector.initiate(para_yml_file);          //step1: initiate
	detector.train();                          //step2: train
	detector.detect(im);                       //step3: detect
	detector.showDetObjs(im,Scalar(0,255,0),Scalar(255,255,255));
	detector.dispDetObjs();
 *
 *  Created on: Jan 28, 2013
 *      Author: caihongp
 */

#undef __GXX_EXPERIMENTAL_CXX0X__
#include "TLdetector.hpp"
#include "caiMat.hpp"
#include "caihash.hpp"       //table related
#include "dt.h"              // distance transform
#include "verifyChamfer.hpp" //Chamfer verification

#include <opencv/highgui.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#define VL_MIN(x,y) (((x)>(y))?(y):(x))
using namespace cv;
using namespace std;

CTLdetector::CTLdetector()
{
	all_obj_cls = NULL;
	ptables = NULL;
}

CTLdetector::~CTLdetector()
{
	delete [] ptables;
	delete [] all_obj_cls;
	inds_para.inds_feat_pt.clear();
	for (int i=0;i<n_tr;i++)
	{
		vec_mat_tr_edge.at(i).release();
		vec_mat_tr_dis.at(i).release();
		vec_mat_tr_ori.at(i).release();
	}
	vec_mat_tr_edge.clear();
	vec_mat_tr_dis.clear();
	vec_mat_tr_ori.clear();
	mat_inds_grouppt.release();
	mat_tr_rect_tight.release();
	vec_tr_id_labels.clear();
	for (int i=0;i<vec_detobj.size();i++){
    	 vec_detobj[i].mat_edge_NN_tr.release();
	}
	vec_detobj.clear();
}




/*
 * Initiate the parameters from *.yml file
 * if input "", then use default valuables
 */
bool CTLdetector::initiate(string yml_file)
{
	FileStorage fs(yml_file,FileStorage::READ);
	if (!fs.isOpened())//if input yml_file doesn't exist, then default valuables
	{
		cout<<"******************************************************"<<endl;
		cout<<"******************   WARNING  ********************"<<endl;		
		cout<<"***Initial file "<<yml_file<<" doesn't exist."<<endl;
		cout<<"   Initiate with default parameters for ## CMP8toys ##."<<endl;
		cout<<"******************************************************"<<endl;
		cout<<"******************************************************"<<endl;
		table_method = TABLE_GROUPPT;

        //*************************************************************************************
        // hard coded file must be removed
        tr_bin_file = "/usr/local/src/robot/iCub/app/morphoGenApp/conf/tr_data_blocks.bin";
        tr_yml_file = "/usr/local/src/robot/iCub/app/morphoGenApp/conf/para_blocks.yml";
        //
        //************************************************************************************
        
		cout<<" table_method:"<<table_method<<endl;
		cout<<" tr_bin_file:"<<tr_bin_file<<endl;

		be_ori_pi = false;
		be_blur   = true;

		norm_para.h_fix = 48;
		norm_para.w_fix = 48;

		test_para.max_cols_rows = 640;
		test_para.scale_factor = 1.2f;
		test_para.aspect_factor = 1.15f;
		test_para.min_scale = -10;
		test_para.max_scale = -1;
		test_para.min_aspect = 0;
		test_para.max_aspect = 0;
		test_para.canny_thigh = 140.0f;
		test_para.canny_tlow = 70.0f;
		test_para.min_edge_pt = 90;
		test_para.max_edge_pt = 50; //added on 03/03/2013
		test_para.th_overlap_out = 0.60f;
		test_para.nmin_matches = 12;
		test_para.sliding_step = 3;

		chamfer_para.th_dis = 3.1f;
		chamfer_para.th_ori = (float)CV_PI/9.0f;
		chamfer_para.th_match_score = 0.64f;//
		chamfer_para.k_linear = 0.6666666666666667f;
		chamfer_para.b_linear = 41.683461934156384f;

		quantize_para.n_bin_dis = 4;
		quantize_para.n_bin_ori_PI = 4;
		quantize_para.n_bin_ori = (be_ori_pi)?(quantize_para.n_bin_ori_PI):(2*quantize_para.n_bin_ori_PI);//
		quantize_para.MAX_range_dis = 12.0f;
		quantize_para.MAX_range_ori = (be_ori_pi)?((float)CV_PI):(2*(float)CV_PI);

		grouppt_para.n_pt = 3;
		grouppt_para.n_table = 50;
		grouppt_para.nmin_matches = 4;

		inds_para.n_cell_x = 8;
		inds_para.n_cell_y = 8;

		n_obj_cls = 8;
		all_obj_cls = new string[n_obj_cls];
		all_obj_cls[0] = "block";
		all_obj_cls[1] = "eye";
		all_obj_cls[2] = "screw";
		all_obj_cls[3] = "driver";
		all_obj_cls[4] = "bridge";
		all_obj_cls[5] = "whiteblock";
		all_obj_cls[6] = "cup";
		all_obj_cls[7] = "lid";
		cout<<" "<<n_obj_cls<<" training objects:"<<endl;
		for (int i=0;i<n_obj_cls;i++)
		{
			cout<<" //obj"<<i<<": "<<all_obj_cls[i] <<endl;
		}
		cout<<"   Write parameters to "<<yml_file<<endl;
		writeParaFileYml(yml_file);
	}
	else
	{
		cout<<"***Load parameters from "<<yml_file <<endl;
		bool flag = loadParaFileYml(yml_file);
		if (!flag)
			return false;
	}
	generateIndsPt();//get inds_para.inds_feat_pt, inds_para.n_feat_pt.
	ptables = NULL;
	printf("***Done.\n");
	return true;
};
/*
 * load the parameter file (*.yml)
 */
bool CTLdetector::loadParaFileYml(string yml_file)
{
	FileStorage fs(yml_file,FileStorage::READ);
	if (!fs.isOpened())
	{
		cout<< "ERROR: "<<yml_file<<" doesn't exist.";
		return false;
	}
	FileNode tm = fs["norm_para"];
	table_method = (int)fs["table_method"];
	be_ori_pi = ((int)fs["be_ori_pi"]>0);
	be_blur = ((int)fs["be_blur"]>0);
	tr_bin_file = (string)fs["tr_bin_file"];
	tr_yml_file = (string)fs["tr_yml_file"];
	cout<<"-table_method:"<<table_method<<endl;
	cout<<"-tr_bin_file:"<<tr_bin_file<<endl;
	if (be_ori_pi)
		cout<<"-Edge orientation in [0 pi]."<<endl;
	else
		cout<<"-Edge orientation in [-pi pi]."<<endl;
	
	norm_para.h_fix = (int)tm["h_fix"];
	norm_para.w_fix = (int)tm["w_fix"];

	tm = fs["test_para"];
	test_para.max_cols_rows  = (int)tm["max_cols_rows"];
	test_para.scale_factor   = (float)tm["scale_factor"];
	test_para.aspect_factor  = (float)tm["aspect_factor"];
	test_para.min_scale      = (int)tm["min_scale"];
	test_para.max_scale      = (int)tm["max_scale"];
	test_para.min_aspect     = (int)tm["min_aspect"];
	test_para.max_aspect     = (int)tm["max_aspect"];
	test_para.canny_thigh    = (float)tm["canny_thigh"];
	test_para.canny_tlow     = (float)tm["canny_tlow"];
	test_para.min_edge_pt    = (int)tm["min_edge_pt"];
	test_para.max_edge_pt    = (int)tm["max_edge_pt"];
	test_para.nmin_matches   = (int)tm["nmin_matches"];
	test_para.sliding_step   = (int)tm["sliding_step"];
	test_para.th_overlap_out = (float)tm["th_overlap_out"];

	tm = fs["chamfer_para"];
	chamfer_para.th_dis = (float)tm["th_dis"];
	chamfer_para.th_ori = (float)tm["th_ori"];
	chamfer_para.th_match_score = (float)tm["th_match_score"];
	chamfer_para.k_linear = (float)tm["k_linear"];
	chamfer_para.b_linear = (float)tm["b_linear"];

	tm = fs["quantize_para"];
	quantize_para.n_bin_dis = (int)tm["n_bin_dis"];
	quantize_para.n_bin_ori_PI = (int)tm["n_bin_ori_PI"];
	quantize_para.MAX_range_dis = (float)tm["MAX_range_dis"];
	quantize_para.n_bin_ori = (be_ori_pi)?(quantize_para.n_bin_ori_PI):(2*quantize_para.n_bin_ori_PI);//
	quantize_para.MAX_range_ori = (be_ori_pi)?((float)CV_PI):(2*(float)CV_PI);//
	

	tm = fs["grouppt_para"];
	grouppt_para.n_pt = (int)tm["n_pt"];
	grouppt_para.n_table = (int)tm["n_table"];
	grouppt_para.nmin_matches = (int)tm["nmin_matches"];

	tm = fs["inds_para"];
	inds_para.n_cell_x = (int)tm["n_cell_x"];
	inds_para.n_cell_y = (int)tm["n_cell_y"];

	n_obj_cls = (int)fs["n_obj_cls"];
	all_obj_cls = new string[n_obj_cls];
	cout<<"-"<<n_obj_cls<<" training objects."<<endl;
	for (int i=0;i<n_obj_cls;i++)
	{
		stringstream ss;
		ss << "obj" << i;
		all_obj_cls[i] = (string)fs[ss.str()];
		cout<<" //obj"<<i<<":"<<all_obj_cls[i] <<endl;
	}
	return true;
}
/*
 * write the parameters into a yml file (*.yml)
 */
bool CTLdetector::writeParaFileYml(string yml_file)
{
	FileStorage fs(yml_file,FileStorage::WRITE);
	fs << "table_method" << table_method;
	if (be_ori_pi)
		fs << "be_ori_pi" << 1;   /////
	else
		fs << "be_ori_pi" << 0;   /////
	if (be_blur)
		fs << "be_blur" << 1;   /////
	else
		fs << "be_blur" << 0;   /////
		
	fs << "tr_bin_file" << tr_bin_file;
	fs << "tr_yml_file" << tr_yml_file;
	fs << "norm_para" << "{" << "h_fix" << norm_para.h_fix << "w_fix" << norm_para.w_fix << "}";
	fs << "test_para" << "{"
			<< "max_cols_rows" << test_para.max_cols_rows
			<< "scale_factor"  << test_para.scale_factor
			<< "aspect_factor" << test_para.aspect_factor
			<< "min_scale"     << test_para.min_scale
			<< "max_scale"     << test_para.max_scale
			<< "min_aspect"    << test_para.min_aspect
			<< "max_aspect"    << test_para.max_aspect
			<< "canny_thigh"   << test_para.canny_thigh
			<< "canny_tlow"    << test_para.canny_tlow
			<< "min_edge_pt"   << test_para.min_edge_pt
			<< "max_edge_pt"   << test_para.max_edge_pt
			<< "th_overlap_out"<< test_para.th_overlap_out
			<< "nmin_matches"  << test_para.nmin_matches
			<< "sliding_step"  << test_para.sliding_step
			<< "}";
	fs << "chamfer_para" << "{"
			<< "th_dis" << chamfer_para.th_dis
			<< "th_ori"  << chamfer_para.th_ori
			<< "th_match_score" << chamfer_para.th_match_score
			<< "k_linear"     << chamfer_para.k_linear
			<< "b_linear"     << chamfer_para.b_linear
			<< "}";
	fs << "quantize_para" << "{"
			<< "n_bin_dis" << quantize_para.n_bin_dis
			<< "n_bin_ori_PI" << quantize_para.n_bin_ori_PI
			<< "MAX_range_dis" << quantize_para.MAX_range_dis
			<< "}";
	fs << "grouppt_para" << "{"
			<< "n_pt" << grouppt_para.n_pt
			<< "n_table" << grouppt_para.n_table
			<< "nmin_matches" << grouppt_para.nmin_matches
			<< "}";
	fs << "inds_para" << "{"
			<< "n_cell_x" << inds_para.n_cell_x
			<< "n_cell_y" << inds_para.n_cell_y
			<< "}";
	fs << "n_obj_cls" << n_obj_cls;
	for (int i=0;i<n_obj_cls;i++)
	{
		stringstream ss;
		ss << "obj" << i;
		fs <<ss.str()<< all_obj_cls[i];
	}
	return true;
}

/*
 * Generate the 36-pt training features from the distance and orientation map
 */
void CTLdetector::generateTrainFeatures(float *pout_feat_dis, float *pout_feat_ori)
{
	int i,j,pos;
	Point pt;
	for (i=0,pos=0;i<n_tr;i++)
	{
		for (j=0;j<inds_para.n_feat_pt;j++,pos++)
		{
			pt = inds_para.inds_feat_pt.at(j);
			pout_feat_dis[pos] = vec_mat_tr_dis.at(i).at<float>(pt.y,pt.x);
			pout_feat_ori[pos] = vec_mat_tr_ori.at(i).at<float>(pt.y,pt.x);
		}
	}
	return;
}
/*
 * Generate the indexes for 36 keypoints
 * output: inds_para.inds_feat_pt, inds_para.n_feat_pt.
 */
void CTLdetector::generateIndsPt()
{
	inds_para.inds_feat_pt.clear();
	int grid_x = norm_para.w_fix/inds_para.n_cell_x;
	int grid_y = norm_para.h_fix/inds_para.n_cell_y;
	int x_half = grid_x/2;
	int y_half = grid_y/2;
	int x,y,ntmp;
	for (x=1;x<inds_para.n_cell_x-1;x++)
	{
		ntmp = x*grid_x;
		for (y=1;y<inds_para.n_cell_y-1;y++)
			inds_para.inds_feat_pt.push_back(Point(ntmp+x_half,y*grid_y+y_half));
	}
	//cout<<"inds_feat_pt[0]:"<<inds_para.inds_feat_pt.at(0).x<<"*"<<inds_para.inds_feat_pt.at(0).y<<endl;
	//cout<<"inds_feat_pt[35]:"<<inds_para.inds_feat_pt.at(35).x<<"*"<<inds_para.inds_feat_pt.at(35).y<<endl;
	inds_para.n_feat_pt = (inds_para.n_cell_x-2)*(inds_para.n_cell_y-2);
	return;
}
/*
 * load the binary train file (*.bin). It is much fast to read a binary file
 * Data include:
 *    n_tr,
 *    mat_inds_grouppt,
 *    mat_tr_rect_tight,
 *    vec_tr_id_labels,
 * //  vec_tr_id_prototype,
 * //  vec_tr_which_k
 *    vec_mat_tr_edge,
 *    vec_mat_tr_dis,
 *    vec_mat_tr_ori
 */
bool CTLdetector::loadTrainFileBin(string bin_file)
{
	std::ifstream infile(bin_file.c_str(),std::ifstream::binary);
	if (!infile.good())
	{
		cout<< " "<<bin_file<<" doesn't exist.";
		return false;
	}

	infile.read((char* ) &n_tr,sizeof(int));
	infile.read((char* ) &norm_para.h_fix,sizeof(int));
	infile.read((char* ) &norm_para.w_fix,sizeof(int));
	cout<< "   Edge, ori, dis matrix size:"<<n_tr<<"*"<<norm_para.h_fix<<"*"<<norm_para.w_fix<<endl;
	infile.read((char* ) &grouppt_para.n_pt,sizeof(int));
	infile.read((char* ) &grouppt_para.n_table,sizeof(int));
	cout<< "   Grouppt: n_table:"<<grouppt_para.n_table<<" n_pt:"<<grouppt_para.n_pt<<endl;
	mat_inds_grouppt = Mat::zeros(grouppt_para.n_pt,grouppt_para.n_table,CV_32SC1);
	infile.read((char* )mat_inds_grouppt.data,mat_inds_grouppt.step * mat_inds_grouppt.rows);//   grouppt_para.n_table*grouppt_para.n_pt*sizeof(int));

	mat_tr_rect_tight = Mat::zeros(n_tr,4,CV_32SC1);
	infile.read((char* )mat_tr_rect_tight.data,mat_tr_rect_tight.step * mat_tr_rect_tight.rows);//
	int ntmp;
	for(size_t i = 0; i < n_tr; i++){
		infile.read((char* ) &ntmp,sizeof(int));
		vec_tr_id_labels.push_back(ntmp);
		Mat edge = Mat::zeros( norm_para.h_fix, norm_para.w_fix, CV_8UC1 );
		infile.read ( (char*) edge.data, norm_para.h_fix * norm_para.w_fix * sizeof(uchar));
		vec_mat_tr_edge.push_back(edge);
		Mat dis = Mat::zeros( norm_para.h_fix, norm_para.w_fix, CV_32FC1 );
		infile.read ( (char*) dis.data, norm_para.h_fix * norm_para.w_fix * sizeof(float));
		vec_mat_tr_dis.push_back(dis);
		Mat ori = Mat::zeros( norm_para.h_fix, norm_para.w_fix, CV_32FC1 );
		infile.read ( (char*) ori.data, norm_para.h_fix * norm_para.w_fix * sizeof(float));
		vec_mat_tr_ori.push_back(ori);
	}
	infile.close();
	//cv::imshow("edge", vec_mat_tr_edge.at(7000)*255);
	//cv::waitKey(20000);

	if (be_ori_pi)
		oriTrans2RangePi();
	return true;
}
/*
 * write the binary train file (*.bin). It is much fast to read a binary file
 *
 * Data include:
 *    n_tr,
 *    mat_inds_grouppt,
 *    mat_tr_rect_tight,
 *    vec_tr_id_labels,
 *   // vec_tr_id_prototype,
 *   // vec_tr_which_k
 *    vec_mat_tr_edge,
 *    vec_mat_tr_dis,
 *    vec_mat_tr_ori
 */
bool CTLdetector::writeTrainFileBin(string bin_file)
{
	ofstream output(bin_file.c_str(),ios::out|ios::binary);
	output.write((char *) &n_tr, sizeof(int));
	size_t dataSize = vec_mat_tr_edge.at(0).rows * vec_mat_tr_edge.at(0).cols;
	output.write((char *) &vec_mat_tr_edge.at(0).rows, sizeof(int));
	output.write((char *) &vec_mat_tr_edge.at(0).cols, sizeof(int));
	output.write((char *) &mat_inds_grouppt.rows, sizeof(int));
	output.write((char *) &mat_inds_grouppt.cols, sizeof(int));
	size_t matSize = mat_inds_grouppt.step * mat_inds_grouppt.rows;
	output.write((char* )mat_inds_grouppt.data, matSize);

	matSize = mat_tr_rect_tight.step * mat_tr_rect_tight.rows;
	output.write((char* )mat_tr_rect_tight.data, matSize);
	for(size_t i = 0; i < n_tr; i++){
		output.write((char *) &vec_tr_id_labels.at(i), sizeof(int));
//		output.write((char *) &vec_tr_which_k.at(i), sizeof(int));
		output.write( (const char* ) vec_mat_tr_edge[i].data, dataSize * sizeof(uchar) );
		output.write( (const char* ) vec_mat_tr_dis[i].data, dataSize * sizeof(float) );
		output.write( (const char* ) vec_mat_tr_ori[i].data, dataSize * sizeof(float) );
	}

//	int ntmp = vec_tr_id_prototype.size();
//	output.write((char *) &ntmp, sizeof(int));
//	for (size_t i=0; i< ntmp;i++)
//		output.write((char *) &vec_tr_id_prototype.at(i), sizeof(int));
	output.close();
	return true;
}
/*
 * load the yml train file (*.yml)
 * Data include:
 *    n_tr,
 *    mat_inds_grouppt,
 *    mat_tr_rect_tight,
 *    vec_tr_id_labels,
 * //   vec_tr_id_prototype,
 *  //  vec_tr_which_k
 *    vec_mat_tr_edge,
 *    vec_mat_tr_dis,
 *    vec_mat_tr_ori
 */
bool CTLdetector::loadTrainFileYml(string yml_file)
{

	FileStorage fs(yml_file,FileStorage::READ);
	if (!fs.isOpened())
	{
		cout<< "ERROR: "<<yml_file<<" doesn't exist.";
		return false;
	}
/*  commented VM
	n_tr = (int)fs["n_tr"];
	fs["mat_inds_grouppt"]>>mat_inds_grouppt;
	fs["mat_tr_rect_tight"]>>mat_tr_rect_tight;
	vec_tr_id_labels.reserve(n_tr);
	fs["vec_tr_id_labels"]>>vec_tr_id_labels;
//	fs["vec_tr_id_prototype"]>>vec_tr_id_prototype;
//	vec_tr_which_k.reserve(n_tr);
//	fs["vec_tr_which_k"]>>vec_tr_which_k;
	vec_mat_tr_edge.reserve(n_tr);
	cout<<"  Loading vec_mat_tr_edge..."<<endl;
	fs["vec_mat_tr_edge"]>>vec_mat_tr_edge;
	vec_mat_tr_dis.reserve(n_tr);
	cout<<"  Loading vec_mat_tr_dis..."<<endl;
	fs["vec_mat_tr_dis"]>>vec_mat_tr_dis;
	vec_mat_tr_ori.reserve(n_tr);
	cout<<"  Loading vec_mat_tr_ori..."<<endl;
	fs["vec_mat_tr_ori"]>>vec_mat_tr_ori;
	grouppt_para.n_table = mat_inds_grouppt.cols;
	grouppt_para.n_pt    = mat_inds_grouppt.rows;
	fs.release();*/
	return true;
}
/*
 * Load the yml file, then write the data to a binary train file (*.bin)
 * It is much fast to read a binary file
 * Data include:
 *    n_tr,
 *    mat_inds_grouppt,
 *    mat_tr_rect_tight,
 *    vec_tr_id_labels,
 *  //  vec_tr_id_prototype,
 *   // vec_tr_which_k
 *    vec_mat_tr_edge,
 *    vec_mat_tr_dis,
 *    vec_mat_tr_ori
 */
bool CTLdetector::transferTrainFileYml2Bin(string yml_file, string bin_file)
{
	bool flag;
	cout<<"  read yml file..."<<std::endl;
	flag = loadTrainFileYml(yml_file);
	if (!flag)
		return false;
	cout<<"  Write to bin file..."<<std::endl;
	writeTrainFileBin(bin_file);
	if (be_ori_pi)
		oriTrans2RangePi();
	return true;
};
/*
 * Transform the vec_mat_tr_ori in [-pi pi] to [0 pi]
 *
 */
void CTLdetector::oriTrans2RangePi()
{
	size_t p,q;
	for (size_t i=0;i<n_tr;i++){
		Mat mat_ori = vec_mat_tr_ori[i];
		for (p=0;p<mat_ori.rows;p++)
			for (q=0;q<mat_ori.cols;q++)
				if (mat_ori.at<float>(p,q)<0)
					mat_ori.at<float>(p,q)+=(float)CV_PI;

	}
	return;
}

/*
 * Texture-less object training stage
 * Load the template data from *.yml or *.bin file
 * and build the table
 *
 * OutPUT:
 *  if it goes correctly, true.
 */
bool CTLdetector::train()
{
	printf("***Training...\n");
	//////////////////////////////////////////////////////
	//step1: load training data
	//////////////////////////////////////////////////////
	cout<<" //step1: load training data from "<<tr_bin_file<<endl;
	if (!loadTrainFileBin(tr_bin_file)) // bin file loading failed
	{
		cout<<"  load training data from "<<tr_yml_file<<endl;
		bool flag = transferTrainFileYml2Bin(tr_yml_file, tr_bin_file);	
		if (!flag)
			return false;
	}
	//generate ptr_feats_dis, ptr_feats_ori
	int nD = n_tr*inds_para.n_feat_pt;
	float *ptr_feats_dis = new float[nD];
	float *ptr_feats_ori = new float[nD];
	generateTrainFeatures(ptr_feats_dis, ptr_feats_ori);

	//////////////////////////////////////////////////////
	//step2: quantize features
	//////////////////////////////////////////////////////
	printf(" //step2: quantize features\n");
	unsigned char *pint_dis_ori=new unsigned char[nD];
	quantizeDisOri(ptr_feats_dis, ptr_feats_ori, nD, quantize_para.n_bin_dis, quantize_para.n_bin_ori,
		quantize_para.MAX_range_dis, quantize_para.MAX_range_ori, be_ori_pi, pint_dis_ori);
	delete []ptr_feats_dis;
	delete []ptr_feats_ori;

	//////////////////////////////////////////////////////
	//step3: build tables
	//////////////////////////////////////////////////////
	printf(" //step3: build tables\n");
    int *pinds_grouppt;
    caiMat tmpmat(grouppt_para.n_pt,grouppt_para.n_table,CV_32S);
	int ntable_size;
	if (ptables != NULL)
		delete []ptables;
	switch(table_method)
	{
	    case TABLE_PLAIN:
	    	ntable_size = quantize_para.n_bin_dis*quantize_para.n_bin_ori*inds_para.n_feat_pt;
	    	ptables = new vector<int>[ntable_size];
	    	buildPlainTablesFromInt(pint_dis_ori,inds_para.n_feat_pt, n_tr,
	    			quantize_para.n_bin_dis, quantize_para.n_bin_ori,ptables);
	        break;
	    case TABLE_GROUPPT:
	    	pinds_grouppt = new int[grouppt_para.n_pt*grouppt_para.n_table];
	    	tmpmat = mat_inds_grouppt;
	    	tmpmat.convert2MatlabMat(pinds_grouppt);
	    	ntable_size = (int)pow((double)quantize_para.n_bin_dis*quantize_para.n_bin_ori,grouppt_para.n_pt)*grouppt_para.n_table;
	   		ptables = new vector<int>[ntable_size];
			buildOrgTablesFromInt(pint_dis_ori, inds_para.n_feat_pt, n_tr,quantize_para.n_bin_dis, quantize_para.n_bin_ori,
					pinds_grouppt,  grouppt_para.n_pt,  grouppt_para.n_table, ptables);
			delete [] pinds_grouppt;
	    	break;
	    case TABLE_CLUSTER:
			cout<<"ERROR: TABLE_CLUSTER is not supported so far."<<endl;
			return false;
	        break;
	    default:
			cout<<"ERROR: Only TABLE_PLAIN (0) and TABLE_Grouppt (1) are supported so far."<<endl;
	    	return false;
	}

	delete [] pint_dis_ori;
	printf("***Done.\n");
	return true;

};
/*
 * draw edges of the NN training templates inside the bounding box
 * INPUT:
 * im             -- original image matrix
 * mat_edge_patch -- edge of the NN training template, norm_para.h_fix*norm_para.w_fix
 * rt             -- the bounding rectangle on the original image.
 * edge_color     -- edge color, can be (255,255,255) for rgb image, or 255 for gray-value image
 */
void CTLdetector::drawEdgePatch(Mat &im, Mat &mat_edge_patch, Rect rt, Scalar edge_color)
{
	// map the small patch edge to the rect
	Mat mat_edge_in_rect = Mat::zeros(rt.height,rt.width,CV_8UC1);
	cv::resize(mat_edge_patch,mat_edge_in_rect,mat_edge_in_rect.size(),0,0,INTER_NEAREST);

	// change the pixel value on the edge points
	if (im.channels()>1)
	{
		for (int i=0;i<rt.height;i++)
			for (int j=0;j<rt.width;j++)
				if (mat_edge_in_rect.at<uchar>(i,j)>0)
					im.at<Vec3b>(i+rt.y,j+rt.x) = Vec3b(edge_color(0),edge_color(1),edge_color(2));
	}
	else
		for (int i=0;i<rt.height;i++)
			for (int j=0;j<rt.width;j++)
				if (mat_edge_in_rect.at<uchar>(i,j)>0)
					im.at<uchar>(i+rt.y,j+rt.x)= (uchar)edge_color(0);
	mat_edge_in_rect.release();
	return;
}
/*
 * show detected objects, bounding box, label, score, matched edge map.
 * INPUT:
 * im             -- original image matrix
 * box_color      -- bounding box, label, score color.
 * edge_color     -- edge color, can be (255,255,255) for rgb image, or 255 for gray-value image
 * w_show_wind    -- the display window width,if 0 or -1 (default),
 *                   means showing the original image size
 */
void CTLdetector::showDetObjs(Mat im,Scalar box_color,Scalar edge_color,int w_show_wind)
{
	int thickness = 2;
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale = 1;

	// show detected rects
	string text;
	char buf[100];
	if ((w_show_wind<1)||(w_show_wind==im.cols)) //show the original image size
	{
        
        for (int i=0;i<vec_detobj.size();i++)
		{
			Rect rt = vec_detobj[i].box;
			Rect rt_tight = vec_detobj[i].box_tight;
			rectangle(im,rt_tight.tl(),rt_tight.br(),box_color,thickness);
			// then put the text
			std::sprintf(buf,"%s:%.2f",all_obj_cls[vec_detobj[i].id_label].c_str(),vec_detobj[i].match_score);
			text = buf;
			putText(im, text, rt_tight.tl()+Point(0,-3), fontFace, fontScale,box_color);
			drawEdgePatch(im, vec_detobj[i].mat_edge_NN_tr, rt, edge_color);
		}
		//namedWindow( "Texture-less object detection", CV_WINDOW_KEEPRATIO | CV_WINDOW_AUTOSIZE);
		//imshow("Texture-less object detection",im);

        postProcessIm = im;
	}
	else //resize the image
	{
		double s = w_show_wind*1.0/im.cols;
		int h_show_wind = int(im.rows*s);
		Mat outim = Mat::zeros(h_show_wind,w_show_wind,CV_8UC3);
		resize(im,outim,outim.size());
		int x0,y0,w0,h0;
		for (int i=0;i<vec_detobj.size();i++)
		{
			Rect rt0 = vec_detobj[i].box;
			Rect rt_tight0 = vec_detobj[i].box_tight;
			x0 = (int)myround(rt0.x*s);
			y0 = (int)myround(rt0.y*s);
			w0 = (int)myround(rt0.width*s);
			h0 = (int)myround(rt0.height*s);
			Rect rt(Point(x0,y0),Size(w0, h0));
			x0 = (int)myround(rt_tight0.x*s);
			y0 = (int)myround(rt_tight0.y*s);
			w0 = (int)myround(rt_tight0.width*s);
			h0 = (int)myround(rt_tight0.height*s);
			Rect rt_tight(Point(x0,y0),Size(w0, h0));

			rectangle(outim,rt_tight.tl(),rt_tight.br(),box_color,thickness);
			// then put the text
			std::sprintf(buf,"%s:%.2f",all_obj_cls[vec_detobj[i].id_label].c_str(),vec_detobj[i].match_score);
			text = buf;
			putText(outim, text, rt_tight.tl()+Point(0,-3), fontFace, fontScale,box_color);
			drawEdgePatch(outim, vec_detobj[i].mat_edge_NN_tr, rt, edge_color);
		}
		namedWindow( "Texture-less object detection",CV_WINDOW_KEEPRATIO | CV_WINDOW_AUTOSIZE);
		imshow("Texture-less object detection",outim);
	}
	return;
}

 /*
  * texture-less object detection.
  * INPUT:
  *    im             -- original image matrix
  * OUTPUT:
  *    if detection goes well (The results are saved in vec_detobj)
  * The result can be shown
  *    1. on the image:
  *       showDetObjs(im,Scalar(0,255,0),Scalar(255,255,255));
  *    2. on the screen with texts
  *       dispDetObjs();
  *    3. get the result data:
  *       vector<DetObj> vec_results = getDetObjs();
  */
bool CTLdetector::detect(Mat im)
{
	if (BEPRINT)
		printf(" ***Started detecting...\n");
    double t_findBestCandi = 0.0,t_matchRatioDisOri=0.0,time_findBestCandi,time_matchRatioDisOri;
    int n_run_match=0,n_count_run_match;

	int *pinds_grouppt = new int[grouppt_para.n_pt*grouppt_para.n_table];
	caiMat tmpmat(grouppt_para.n_pt,grouppt_para.n_table,CV_32S);
	tmpmat = mat_inds_grouppt;
	tmpmat.convert2MatlabMat(pinds_grouppt);
	((Mat)tmpmat).release();

	//unsigned char *pcur_edge_map;
	float *pte_feats_dis, *pte_feats_ori;
	vector<Point> vec_lu_pts;
	int n_te,i_s, i_asp,n_count_this_s,p;
	float s,asp;
	double start;
	double *t_steps = new double[5];
	memset(t_steps,0,sizeof(double)*5);
	vector<DetObj> vec_candi_detobj;

	//check image size, if it is different from 640*480, then fratio.
	double fratio = 1.0;
	int max_cols_rows = (im.cols>im.rows)?im.cols:im.rows;
	if (max_cols_rows != test_para.max_cols_rows)
		fratio = (double)test_para.max_cols_rows/(double)max_cols_rows;
	for (i_s=test_para.min_scale;i_s<=test_para.max_scale;i_s++)
	{
		n_count_this_s = 0;
		s=(float)(pow(test_para.scale_factor,i_s)*fratio); //consider the original size is not 640*480
		if (BEPRINT)
			printf("s=%.2f...\n",s);
		for (i_asp=test_para.min_aspect;i_asp<=test_para.max_aspect;i_asp++)///////////////////
		{
			asp = (float)pow(test_para.aspect_factor,i_asp);
			if (BEPRINT)
				printf("s=%.2f,asp=%.2f\n",s,asp);
			//////////////////////////////////////////////////////
			//step1: resize the image
			//////////////////////////////////////////////////////
			if (BEPRINT)
				printf("  //step1: resize the image\n");
			if (BETIME)
				start = cv::getTickCount();
			Mat im_cur_rgb,im_;
			if (be_blur)
				blurBeforeResize(im,im_,s);
			else
				im_ = im.clone();
			resize(im_,im_cur_rgb,Size(),s,s/asp);// Size(cur_w,cur_h));
			im_.release();
			if (BETIME)
				t_steps[0] = t_steps[0]+getTickCount()-start;

			//////////////////////////////////////////////////////
			//step2: edge detect, odt, generate features
			//////////////////////////////////////////////////////
			if (BEPRINT)
				printf("  //step2: edge detect, odt, generate features\n");
			if (BETIME)
					start = getTickCount();
			Mat mat_matches,mat_te_dis_im, mat_te_ori_im,mat_te_edge_im;
			n_te = generateTestFeatures(im_cur_rgb,
				&mat_te_edge_im, &mat_te_dis_im, &mat_te_ori_im, vec_lu_pts,
				&pte_feats_dis, &pte_feats_ori);
			im_cur_rgb.release();
			if (BETIME)
				t_steps[1] = t_steps[1]+getTickCount()-start;


			//////////////////////////////////////////////////////
			//step3: matches from table
			//////////////////////////////////////////////////////
			if (BEPRINT)
				printf("  //step3: matches from table\n");
			if (BETIME)
					start = getTickCount();
			switch(table_method)
			{
				case TABLE_PLAIN:
					matchFromPlainTables(ptables, n_tr,pte_feats_dis, pte_feats_ori,inds_para.n_feat_pt,
							n_te, quantize_para.n_bin_dis, quantize_para.n_bin_ori,
							quantize_para.MAX_range_dis, quantize_para.MAX_range_ori, be_ori_pi,mat_matches);
					break;
				case TABLE_GROUPPT:
					//ntable_size = (int)pow(quantize_para.n_bin_dis*quantize_para.n_bin_ori,grouppt_para.n_pt)*grouppt_para.n_table;
					matchFromOrgTables(ptables, n_tr,pinds_grouppt,grouppt_para.n_pt, grouppt_para.n_table,
							pte_feats_dis, pte_feats_ori,inds_para.n_feat_pt, n_te,
							quantize_para.n_bin_dis, quantize_para.n_bin_ori,
							quantize_para.MAX_range_dis, quantize_para.MAX_range_ori, be_ori_pi,mat_matches);
					break;
				case TABLE_CLUSTER:
					cout<<"ERROR: TABLE_CLUSTER is not supported so far."<<endl;
					return false;
					break;
				default:
					cout<<"ERROR: Only TABLE_PLAIN (0) and TABLE_Grouppt (1) are supported so far."<<endl;
					return false;
			}

			delete []pte_feats_dis;
			delete []pte_feats_ori;
			if (BETIME)
				t_steps[2] = t_steps[2]+getTickCount()-start;

			//////////////////////////////////////////////////////
			//step4: verification with OCMvip
			//////////////////////////////////////////////////////
			if (BEPRINT)
				printf("  //step4: verification with OCMvip\n");
			if (BETIME)
					start = getTickCount();
			vector<DetObj> vec_cur;
			if (table_method==TABLE_GROUPPT)
				vec_cur=NNVerifyChamfer(mat_matches, grouppt_para.nmin_matches,mat_te_dis_im, mat_te_ori_im,
					s, asp,	vec_lu_pts, vec_mat_tr_edge, vec_mat_tr_ori,mat_tr_rect_tight, vec_tr_id_labels,
					be_ori_pi, chamfer_para,
					n_obj_cls, time_findBestCandi,time_matchRatioDisOri,n_count_run_match);
			else
				vec_cur=NNVerifyChamfer(mat_matches, test_para.nmin_matches,mat_te_dis_im, mat_te_ori_im,
						s, asp,	vec_lu_pts, vec_mat_tr_edge, vec_mat_tr_ori, mat_tr_rect_tight, vec_tr_id_labels,
					be_ori_pi, chamfer_para,
					n_obj_cls, time_findBestCandi,time_matchRatioDisOri,n_count_run_match);
			for (p=0;p<vec_cur.size();p++)
				vec_candi_detobj.push_back(vec_cur.at(p));
			n_count_this_s +=(int)vec_cur.size();
			vec_cur.clear();
			mat_matches.release();
			vec_lu_pts.clear();
			mat_te_edge_im.release();
			mat_te_dis_im.release();
			mat_te_ori_im.release();

			if (BETIME_DETAIL)
			{
				t_findBestCandi+=time_findBestCandi;
				t_matchRatioDisOri+=time_matchRatioDisOri;
				n_run_match +=n_count_run_match;
			}
			if (BETIME)
				t_steps[3] = t_steps[3]+getTickCount()-start;
		}
		if (BEPRINT)
			printf("    %d candidates detected.\n",n_count_this_s);

	}
	//////////////////////////////////////////////////////
	//step5: refine the results
	//////////////////////////////////////////////////////
	if (BETIME)		start = getTickCount();
	refineObj(vec_candi_detobj);
	if (BETIME)		t_steps[4] = getTickCount()-start;
	vec_candi_detobj.clear();
	delete []pinds_grouppt;
	delete []t_steps;
	if (BETIME)
	{
		int i=0;
		printf("  //step%d: %.3f s for resizing the image.\n",i+1,t_steps[i]/(double)getTickFrequency());
		i=1;
		printf("  //step%d: %.3f s for edge detection, ODT, generating features.\n",i+1,t_steps[i]/(double)getTickFrequency());
		i=2;
		printf("  //step%d: %.3f s for table matching.\n",i+1,t_steps[i]/(double)getTickFrequency());
		i=3;
		printf("  //step%d: %.3f s for verification with improved OCM.\n",i+1,t_steps[i]/(double)getTickFrequency());
		i=4;
		printf("  //step%d: %.3f s for refining the results.\n",i+1,t_steps[i]/(double)getTickFrequency());
		
	}
	if (BETIME_DETAIL)
	{
		printf("\nTotally:\n");
		printf("findBestCandi cost:%.4f s.\n",t_findBestCandi);
		printf("matchRatioDisOri cost:%.4f s (run %d times).\n ",t_matchRatioDisOri,n_run_match);
	}
	if (BEPRINT)
		printf(" ***Detecting Done\n");
	return true;
};

/*
 * To refine the detected objects, mainly non-maxima suppression
 * INPUT:
 *    vec_candi_detobj -- all the candidates
 * OUTPUT:
 *    how many detected objects (The results are saved in vec_detobj)
 */
int CTLdetector::refineObj(vector<DetObj> & vec_candi_detobj)
{
	int  i,j;
	for (i=0;i<vec_detobj.size();i++){
    	 vec_detobj[i].mat_edge_NN_tr.release();
	}
	vec_detobj.clear();
	int n = vec_candi_detobj.size();
	if (n==0)
		return 0;

	DetObj obj1,obj2;
	Rect rt_intersect;
	int area_intersect,area_union;
	int *p_area = new int[n];
	float *p_w_score = new float[n];
	for (i=0;i<n;i++)
	{
		obj1 = vec_candi_detobj.at(i);
		p_area[i] = obj1.box_tight.area();
		p_w_score[i] = obj1.match_score*log((double)(p_area[i]));//// weight the score
	}

	//////////////////////////////
	// compute each boxes overlap
	/////////////////////////////
	Mat mat_overlap(n,n,CV_32FC1,Scalar(1.0f));
	for (i=0;i<n;i++)
	{
		obj1 = vec_candi_detobj.at(i);
		for (j=0;j<i;j++)
		{
			obj2 = vec_candi_detobj.at(j);
			rt_intersect = obj1.box_tight & obj2.box_tight;
			area_intersect = rt_intersect.area();
			if (area_intersect<1)
				mat_overlap.at<float>(i,j) = 0.0f;
			else
			{
				area_union =  p_area[i]+p_area[j] - area_intersect;
				//mat_overlap.at<float>(i,j) = (area_intersect*1.0f)/area_union;				
				//if ((area_union-p_area[i]<area_union*0.02)||(area_union-p_area[j]<area_union*0.02))
				//	mat_overlap.at<float>(i,j) = 0.99f;
				if ((area_intersect*1.0f)/area_union > 0.18)
					mat_overlap.at<float>(i,j) = 0.99f;
				else
					mat_overlap.at<float>(i,j) = (area_intersect*1.0f)/(VL_MIN(p_area[i],p_area[j])*1.0f);
			}
		}
	}
	for (i=0;i<n;i++)
		for (j=i+1;j<n;j++)
			mat_overlap.at<float>(i,j) = mat_overlap.at<float>(j,i);

	/////////////////////////////
	// sort the weighted score
	/////////////////////////////
	Mat this_row,cur_dis_map;
	float *max_;
	int which_one;
	int n_remain = 0,n_remove = 0;
	while (n_remove<n)
	{
		max_ = max_element(p_w_score,p_w_score+n);
		which_one = abs(std::distance(max_, p_w_score));
		this_row = mat_overlap.row(which_one);
		obj1 = vec_candi_detobj.at(which_one);
		if (obj1.match_score>1.0f) obj1.match_score=0.99f; //if it is bigger than 1, set to 0.99.
		
		cur_dis_map = vec_mat_tr_dis.at(obj1.id_NN_tr);
		obj1.mat_edge_NN_tr = Mat::zeros(norm_para.h_fix,norm_para.w_fix,CV_8UC1);
		for (int ii=0;ii<norm_para.h_fix;ii++)
			for (int jj=0;jj<norm_para.w_fix;jj++)
				if (cur_dis_map.at<float>(ii,jj)<0.05) 
					obj1.mat_edge_NN_tr.at<uchar>(ii,jj) = 1; // generate the edges from distance map
		vec_detobj.push_back(obj1);
		n_remain++;
		for (i=0;i<n;i++)
		{
			if (this_row.at<float>(i)>test_para.th_overlap_out) //overlap > th
			{
				n_remove++;
				mat_overlap.at<float>(which_one,i) = 0.0f;
				mat_overlap.at<float>(i,which_one) = 0.0f;
				p_w_score[i] = 0.0f;
			}
		}
	}
	/////////////////////////////
	// remove
	/////////////////////////////
	vec_candi_detobj.clear();
	cur_dis_map.release();//////
	mat_overlap.release();
	delete []p_area;
	delete []p_w_score;
	return n_remain;
}
/*
 * Edge detection, ODT, remove less-edge sliding windows, generate 36-pt features
 * INPUT:
 *  im_cur            -- the current image matrix
 *  *out_mat_edge_map -- output edge map (0,1)
 *  *out_mat_dis_map  -- output distance map
 *  *out_mat_ori_map  -- output orientation map
 *  pout_lu_pts       -- output the left-up locations of valid sliding windows
 *  pout_feat_dis     -- output the distance features for the 36-pt
 *  pout_feat_ori     -- output the orientation features for the 36-pt
 *  OUTPUT:
 *  the number of valid sliding windows
 */
int CTLdetector::generateTestFeatures(Mat im_cur,
		Mat *out_mat_edge_map,Mat *out_mat_dis_map, Mat *out_mat_ori_map,
		vector<Point> &pout_lu_pts, float **pout_feat_dis,float **pout_feat_ori)
{
	//////////////////////////////////////////////////////
	//step1: edge, Oriented Distance Transform
	//////////////////////////////////////////////////////
	/// Convert it to gray
	Mat im_cur_gray;
	if (im_cur.channels()>1)//rgb image
		cvtColor( im_cur,im_cur_gray, CV_RGB2GRAY );
	else
		im_cur_gray = im_cur.clone();

	Canny(im_cur_gray,*out_mat_edge_map,test_para.canny_tlow, test_para.canny_thigh);	// canny output: 0 255
	*out_mat_edge_map = (*out_mat_edge_map)/255;   // edge is 0-1

	/// Generate grad_x and grad_y
	Mat grad_x, grad_y;
	Sobel( im_cur_gray, grad_x, CV_32F, 1, 0, 3);//, 3, scale, delta, BORDER_DEFAULT );
	Sobel( im_cur_gray, grad_y, CV_32F, 0 ,1, 3);

	Mat out_mat_ind_map;
	odt(*out_mat_edge_map,grad_x,grad_y,be_ori_pi, out_mat_dis_map, out_mat_ori_map, &out_mat_ind_map);
	out_mat_ind_map.release();
	grad_x.release();
	grad_y.release();
	im_cur_gray.release();

	//////////////////////////////////////////////////////
	//step2: remove the too few edge point windows
	//////////////////////////////////////////////////////
	removeLessEdgeWind(*out_mat_edge_map,pout_lu_pts);

	//////////////////////////////////////////////////////
	//step3: generate the features.
	//////////////////////////////////////////////////////
	int n_wind = pout_lu_pts.size();
	if (BEPRINT)
		printf("    %d sliding windows generated.\n",n_wind);
	*pout_feat_dis = new float[inds_para.n_feat_pt*n_wind];
  	*pout_feat_ori = new float[inds_para.n_feat_pt*n_wind];
	int x_global,y_global,pos;//,ntmp2,
	for (int i=0,pos=0;i<n_wind;i++)
	{
		for (int j=0;j<inds_para.n_feat_pt;j++,pos++)
		{
			x_global = inds_para.inds_feat_pt[j].x+pout_lu_pts[i].x;
			y_global = inds_para.inds_feat_pt[j].y+pout_lu_pts[i].y;
			(*pout_feat_dis)[pos] = (*out_mat_dis_map).at<float>(y_global,x_global);
			(*pout_feat_ori)[pos] = (*out_mat_ori_map).at<float>(y_global,x_global);
		}
	}
	return n_wind;
};
/*
 * remove the less edge windows than test_para.min_edge_pt
 * and bigger than test_para.max_edge_pt
 */
void CTLdetector::removeLessEdgeWind(Mat im_edge,vector<Point> &pout_lu_pts)
{
	//////////////////////////////////////
	//step1: integral image
	//////////////////////////////////////
	int w = im_edge.cols;
	int h = im_edge.rows;
	Mat sum_edge;
    integral(im_edge, sum_edge);

	//////////////////////////////////////
	//step2: count the edge points
	//////////////////////////////////////
	//Point cur_lu;
	int i,j,ii,jj,npt_edge;
	int n_win_x =(int)floor((double)((w-norm_para.w_fix)/test_para.sliding_step));
	int n_win_y =(int)floor((double)((h-norm_para.h_fix)/test_para.sliding_step));

	for (i=0;i<n_win_y;i++)
	{
		ii = i*test_para.sliding_step;
		for (j=0;j<n_win_x;j++)
		{
			jj = j*test_para.sliding_step;
			npt_edge = sum_edge.at<int>(ii+norm_para.h_fix,jj+norm_para.w_fix)
							+ sum_edge.at<int>(ii,jj)
							- sum_edge.at<int>(ii,jj+norm_para.w_fix)
							- sum_edge.at<int>(ii+norm_para.h_fix,jj);
 
			if ((npt_edge>test_para.min_edge_pt) && (npt_edge<test_para.max_edge_pt))//output the windows with more edge points
				pout_lu_pts.push_back(Point(jj,ii));
		}
	}
	sum_edge.release();
	return;
}

void CTLdetector::blurBeforeResize(Mat & im0, Mat & im, float s)
{
	double sigma = 0.7/s;
	int ksize = (int)ceil(5.3*sigma);
	if ((ksize%2)==0)
		ksize = ksize-1;
	GaussianBlur(im0,im,cv::Size(ksize,ksize),sigma,sigma);
	return;
}
/*
 * get the value of the detected objects
 */
vector<DetObj> CTLdetector::getDetObjs()
{
	vector<DetObj> result(vec_detobj);
	return result;
}
/*
 * show the description of each object on the screen
 */
void CTLdetector::dispDetObjs()
{
	int n_obj = vec_detobj.size();
	if (n_obj>0)
		cout<<" //==================== Detection Report ========================="<<endl;
	for (int i=0;i<n_obj;i++)
	{
		printf(" // OBJ%2d: %10s [%4d %4d %4d %4d], score: %.2f, id_NN_tr: %5d\n",i,all_obj_cls[vec_detobj.at(i).id_label].c_str(),
				vec_detobj.at(i).box_tight.x, vec_detobj.at(i).box_tight.y,
				vec_detobj.at(i).box_tight.width, vec_detobj.at(i).box_tight.height,
				vec_detobj.at(i).match_score,vec_detobj.at(i).id_NN_tr);
	}
	return;
}



