
#ifndef VERIFYCHAMFER_HPP_
#define VERIFYCHAMFER_HPP_
#undef __GXX_EXPERIMENTAL_CXX0X__

#include <float.h>
#include <memory.h>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <time.h>

inline int myround(double value)
{
	return (int)(value + 0.5);
}

//#define (float)CV_PI 3.1416926
#define PI_2 (float)CV_PI*2
typedef struct{
	int nid_tr;
	int nid_te;
	int nmatch;
}best_candi;

/*
 * Find the best candidate for each category
 * above a match threshold (nmin_matches)
 *
 * if find any candidate, then output true, otherwise false
 */
bool findBestCandi(Mat &mat_match, int nid_te,  int nmin_matches,
		vector<int>& vec_tr_id_labels, int n_labels,
		best_candi * out_best_candi)
{	// initialize
	memset(out_best_candi,-1,sizeof(best_candi)*n_labels);
	int id_cur_label,i_count=0;
	unsigned char cur_match;
	int which_tr, n_tr = mat_match.cols;

	//const unsigned char * p_thisrow = mat_match.ptr<unsigned char>(nid_te,0); //for higher opencv version
	for (int which_tr=0;which_tr<n_tr;which_tr++)
	{
		//cur_match = p_thisrow[which_tr];                                  //for higher opencv version
		cur_match = mat_match.at<unsigned char>(nid_te,which_tr);           //for lower opencv version
		if (cur_match>=nmin_matches)
		{
				id_cur_label = vec_tr_id_labels[which_tr]; //no matlab index
				if ((out_best_candi[id_cur_label].nid_te<0)||(out_best_candi[id_cur_label].nmatch< cur_match))
				{//empty or smaller match, update it
					out_best_candi[id_cur_label].nid_te = nid_te;
					out_best_candi[id_cur_label].nid_tr = which_tr;
					out_best_candi[id_cur_label].nmatch = cur_match;
					i_count++;
				}
		}
	}
	return (i_count>0);
}

/*
 * Compute the similarity score (OCM01)
 * It is slightly changed by a linear function (y=kx+b)
 * to give penalty to the small edge object.
 */
float matchRatioDisOri(Mat edge_temp, Mat orient_temp, Mat dist_test, Mat orient_test,
                       bool be_ori_pi,double th_dist, double th_orient,
                       double k_linear, double b_linear)
{
	double T;
	if (be_ori_pi)
		T = (float)CV_PI;
	else
		T = PI_2;
  	int n_edge = 0;
    int n_match = 0;
	float diff;
	for (int j=0;j<edge_temp.rows;j++)
	{
		for (int i=0;i<edge_temp.cols;i++)
		{
			if (edge_temp.at<unsigned char>(j,i)>0)//edge 0, 1
			{
				n_edge++;
				if (dist_test.at<float>(j,i)<th_dist)
				{
					diff = fabs(orient_test.at<float>(j,i)-orient_temp.at<float>(j,i));
					if ((diff<th_orient)||(T-diff<th_orient))//circular distance
						n_match++;
				}
			}
		}
	}

	/////////////////////////////////////////////////
	// add some penalty to small edge object.
	float f_edge = n_edge*1.0f*k_linear + b_linear; //////
	/////////////////////////////////////////////////
 	return float(n_match/f_edge);
}
/*
 * convert the current sliding window to the location on the original image
 */
Rect convertRectToOrg(Point lu_pt,int norm_w, int norm_h,float s, float asp)
{
	int x0,y0,w0,h0;
	x0 = (int)myround(lu_pt.x/s);
	y0 = (int)myround(lu_pt.y*asp/s);
	w0 = (int)myround(norm_w/s);
	h0 = (int)myround(norm_h*asp/s);
	return Rect(Point(x0,y0),Size(w0, h0));
}
/*
 * Verification by OCM01
 * INPUT:
 *   mat_matches     -- unsighed char matrix for the table output
 *                      M(i,j) means how many matched keypoints of the i-th test patch
 *                      and the j-th training patch
 *                      NOTE: the input mat_matches is n_te*n_tr size
 *   nmin_matches    -- threshold of matched keypoints (suggest: 12 for 36 points)
 *   mat_te_dis_im   -- the distance map of the test image
 *   mat_te_ori_im   -- the orientation map of the test image
 *   s               -- the scale ratio of the current image to the original (for output bb)
 *   asp             -- the aspect ratio of the current image to the original (for output bb)
 *   vec_te_lu       -- the left-up corners of all sliding windows
 *   vec_mat_tr_edge -- edge maps for all training patches (suggest vip edges)
 *   vec_mat_tr_ori  -- orientation maps for all training patches
 * mat_tr_rect_tight -- the tight rects for all training patches  (for output bb)
 * vec_tr_id_labels  -- label ids for all training patches
 *   be_ori_pi       -- 1: orientation is [0 pi], 0: [-pi pi]
 *   chamfer_para    -- OCM-realted parameters
 *   n_labels        -- How many classes (or training objs)
 * time_findBestCandi,
 * time_matchRatioDisOri,
 * n_count_run_match  --for showing some details
 */
std::vector<DetObj> NNVerifyChamfer(Mat mat_matches, int nmin_matches, Mat mat_te_dis_im,
    Mat mat_te_ori_im, float s, float asp,vector<Point> & vec_te_lu,
    vector<Mat> & vec_mat_tr_edge, vector<Mat> & vec_mat_tr_ori,Mat mat_tr_rect_tight,
    vector<int> & vec_tr_id_labels,  bool be_ori_pi,ParaChamfer chamfer_para, int n_labels,
    double &time_findBestCandi, double &time_matchRatioDisOri,int &n_count_run_match)
{
	int w, h;
	w = mat_te_dis_im.cols;
	h = mat_te_dis_im.rows;
	int norm_h = vec_mat_tr_edge.at(0).rows;
	int norm_w = vec_mat_tr_edge.at(0).cols;
	int n_tr = vec_mat_tr_edge.size();
	int n_te = vec_te_lu.size();
	int norm_hw = norm_h*norm_w;
	std::vector<DetObj> vec_out;
	DetObj cur_candi;
	int i,j,k,x_leftup,y_leftup,which_tr;
	bool have_candi= false, flag_qual;
	best_candi * pbest_candis = new best_candi[n_labels];

	float fmatch,max_match_score=0.0f,ftmp;
	Mat cur_edge_temp;
	Mat cur_ori_temp;
	////////////////////////////////////////////////////////////////////////
	clock_t start;
	n_count_run_match=0;
	time_findBestCandi = 0;
	time_matchRatioDisOri = 0;
//#pragma omp parallel for
	for (i=0;i<n_te;i++)
	{
		// find qualified candidates, each label has at most one.
		////////////////////////////////////////////////////////////////////////
		if (BETIME_DETAIL)			start = clock();
		////////////////////////////////////////////////////////////////////////
		have_candi = findBestCandi(mat_matches, i, nmin_matches, vec_tr_id_labels, n_labels,
				pbest_candis);
		////////////////////////////////////////////////////////////////////////
		if (BETIME_DETAIL)
		{
			time_findBestCandi = time_findBestCandi+((double)clock() - start); // count time
			start = clock();
		}
		////////////////////////////////////////////////////////////////////////

		if (have_candi)
		{
			x_leftup = vec_te_lu[i].x;     //No matlab index
			y_leftup = vec_te_lu[i].y;     //No matlab index

			Mat cur_dis_test=mat_te_dis_im(Rect(x_leftup,y_leftup,norm_w,norm_h));
			Mat cur_ori_test=mat_te_ori_im(Rect(x_leftup,y_leftup,norm_w,norm_h));
			// find the highest match score one (>th_match_score)
			flag_qual = false;
			max_match_score= (float)chamfer_para.th_match_score; // initiate it
			for(j=0;j<n_labels;j++)
			{
				if (pbest_candis[j].nmatch<0)//empty
					continue;

				which_tr = pbest_candis[j].nid_tr;
				cur_edge_temp = vec_mat_tr_edge.at(which_tr);
				cur_ori_temp = vec_mat_tr_ori.at(which_tr);
				fmatch = matchRatioDisOri(cur_edge_temp, cur_ori_temp, cur_dis_test, cur_ori_test,
				                       be_ori_pi, chamfer_para.th_dis, chamfer_para.th_ori,
				                       chamfer_para.k_linear, chamfer_para.b_linear);

				////////////////////////////////////////////////////////////////////////
				n_count_run_match++; //count how many times running matchRatioDisOri()
				////////////////////////////////////////////////////////////////////////

				if (fmatch>max_match_score){//update
					int w_rect_tight = mat_tr_rect_tight.at<int>(which_tr,2);
					int h_rect_tight = mat_tr_rect_tight.at<int>(which_tr,3);
					Point offset_tight(mat_tr_rect_tight.at<int>(which_tr,0),mat_tr_rect_tight.at<int>(which_tr,1));
					cur_candi.match_score = fmatch;
					cur_candi.id_NN_tr = which_tr;
					cur_candi.id_label = j;
					cur_candi.box = convertRectToOrg(vec_te_lu[i],norm_w, norm_h, s, asp);
					cur_candi.box_tight = convertRectToOrg(vec_te_lu[i]+offset_tight,w_rect_tight, h_rect_tight, s, asp);
					//cur_candi.mat_edge_NN_tr = cur_edge_temp.clone();
					//cur_candi.mat_edge_NN_tr = NULL;
					max_match_score = fmatch;
					flag_qual = true;
				}

			}
			if (flag_qual)
				vec_out.push_back(cur_candi);

		}
		////////////////////////////////////////////////////////////////////////
		if (BETIME_DETAIL)
			time_matchRatioDisOri = time_matchRatioDisOri+((double)clock() - start); // count time
		////////////////////////////////////////////////////////////////////////

	}

	delete []pbest_candis;
	if (BETIME_DETAIL)
	{
		time_findBestCandi = time_findBestCandi / CLOCKS_PER_SEC;
		time_matchRatioDisOri = time_matchRatioDisOri / CLOCKS_PER_SEC;
		printf("  run    findBestCandi %4d times, spend %.4f s.\n",n_te,time_findBestCandi);
		printf("  run matchRatioDisOri %4d times, spend %.4f s.\n",n_count_run_match,time_matchRatioDisOri);
	}
	////////////////////////////////////////////////////////////////////////
	return vec_out;
}

#endif

