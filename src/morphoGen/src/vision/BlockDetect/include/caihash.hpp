/* added by caicai
Algorithms for building table and matching from table
27/03/2013 */
#pragma once

#ifndef CAIHASH_HPP_
#define CAIHASH_HPP_
#undef __GXX_EXPERIMENTAL_CXX0X__


#include <memory.h>
#include <math.h>
#include <vector>
#include <time.h>/////////////////////////////////////////////////////////////
#define VL_MAX(x,y) (((x)>(y))?(x):(y))


unsigned int fnv_hash (void const *key, int len)/* hash function */
{
  unsigned char const *p = (unsigned char const *)key;
  unsigned int h = 2166136261U ;
  int i;

  for ( i = 0; i < len; i++ )
    h = ( h * 16777619 ) ^ p[i];

  return h;
}

int is_null (unsigned char const * x, int n)
{
  int i ;
  for (i = 0 ; i < n ; ++i) {
    if (x[i]) return 0 ;
  }
  return 1 ;
}

int is_equal (unsigned char const * x, unsigned char const * y, int n)
{
  int i ;
  for (i = 0 ; i < n ; ++i) {
    if (x[i] != y[i]) return 0 ;
  }
  return 1 ;
}

void cpy (unsigned char * x, unsigned char const * y, int n)
{
  int i ;
  for (i = 0 ; i < n ; ++i){
    /*    mexPrintf("cpy:%d %d\n",x[i],y[i]);*/
    x[i] = y[i] ;
  }
}
void quantizeDis(float *pfeat_dis, int D, 
	         double MAX_range_dis, int n_bin_dis, unsigned char *pout_int)
{
	int i;
	unsigned char id_dis;
	for (i=0;i<D;i++)
	{
		id_dis = (unsigned char)floor(pfeat_dis[i]/MAX_range_dis * n_bin_dis);
		if (id_dis>=n_bin_dis)
			id_dis = n_bin_dis-1;	
		if (id_dis<0)
			id_dis = 0;		
		pout_int[i] = id_dis;
	}	
	return;
}

void quantizeOri(float *pfeat_ori, int D, 
	         double MAX_range_ori, int n_bin_ori,
	         bool be_ori_pi, unsigned char *pout_int)
{
	int i, id_ori;
	if (be_ori_pi)
	{
		for (i=0;i<D;i++)
		{			
			id_ori = (unsigned char)floor(pfeat_ori[i]/MAX_range_ori * n_bin_ori);
			if (id_ori>=n_bin_ori)
				id_ori = n_bin_ori-1;
			if (id_ori<0)
				id_ori = 0;
			pout_int[i] =  id_ori;
		}
	}
	else
	{
		for (i=0;i<D;i++)
		{
			id_ori = (unsigned char)floor((pfeat_ori[i]+(float)CV_PI)/MAX_range_ori * n_bin_ori);
			if (id_ori>=n_bin_ori)
				id_ori = n_bin_ori-1;
			if (id_ori<0)
				id_ori = 0;
			pout_int[i] =  id_ori;
		}
	}
	return;
}
/* -----------------------------------------------------------------
Input: 
output: n_id -- dim of ids.
   * -------------------------------------------------------------- */
int singleHash(unsigned char const * x,int N, int ndims, int K, 
		unsigned int ** ph,unsigned char ** pid, unsigned int ** pnext)
{  
  	unsigned int *pout_h    = (unsigned int *)calloc(K, sizeof(int));
  	unsigned int *pout_next = (unsigned int *)calloc(K, sizeof(int));
  	unsigned char *pout_id    = (unsigned char *)calloc(K * ndims, sizeof(unsigned char));
  	memset (pout_h,     0, K * sizeof(unsigned int)       ) ;
  	memset (pout_next,  0, K * sizeof(unsigned int)       ) ;
  	memset (pout_id  ,  0, K * sizeof(unsigned char) * ndims) ;
	int n_id = K,res=K;
 	unsigned int i, h1, h2 ,j, p = 0;
  	for (i = 0 ; i < N ; ++i) {    	/* hash */
    		/* cannot hash null labels */
    		if (is_null (x + i * ndims, ndims)) {
       			printf("ERROR: One column of X is null (all 0s).");
      			return -1;
    		}

    		h1 = fnv_hash(x + i * ndims, ndims) % K ;
    		h2 = h1 | 0x1 ; /* this needs to be odd */

    		/* search first free or matching position */
    		p = h1 % K ;
    		for (j = 0 ; j < K ; ++j) {
      			if (is_null (pout_id + p * ndims,                ndims) ||
          			is_equal(pout_id + p * ndims, x + i * ndims, ndims)) break ;
      			h1 += h2 ;
      			p = h1 % K ;
    		}

   		/* if after scanning the K elements in the hash table an empty/matching
      		bucket is still not found, start using next to go into the overflow table */
    		while (! is_null (pout_id + p * ndims,                ndims) &&
           	       ! is_equal(pout_id + p * ndims, x + i * ndims, ndims)) {
      			if (pout_next[p] > res) {
      				printf("ERROR:An element of NEXT is greater than the size of the table") ;
      				return -1;
      			}
      			/* append */
      			if (pout_next[p] == 0) {
        			if (n_id >= res) {
          				size_t res_ = res + VL_MAX(res / 2, 2) ;
          				pout_h    = (unsigned int *)realloc(pout_h,    res_ * sizeof(unsigned int)       ) ;
          				pout_next = (unsigned int *)realloc(pout_next, res_ * sizeof(unsigned int)       ) ;
          				pout_id   = (unsigned char *)realloc(pout_id,   res_ * sizeof(unsigned char) * ndims) ;
          				memset (pout_h    + res,         0, (res_ - res) * sizeof(unsigned int)       ) ;
          				memset (pout_next + res,         0, (res_ - res) * sizeof(unsigned int)       ) ;
          				memset (pout_id   + res * ndims, 0, (res_ - res) * sizeof(unsigned char) * ndims) ;
          				res = res_ ;
        			}
        			pout_next [p] = ++ n_id ;
      			}
      			p = pout_next [p] - 1 ;
    		}

    		/* accumulate */
    		pout_h[p] += 1 ;
    		cpy(pout_id + p * ndims, x + i * ndims, ndims) ;
    		//mexPrintf("p %d dims %d i %d N %d\n ", p, ndims, i, N) ;

  	}

	pout_h    = (unsigned int *)realloc(pout_h,    n_id * sizeof(unsigned int)) ;
	pout_next = (unsigned int *)realloc(pout_next, n_id * sizeof(unsigned int)       ) ;
        pout_id   = (unsigned char *)realloc(pout_id,    n_id * sizeof(unsigned char) * ndims) ;
	*ph = pout_h;
	*pnext = pout_next;
	*pid= pout_id;
	return n_id;
}
/* hash find the id*/
void singleHashFind(unsigned char const *id, unsigned int const *next, int K, unsigned char const *x, int ndims,int N, unsigned int *pout_sel)
{
	unsigned int h1, h2, j, p,i;
	for (i = 0 ; i < N ; ++i) {
  		/*printf("i=%d\n",i);*/
    		p = 0 ;

    		if (is_null (x + i * ndims, ndims)) {
      			*pout_sel++ = 0 ;
      			continue ;
    		}

    		h1 = fnv_hash(x + i * ndims, ndims) % K ;
    		h2 = h1 | 0x1 ; // this needs to be odd 

    		// search first free or matching position 
    		p = h1 % K ;
    		for (j = 0 ; j < K ; ++j) {
      			if (is_null (id + p * ndims,                ndims) ||
          			is_equal(id + p * ndims, x + i * ndims, ndims)) break ;
      			h1 += h2 ;
      			p = h1 % K ;
    		}

    		// handle extended table 
    		while (! is_null (id + p * ndims,                ndims) &&
           		! is_equal(id + p * ndims, x + i * ndims, ndims)) {
      			if (next[p] == 0) break ;
      			p = next [p] - 1 ;
    		}
 
    		if (is_equal(id + p * ndims, x + i * ndims, ndims))      			
      			*pout_sel++ = p + 1 ;  //// matlab index //////
		else// not found 
     			*pout_sel++ = 0 ;

  	}
	return;
}
/* hash find the id, comparing one by one.
 it is much much slower than the above one*/
void singleHashFindBrute(unsigned char const *id, int n_id, unsigned char const *x, int ndims,int N, unsigned int *pout_sel)
{
	unsigned int h1, h2, j, p,i;
  	bool flag;
	for (i = 0 ; i < N ; ++i) 
	{
		flag=false;
		for (j=0; j< n_id; j++)
		{
			if (is_null (id + j * ndims,ndims)){
				continue;}

			flag = is_equal(id+j*ndims, x+i*ndims,ndims);
			if (flag)
			{
				*pout_sel++ = j+1;  //// matlab index //////
				break;
			}
		}
		if (!flag) // not found
			*pout_sel++ = 0;
	}
	return;
}

// select n_pt points in the pint_dis_ori
void generateX(unsigned char *pint_dis_ori,int D, int n, 
	int *pinds_select, int n_pt,
	unsigned char * pout_X)
{
	int i,j,jj,base_count1,base_count2,pos;
	for (i=0,pos=0;i<n;i++)
	{
		//base_count1 = i*n_pt;
		base_count2 = i*D;
		for (j=0;j<n_pt;j++,pos++)
		{
			jj = pinds_select[j];  ///// n0 matlab index ///////
			pout_X[pos] = pint_dis_ori[base_count2+jj];//[j+base_count1]
		}
	}
	return;
}
//quantize all the features
//Input: nD -- is the element numbers for pfeats_dis and *_ori. nD = n*D
void quantizeDisOri(float *pfeats_dis, float *pfeats_ori,int nD, int n_bin_dis, int n_bin_ori,
		double MAX_range_dis, double MAX_range_ori, bool be_ori_pi, unsigned char *pint_dis_ori)
{
	int i;
	unsigned char *pint_dis = new unsigned char[nD];
	unsigned char *pint_ori = new unsigned char[nD];
	quantizeDis(pfeats_dis,nD,MAX_range_dis,n_bin_dis,pint_dis);
	quantizeOri(pfeats_ori,nD,MAX_range_ori,n_bin_ori,be_ori_pi,pint_ori);
	for (i=0;i<nD;i++)
		pint_dis_ori[i] = pint_dis[i]*n_bin_ori+pint_ori[i];  // start from 0, different from Hash table build.

	delete []pint_dis;
	delete []pint_ori;
}
/*
build the n_tables directly, without hashing.

before this function , the table should be allocate memory as:
pout_table = new std::vector<int>[(n_bin_dis*n_bin_ori)^n_pt*n_table];
*/
int buildOrgTablesFromInt(unsigned char *pint_dis_ori,int D, int n,int n_bin_dis, int n_bin_ori,
		int *pinds_pt_bundle, int n_pt, int n_table,
		std::vector<int> * pout_tables)
{
	//quantize all the features
	int i,j,p,nn_pt,n_bins_per_table, n_bins_per_pt, id_start_table,which_bin;
	//
	nn_pt = n*n_pt;
	unsigned char * x=new unsigned char[nn_pt];
	unsigned char * pcur_x;
	n_bins_per_pt = n_bin_dis*n_bin_ori;
	n_bins_per_table = (int)pow((double)n_bins_per_pt,n_pt);
	int *p_multi=new int[n_pt];
	for (p=0;p<n_pt;p++)
		p_multi[p] = (int)pow((double)n_bins_per_pt,p);
	for (i=0;i<n_table;i++)
	{
//printf("Table %d:n_bins_per_pt=%d, n_bins_per_table=%d\n",i,n_bins_per_pt,n_bins_per_table);
		// generate x
		generateX(pint_dis_ori, D,  n,
			  pinds_pt_bundle+i*n_pt, n_pt,x);
		// feed the table
		id_start_table = n_bins_per_table*i;
		pcur_x = x;
		for (j=0;j<n;j++)
		{
			which_bin = 0;
			for (p=0;p<n_pt;p++)
			{
				which_bin = which_bin+pcur_x[p]*p_multi[p];
			}
//printf("x[%d]=%d,%d,%d TO: bin%d\n",j,pcur_x[0],pcur_x[1],pcur_x[2], id_start_table+which_bin);
			pout_tables[id_start_table+which_bin].push_back(j); //// no matlab index ///
			pcur_x = pcur_x + n_pt;
		}

	}
	delete []x;
	delete []p_multi;
	return 1;
}

/*
build the n_tables directly, without hashing.

before this function , the table should be allocate memory as:
pout_table = new std::vector<int>[(n_bin_dis*n_bin_ori)^n_pt*n_table];
*/
int buildOrgTables(float *pfeats_dis, float *pfeats_ori,int D, int n,int n_bin_dis, int n_bin_ori,
		double MAX_range_dis, double MAX_range_ori, bool be_ori_pi,
		int *pinds_pt_bundle, int n_pt, int n_table,
		std::vector<int> * pout_tables)
{	
	//quantize all the features
	int nD = n*D;
	unsigned char *pint_dis_ori = new unsigned char[nD];
	quantizeDisOri(pfeats_dis,pfeats_ori, nD, n_bin_dis, n_bin_ori,
			 MAX_range_dis, MAX_range_ori, be_ori_pi, pint_dis_ori);
	//buil the table
	buildOrgTablesFromInt(pint_dis_ori,D, n, n_bin_dis, n_bin_ori,
			 pinds_pt_bundle, n_pt, n_table, pout_tables);
	delete []pint_dis_ori;
	return 1;
}
//
//NOTE: matches be n_te*n_tr
//
void matchFromOrgTablesFromInt(std::vector<int> *ptables,int n_tr,int *pinds_pt_bundle,int n_pt, int n_table,
		unsigned char *pint_dis_ori,int D, int n_te, int n_bin_dis, int n_bin_ori,
		Mat &out_mat_matches)
{
	int i,j,p,id_start,nD,nn_pt,n_bins_per_table, n_bins_per_pt, which_bin,nbin_size;
	nD=n_te*D;
    std::vector<int> cur_bin;
    out_mat_matches = Mat::zeros(n_te,n_tr,CV_8UC1); ///////////////////

    nn_pt = n_te*n_pt;
	unsigned char * x=new unsigned char[nn_pt];
	//unsigned char * pcur_x;
	n_bins_per_pt = n_bin_dis*n_bin_ori;
	n_bins_per_table = (int)pow((double)n_bins_per_pt,n_pt);
	int *p_multi=new int[n_pt];
	for (p=0;p<n_pt;p++)
		p_multi[p] = (int)pow((double)n_bins_per_pt,p);
	for (i=0;i<n_table;i++)
	{
//printf("Table %d:\n",i);
		// generate x
		generateX(pint_dis_ori, D,  n_te,
			  pinds_pt_bundle+i*n_pt, n_pt,x);
		id_start = i*n_bins_per_table;
		unsigned char *pcur_x = x;
		for (j=0;j<n_te;j++)
		{
			which_bin = 0;
			for (p=0;p<n_pt;p++)
			{
				which_bin = which_bin+pcur_x[p]*p_multi[p];
			}
			cur_bin = ptables[id_start+which_bin];
			nbin_size = cur_bin.size();
			for (p=0;p<nbin_size;p++)
				out_mat_matches.at<unsigned char>(j,cur_bin.at(p))++;///// no matlab index /////
			pcur_x = pcur_x + n_pt;

		}
	}
	delete []x;
	delete []p_multi;
	return;
}
/* Parallel test..
void matchFromOrgTablesFromInt(std::vector<int> *ptables,int n_tr,int *pinds_pt_bundle,int n_pt, int n_table,
		unsigned char *pint_dis_ori,int D, int n_te, int n_bin_dis, int n_bin_ori,
		Mat &out_mat_matches)
{
	int nD,nn_pt,n_bins_per_table, n_bins_per_pt;
	nD=n_te*D;
    out_mat_matches = Mat::zeros(n_te,n_tr,CV_8UC1); ///////////////////

    nn_pt = n_te*n_pt;

	//unsigned char * pcur_x;
	n_bins_per_pt = n_bin_dis*n_bin_ori;
	n_bins_per_table = (int)pow((double)n_bins_per_pt,n_pt);
	int *p_multi=new int[n_pt];
	for (int p=0;p<n_pt;p++)
		p_multi[p] = (int)pow((double)n_bins_per_pt,p);
#pragma omp parallel for
	for (int i=0;i<n_table;i++)
	{
//printf("Table %d:\n",i);
		// generate x
		unsigned char * x=new unsigned char[nn_pt];
		generateX(pint_dis_ori, D,  n_te,
			  pinds_pt_bundle+i*n_pt, n_pt,x);
		int id_start = i*n_bins_per_table;
		unsigned char *pcur_x = x;
		for (int j=0;j<n_te;j++)
		{
			int which_bin = 0;
			int p;
			for (int p=0;p<n_pt;p++)
			{
				which_bin = which_bin+pcur_x[p]*p_multi[p];
			}
			std::vector<int> cur_bin = ptables[id_start+which_bin];
			for (p=0;p<cur_bin.size();p++)
				out_mat_matches.at<unsigned char>(j,cur_bin.at(p))++;///// no matlab index /////
			pcur_x = pcur_x + n_pt;

		}
		delete []x;
	}
//#pragma omp barrier

	delete []p_multi;
	return;
}
*/
//
//NOTE: matches be n_te*n_tr
//
void matchFromOrgTables(std::vector<int> *ptables,int n_tr,int *pinds_pt_bundle,int n_pt, int n_table,
		float *pfeats_dis, float *pfeats_ori,int D, int n_te,
		int n_bin_dis, int n_bin_ori,double MAX_range_dis, double MAX_range_ori, bool be_ori_pi,
		Mat &out_mat_matches)
{	//////////////////////////////////////////////////////////////
	clock_t start;
	double time_quantize = 0;
	double time_lookup = 0;
	if (BETIME_DETAIL){
	start = clock();
	}
	//////////////////////////////////////////////////////////////

	// step1: quantize testing features
	int nD = n_te*D;
	unsigned char *pint_dis_ori = new unsigned char[nD];
	quantizeDisOri(pfeats_dis,pfeats_ori, nD, n_bin_dis, n_bin_ori,
				 MAX_range_dis, MAX_range_ori, be_ori_pi, pint_dis_ori);
	//////////////////////////////////////////////////////////////
	if (BETIME_DETAIL){
	time_quantize = time_quantize+((double)clock() - start);//count time
	start = clock();
	}
	//////////////////////////////////////////////////////////////

	//step2: lookup table
	matchFromOrgTablesFromInt(ptables,n_tr,pinds_pt_bundle,n_pt,n_table,
			pint_dis_ori,D, n_te, n_bin_dis, n_bin_ori, out_mat_matches);//pout_matches);


	//////////////////////////////////////////////////////////////
	if (BETIME_DETAIL){
	time_lookup = time_lookup+((double)clock() - start);//count time
	time_quantize = time_quantize / CLOCKS_PER_SEC;
	time_lookup   = time_lookup / CLOCKS_PER_SEC;
	printf("    run quantization %.4f s.\n",time_quantize);
	printf("    run lookup       %.4f s.\n",time_lookup);
	}
	 //////////////////////////////////////////////////////////////*/
	delete []pint_dis_ori;
	return;
}

/*

*/
int multiHashToID(float *pfeats_dis, float *pfeats_ori,int D, int n,int n_bin_dis, int n_bin_ori,
		double MAX_range_dis, double MAX_range_ori, bool be_ori_pi,
		int *pinds_pt_bundle, int n_pt, int n_table, int K, 
                std::vector<unsigned int> * out_vec_hs, std::vector<unsigned char> * out_vec_ids, 
	        std::vector<unsigned int> * out_vec_nexts, 
		int *poutid_start_table, unsigned int *poutind_tr2id) 
{
	std::vector<unsigned int>::iterator it_uint32;
	std::vector<unsigned char>::iterator it_uint8;
	//quantize all the features
	int i,nD,nn_pt,n_all=0,n_id=0;
	nD=n*D;
	unsigned char *pint_dis = new unsigned char[nD];
	unsigned char *pint_ori = new unsigned char[nD]; 
	quantizeDis(pfeats_dis,nD,MAX_range_dis,n_bin_dis,pint_dis);
	quantizeOri(pfeats_ori,nD,MAX_range_ori,n_bin_ori,be_ori_pi,pint_ori);
	unsigned char *pint_dis_ori = new unsigned char[nD]; 
	for (i=0;i<nD;i++)
		pint_dis_ori[i] = pint_dis[i]*n_bin_ori+pint_ori[i]+1;  // start from 1.
//printf(" pint_dis[0]=%d, pint_ori[0]=%d, pint_dis_ori[0]=%d\n",pint_dis[0],pint_ori[0],pint_dis_ori[0]);
	delete []pint_dis;
	delete []pint_ori;

	nn_pt = n*n_pt;  
	unsigned char * x=new unsigned char[nn_pt];
	unsigned int * ph_tmp, *pnext_tmp, *poutind_tr2id_tmp = poutind_tr2id;
	unsigned char * pid_tmp; 
	for (i=0;i<n_table;i++)
	{
		ph_tmp = NULL;
 		pnext_tmp = NULL;
		pid_tmp = NULL;
		// generate x
		generateX(pint_dis_ori, D,  n, 
			  pinds_pt_bundle+i*n_pt, n_pt,x);
		// single Hash table build
 		n_id = singleHash(x,n,n_pt,K, &ph_tmp,&pid_tmp,&pnext_tmp);///////////////////
	
		// transfer data to vectors
		if (i==0)
		{
			(*out_vec_hs).assign(ph_tmp,ph_tmp+n_id); 
			(*out_vec_nexts).assign(pnext_tmp,pnext_tmp+n_id);
			(*out_vec_ids).assign(pid_tmp,pid_tmp+n_id*n_pt);
		}else
		{
			it_uint32 = (*out_vec_hs).end();
			(*out_vec_hs).insert(it_uint32,ph_tmp,ph_tmp+n_id); 
			it_uint32 = (*out_vec_nexts).end();
			(*out_vec_nexts).insert(it_uint32,pnext_tmp,pnext_tmp+n_id);
			it_uint8  = (*out_vec_ids).end();
			(*out_vec_ids).insert(it_uint8,pid_tmp,pid_tmp+n_id*n_pt);
		}
		singleHashFind(pid_tmp, pnext_tmp, K, x, n_pt, n, poutind_tr2id_tmp);
	
		// delete ph, *pnext;	pid
		free(ph_tmp);
		free(pid_tmp);
		free(pnext_tmp);
		poutind_tr2id_tmp = poutind_tr2id_tmp+n;
		poutid_start_table[i] = n_all;
 		n_all = n_all + n_id;
// 		printf("Table %d is built. Size: %d.\n",i,n_id);		
	}

	delete []pint_dis_ori;
	delete []x;
	return n_all;
}

// write into the hash tables (in an array)
void writeTable(int *pid_start_table, unsigned int *pind_tr2id, int n_table, int n_tr, std::vector<int> * pout_tables)
{
	int i,j,start,start0;
	unsigned int id;
	for (i=0;i<n_table;i++)
	{
		start0 = n_tr*i;//
		start  = pid_start_table[i];
		for (j=0;j<n_tr;j++)
		{
			id = pind_tr2id[start0+j]-1;         ////// matlab index //////
			pout_tables[start+id].push_back(j+1);////// matlab index //////
		}
	}
	return;
}


void matchFromHash(std::vector<int> *ptables,int n_tr, int n_pt, int n_table, int *pid_start_table,
	int *pinds_pt_bundle, unsigned char *pids, unsigned int *pnexts, int K,
	float *pfeats_dis, float *pfeats_ori,int D, int n_te,
	int n_bin_dis, int n_bin_ori,double MAX_range_dis, double MAX_range_ori, bool be_ori_pi,
	unsigned char *pout_matches)
{	//////////////////////////////////////////////////////////////
	clock_t start;
	double time_quantize = 0;
	double time_generateX = 0;
	double time_hashfind = 0;
	double time_lookup = 0;
	if (BETIME_DETAIL){
	start = clock();
	}
	//////////////////////////////////////////////////////////////
	// step1: quantize testing features
	int i,j,p,id_start,nD,nn_pt;
	nD=n_te*D;
	unsigned char *pint_dis = new unsigned char[nD];
	unsigned char *pint_ori = new unsigned char[nD]; 
	quantizeDis(pfeats_dis,nD,MAX_range_dis,n_bin_dis,pint_dis);
	quantizeOri(pfeats_ori,nD,MAX_range_ori,n_bin_ori,be_ori_pi,pint_ori);
	unsigned char *pint_dis_ori = new unsigned char[nD]; 
	for (i=0;i<nD;i++)
		pint_dis_ori[i] = pint_dis[i]*n_bin_ori+pint_ori[i]+1;  // start from 1.
	delete []pint_dis;
	delete []pint_ori;
	//////////////////////////////////////////////////////////////
	if (BETIME_DETAIL){
	time_quantize = time_quantize+((double)clock() - start);//count time
	}
	//////////////////////////////////////////////////////////////

    std::vector<int> cur_bin;
	memset(pout_matches,0,n_tr*n_te*sizeof(unsigned char));

	int *p_j_n_tr = new int[n_te];
	for (j=0;j<n_te;j++)
		p_j_n_tr[j] = j*n_tr;

	nn_pt = n_te*n_pt;  
	unsigned char * x=new unsigned char[nn_pt];
	unsigned int *pnext_tmp;
	unsigned char *pid_tmp; 
	unsigned int * pind_te2id = new unsigned int[n_te];
	int local_id,global_id;


	for (i=0;i<n_table;i++)
	{
//printf("Table %d:\n",i);
		if (BETIME_DETAIL){
		start = clock();
		}//////////////////////////////////////
		// generate x
		generateX(pint_dis_ori, D,  n_te, 
			  pinds_pt_bundle+i*n_pt, n_pt,x);
 		//////////////////////////////////////////////////////////////
		if (BETIME_DETAIL){
		time_generateX = time_generateX+((double)clock() - start);//count time
		start = clock();
		}
 		//////////////////////////////////////////////////////////////

		// single hash find
		pid_tmp = pids+pid_start_table[i]*n_pt;
 		pnext_tmp = pnexts+pid_start_table[i];
		singleHashFind(pid_tmp, pnext_tmp, K, x, n_pt, n_te, pind_te2id);
		
		//////////////////////////////////////////////////////////////
		if (BETIME_DETAIL){
		time_hashfind = time_hashfind+((double)clock() - start);//count time
		start = clock();
		}
 		//////////////////////////////////////////////////////////////*/

		id_start = pid_start_table[i];
		for (j=0;j<n_te;j++)
		{
			local_id = pind_te2id[j];
			if (local_id<=0)  //no hit
				continue;

			global_id = local_id+id_start-1;         ///// matlab index /////
			cur_bin = ptables[global_id];
			for (p=0;p<cur_bin.size();p++)
				pout_matches[p_j_n_tr[j]+cur_bin.at(p)-1]++; ///// matlab index /////

		}
 		//////////////////////////////////////////////////////////////
  		if (BETIME_DETAIL){time_lookup = time_lookup+((double)clock() - start);}//count time
 		//////////////////////////////////////////////////////////////
	}
 	//////////////////////////////////////////////////////////////
	if (BETIME_DETAIL){
	time_quantize = time_quantize / CLOCKS_PER_SEC;
	time_generateX= time_generateX/ CLOCKS_PER_SEC;
	time_hashfind = time_hashfind / CLOCKS_PER_SEC;
	time_lookup   = time_lookup / CLOCKS_PER_SEC;
	printf("  run quantization %.4f s.\n",time_quantize);
	printf("  run generateX    %.4f s.\n",time_generateX);
	printf("  run hashfind     %.4f s.\n",time_hashfind);
	printf("  run lookup       %.4f s.\n",time_lookup);
	}
 	//////////////////////////////////////////////////////////////
	delete []pind_te2id;
	delete []x;
	delete []p_j_n_tr;
	return;
}

// the table is 3-dim:
// n_bin_dis*n_bin_ori x K x D
void buildClusterTables(float *pfeats_dis, float *pfeats_ori,int *pwhich_k, int D, int n,int K, 
		int n_bin_dis, int n_bin_ori, double MAX_range_dis, double MAX_range_ori,
		bool be_ori_pi,	std::vector<int> *pout_table)
{
	int i,j,nD,n_bin_dis_ori,which_k,which_bin;//table_size;
	nD=n*D;
	n_bin_dis_ori = n_bin_dis*n_bin_ori;
	unsigned char *pint_dis = new unsigned char[nD];
	unsigned char *pint_ori = new unsigned char[nD]; 
	quantizeDis(pfeats_dis,nD,MAX_range_dis,n_bin_dis,pint_dis);
	quantizeOri(pfeats_ori,nD,MAX_range_ori,n_bin_ori,be_ori_pi,pint_ori);
	unsigned char *pint_dis_ori = new unsigned char[nD];
	for (i=0;i<nD;i++)
		pint_dis_ori[i] = pint_dis[i]*n_bin_ori+pint_ori[i];  // start from 0, different from Hash table build.
	delete []pint_dis;
	delete []pint_ori;

	// define two arrays, to avoid repeating it in the following codes
        int *p_j_layer = new int[D];
	p_j_layer[0] = 0;
	for (j=1;j<D;j++)
		p_j_layer[j] = p_j_layer[j-1] + n_bin_dis_ori*K;
	int *p_j_col = new int[K];
	p_j_col[0] = 0;
	for (j=1;j<K;j++)
		p_j_col[j] = p_j_col[j-1] + n_bin_dis_ori;

	unsigned char *pcur_int_dis_ori = pint_dis_ori;
	for (i=0;i<n;i++)
	{
		which_k = pwhich_k[i]-1;//matlab's index
/*if (which_k==1)
{
	printf("i=%d:\n",i);
	for (j=0;j<D;j++)
		printf(" %d",pcur_int_dis_ori[j]);
	printf("\n");
}*/

		for (j=0;j<D;j++)
		{
			which_bin = pcur_int_dis_ori[j] + p_j_col[which_k] + p_j_layer[j];
//if (which_k==104)
//printf("ptable[%d](%d,%d,%d).pushback(%d)\n", which_bin, pcur_int_dis_ori[j],which_k,j,i+1);
			pout_table[which_bin].push_back(i+1); //matlab's index begins with 1, not 0
		}
		pcur_int_dis_ori = pcur_int_dis_ori + D;
	}
	delete [] pint_dis_ori;
	delete[] p_j_layer;
	delete[] p_j_col;
	return;
	
}
void matchFromClusterTables(std::vector<int> *ptable,int n_tr,float *pfeats_dis, float *pfeats_ori,
	bool * pb_mat, int K, int D, int n_te,int n_bin_dis, int n_bin_ori,
	double MAX_range_dis, double MAX_range_ori, bool be_ori_pi,unsigned char *pout_matches)
{	//////////////////////////////////////////////////////////////

	clock_t start;
	double time_quantize = 0;
	double time_lookup = 0;
	int n_count_lookup = 0;
	if (BETIME_DETAIL){
	start = clock();
	}
	//////////////////////////////////////////////////////////////
	// step1: quantize testing features
	int i,j,p,k,nD,n_bin_dis_ori,i_n_tr, which_bin,nbin_size;
	nD=n_te*D;
	n_bin_dis_ori = n_bin_dis*n_bin_ori;
	unsigned char *pint_dis = new unsigned char[nD];
	unsigned char *pint_ori = new unsigned char[nD]; 
	quantizeDis(pfeats_dis,nD,MAX_range_dis,n_bin_dis,pint_dis);
	quantizeOri(pfeats_ori,nD,MAX_range_ori,n_bin_ori,be_ori_pi,pint_ori);
	unsigned char *pint_dis_ori = new unsigned char[nD]; 
	for (i=0;i<nD;i++)
		pint_dis_ori[i] = pint_dis[i]*n_bin_ori+pint_ori[i];  // start from 0., different from Hash table.
	delete []pint_dis;
	delete []pint_ori;
	//////////////////////////////////////////////////////////////
	if (BETIME_DETAIL){time_quantize = time_quantize+((double)clock() - start);}//count time
	//////////////////////////////////////////////////////////////

	// define two arrays, to avoid repeating it in the following codes
    int *p_j_layer = new int[D];
	p_j_layer[0] = 0;
	for (j=1;j<D;j++)
		p_j_layer[j] = p_j_layer[j-1] + n_bin_dis_ori*K;
	int *p_j_col = new int[K];
	p_j_col[0] = 0;
	for (j=1;j<K;j++)
		p_j_col[j] = p_j_col[j-1] + n_bin_dis_ori;
	
    std::vector<int> cur_bin;
	memset(pout_matches,0,n_tr*n_te*sizeof(unsigned char));

	//////////////////////////////////////////////////////////////
	if (BETIME_DETAIL){start = clock();}
	//////////////////////////////////////////////////////////////
	unsigned char *pcur_int_dis_ori = pint_dis_ori;
        bool be_match;
	for (i=0;i<n_te;i++)
	{
		i_n_tr = i*n_tr;
		for (k=0;k<K;k++)
		{
			be_match = pb_mat[i*K+k];
			if (!be_match)
				continue;
			for (j=0;j<D;j++)
			{
				which_bin = pcur_int_dis_ori[j] + p_j_col[k] + p_j_layer[j];
				cur_bin = ptable[which_bin];
				nbin_size = cur_bin.size();
//printf("i=%d,(tt=%d,k=%d,j=%d): which_bin=%d\n",i,pcur_int_dis_ori[j],k,j,which_bin);		
				for (p=0;p<nbin_size;p++) //matlab's index begins with 1, not 0
				{
					pout_matches[i_n_tr+cur_bin.at(p)-1]++;	 ///// matlab index /////
//printf("pout_matches[%d]=%d\n",i_n_tr+cur_bin.at(p),pout_matches[i_n_tr+cur_bin.at(p)-1]);		
				}
	 			//////////////////////////////////////////////////////
				if (BETIME_DETAIL){n_count_lookup = n_count_lookup+nbin_size;}
	 			//////////////////////////////////////////////////////
			}
		}
		pcur_int_dis_ori = pcur_int_dis_ori + D;
		
        }	
	//////////////////////////////////////////////////////////////
	if (BETIME_DETAIL){time_lookup = time_lookup+((double)clock() - start);//count time
	time_quantize = time_quantize / CLOCKS_PER_SEC;
	time_lookup   = time_lookup / CLOCKS_PER_SEC;
	printf("  run quantization %d times, spend %.4f s.\n",n_te,time_quantize);
	printf("        run lookup %d times, spend %.4f s.\n",n_count_lookup, time_lookup);
	printf("        run lookup %d times.\n",n_count_lookup);
	}
 	//////////////////////////////////////////////////////////////
	delete [] pint_dis_ori;
	delete[] p_j_layer;
	delete[] p_j_col;
	return;
	
}


// method: TABLE_PLAIN
// the 4*8*36 table match, conterparter:buildPlainTables()
// mex file: mexMatchFromTables()
//
void matchFromPlainTablesFromInt(std::vector<int> *ptable,int n_tr,unsigned char *pint_dis_ori,
		int D, int n_te,int n_bin_dis, int n_bin_ori, Mat &out_mat_matches)
{
	int i,j,p,table_size,nbin_size,i_n_tr;

	table_size = n_bin_dis*n_bin_ori;

	// define the array, to avoid repeating it in the following codes
	int *p_j_table_size = new int[D];
 	p_j_table_size[0] = 0;
	for (j=1;j<D;j++)
		p_j_table_size[j] = p_j_table_size[j-1] + table_size;

    	std::vector<int> cur_bin;

    	unsigned char *pcur_int_dis_ori = pint_dis_ori;
    	out_mat_matches = Mat::zeros(n_te,n_tr,CV_8UC1);

    	//////////////////////////
	//unsigned char * p_thisrow;                       	      //for higher opencv version
    	//////////////////////////
	for (i=0;i<n_te;i++)
	{
		//p_thisrow = out_mat_matches.ptr<unsigned char>(i,0);//for higher opencv version
		for (j=0;j<D;j++)
		{
			cur_bin = ptable[pcur_int_dis_ori[j]+p_j_table_size[j]];
			nbin_size = cur_bin.size();
			for (p=0;p<nbin_size;p++)
				//(p_thisrow[cur_bin.at(p)])++;          //for higher opencv version
				out_mat_matches.at<unsigned char>(i,cur_bin.at(p))++;///// no matlab index /////for lower opencv version

		}
		pcur_int_dis_ori = pcur_int_dis_ori + D; //to the next patch
	}

	delete[] p_j_table_size;
	return;

}
// method: TABLE_PLAIN
// the 4*8*36 table match, conterparter:buildPlainTables()
// mex file: mexMatchFromTables()
//
void matchFromPlainTables(std::vector<int> *ptable,int n_tr,float *pfeats_dis, float *pfeats_ori,
		int D, int n_te,int n_bin_dis, int n_bin_ori,double MAX_range_dis, double MAX_range_ori,
		bool be_ori_pi,Mat &out_mat_matches)
{//////////////////////////////////////////////////////////////
	clock_t start;
	double time_quantize = 0;
	double time_lookup = 0;
	if (BETIME_DETAIL){
	start = clock();
	}
	//////////////////////////////////////////////////////////////

	// step1: quantize testing features
	int nD = n_te*D;
	unsigned char *pint_dis_ori = new unsigned char[nD];
	quantizeDisOri(pfeats_dis,pfeats_ori, nD, n_bin_dis, n_bin_ori,
				 MAX_range_dis, MAX_range_ori, be_ori_pi, pint_dis_ori);
	//////////////////////////////////////////////////////////////
	if (BETIME_DETAIL){
	time_quantize = time_quantize+((double)clock() - start);//count time
	start = clock();
	}
	//////////////////////////////////////////////////////////////

	//step2: lookup table
	matchFromPlainTablesFromInt(ptable,n_tr,
			pint_dis_ori,D, n_te, n_bin_dis, n_bin_ori, out_mat_matches);

	//////////////////////////////////////////////////////////////
	if (BETIME_DETAIL){
	time_lookup = time_lookup+((double)clock() - start);//count time
	time_quantize = time_quantize / CLOCKS_PER_SEC;
	time_lookup   = time_lookup / CLOCKS_PER_SEC;
	printf("    run quantization %.4f s.\n",time_quantize);
	printf("    run lookup       %.4f s.\n",time_lookup);
	}
	 //////////////////////////////////////////////////////////////*/
	delete []pint_dis_ori;
	return;
}


// method: TABLE_PLAIN
// the 4*8*36 table match. conterparter: matchFromPlainTables()
// mex file: mexBuildTrTables()
//
void buildPlainTablesFromInt(unsigned char *pint_dis_ori,int D, int n,int n_bin_dis, int n_bin_ori,
		std::vector<int> *pout_table)
{
	int i,j,table_size;
	table_size = n_bin_dis*n_bin_ori;

	// define the arrays, to avoid repeating it in the following codes
	int *p_j_table_size = new int[D];
 	p_j_table_size[0] = 0;
	for (j=1;j<D;j++)
		p_j_table_size[j] = p_j_table_size[j-1] + table_size;

	unsigned char *pcur_int_dis_ori = pint_dis_ori;
	for (i=0;i<n;i++)
	{
		for (j=0;j<D;j++)
		{
//printf("  %d(%d-%d-%d) cell ",pcur_int_dis[j]+pcur_int_ori[j]*n_bin_dis+j*n_bin_dis*n_bin_ori,pcur_int_dis[j],pcur_int_ori[j],j);
//printf("  %d\n",pcur_int_dis[j]+p_j_n_bin_dis[pcur_int_ori[j]]+p_j_table_size[j]);
			pout_table[pcur_int_dis_ori[j]+p_j_table_size[j]].push_back(i); //no matlab's index begins with 1, not 0
		}
		pcur_int_dis_ori = pcur_int_dis_ori+D; // next patch
	}
	delete[] p_j_table_size;
	return;
}

// method: TABLE_PLAIN
// the 4*8*36 table match. conterparter: matchFromPlainTables()
// mex file: mexBuildTrTables()
//
void buildPlainTables(float *pfeats_dis, float *pfeats_ori,int D, int n,int n_bin_dis, int n_bin_ori,
		double MAX_range_dis, double MAX_range_ori, bool be_ori_pi,
		std::vector<int> *pout_table)
{
	int nD = n*D;
	unsigned char *pint_dis_ori = new unsigned char[nD];
	quantizeDisOri(pfeats_dis,pfeats_ori, nD, n_bin_dis, n_bin_ori,
			 MAX_range_dis, MAX_range_ori, be_ori_pi, pint_dis_ori);

	buildPlainTablesFromInt(pint_dis_ori, D, n, n_bin_dis,  n_bin_ori, pout_table);
	return;

}

#endif // caihash_hpp
