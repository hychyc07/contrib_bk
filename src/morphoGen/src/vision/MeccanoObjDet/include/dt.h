/*  
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

/* distance transform */

#ifndef DT_H
#define DT_H
#undef __GXX_EXPERIMENTAL_CXX0X__

#include <algorithm>
#include "image.h"
#include "math.h"
#include <queue>

#define INF 1E20

static inline int square(int a) {return a*a;}

/////////////////////////////////////////////
// added by caicai
/////////////////////////////////////////////
/* dt of 1d function using squared distance 
return not only distance, but the location of 
nearest point.*/
static float *dt1D(float *f, int n, int *ploc, int * square_n) {
  float *d = new float[n];
  int *v = new int[n];
  float *z = new float[n+1];
  int k = 0;
  v[0] = 0;
  z[0] = -INF;
  z[1] = +INF;
  for (int q = 1; q <= n-1; q++) {
    float s  = ((f[q]+square_n[q])-(f[v[k]]+square_n[v[k]]))/((q-v[k])<<1);
    while (s <= z[k]) {
      k--;
      s  = ((f[q]+square_n[q])-(f[v[k]]+square_n[v[k]]))/((q-v[k])<<1);
    }
    k++;
    v[k] = q;
    z[k] = s;
    z[k+1] = +INF;
  }

  k = 0;
  for (int q = 0; q <= n-1; q++) {
    while (z[k+1] < q)
      k++;
    if (q>v[k])
      d[q] = square_n[q-v[k]] + f[v[k]];
    else
      d[q] = square_n[v[k]-q] + f[v[k]];
    ploc[q] = v[k]; // added by caicai
  }

  delete [] v;
  delete [] z;
  return d;
}
/////////////////////////////////////////////
// added by caicai
/////////////////////////////////////////////
/* dt of 2d function using squared distance 
return not only distance, but the location of 
nearest point.*/
static void dt2D(image<float> *im, image<int> *pNearest) {
  int width = im->width();
  int height = im->height();
  int x,y;
  int max_w_h = std::max(width,height);
  float *f = new float[max_w_h];
  float *d=NULL;
  image<int> *pNear1D = new image<int>(width, height, false);

  // save the square(n) in an array, to avoid compute it again and again 
  int * square_n = new int[max_w_h];
  for (x=0;x<max_w_h;x++)
	square_n[x] = square(x);

  int *ploc=new int[max_w_h];//added by caicai
  // transform along columns
  for (x = 0; x < width; x++) {
    for (y = 0; y < height; y++) {
      f[y] = imRef(im, x, y);
    }
    if (d!=NULL)
    	delete []d;                 //added by caicai
    d = dt1D(f, height,ploc,square_n);
    for (int y = 0; y < height; y++) {
      imRef(im, x, y) = d[y];
      imRef(pNear1D,x,y) = ploc[y]; //added by caicai
    }    
  }

  // transform along rows
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      f[x] = imRef(im, x, y);
    }
    if (d!=NULL)
    	delete []d;                 //added by caicai
    d = dt1D(f, width,ploc,square_n);
    for (int x = 0; x < width; x++) {
      imRef(im, x, y) = d[x];
      imRef(pNearest,x,y) = height*ploc[x]+imRef(pNear1D,ploc[x],y);//added by caicai
    }
  }
  delete [] ploc;//added by caicai
  delete [] f;
  delete [] d;
  delete pNear1D;//added by caicai
  delete [] square_n; // added by caicai
}
/////////////////////////////////////////////
// added by caicai
// input Mat.
/////////////////////////////////////////////
/* dt of binary image using squared distance
return not only distance, but the location of
nearest point.*/
/* dt of binary image using squared distance */
static image<float> *dt(Mat mat_edge, image<int> *pNearest, unsigned char on = 1) {
  int width = mat_edge.cols;
  int height = mat_edge.rows;

  image<float> *out = new image<float>(width, height, true);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (mat_edge.at<unsigned char>(y,x) != on)
	imRef(out, x, y) = INF;
    }
  }

  dt2D(out,pNearest); //modified by caicai
  return out;
}
/////////////////////////////////////////////
// added by caicai
// new
// with Mat output
/////////////////////////////////////////////
void odt(Mat mat_edge,Mat mat_px,Mat mat_py,bool be_ori_pi,
		Mat *out_mat_dis_map, Mat *out_mat_ori_map, Mat *out_mat_ind_map)
{
	  int w, h;
	  w = mat_edge.cols;
	  h = mat_edge.rows;
	  int x,y,pos,ind_nn;
	  float angle;

	  //distance transform
	  image<float> *pout;
	  image<int> *pNearest = new image<int>(w, h, false);
	  pout = dt(mat_edge,pNearest);
	 // delete pinput;

	  (*out_mat_dis_map).create(h,w,CV_32F);
	  (*out_mat_ori_map).create(h,w,CV_32F);
	  (*out_mat_ind_map).create(h,w,CV_32S);
	  //output dis, ind
	  float *pangle=new float[h*w];

	  for (x = 0,pos=0; x < w; x++)
	  {
		for (y = 0; y < h; y++,pos++)
	    {
			  (*out_mat_dis_map).at<float>(y,x) = sqrt(imRef(pout, x, y));    //square root
			  (*out_mat_ind_map).at<int>(y,x) = imRef(pNearest, x, y);    //no matlab index
			  //angle = atan2(float(py[pos]),float(px[pos]));
			  angle = atan2(mat_py.at<float>(y,x), mat_px.at<float>(y,x));
			  pangle[pos] = angle;
	      }
	  }

	  //output ori
	  for (y=0;y<h;y++)
	  {
		  for (x=0;x<w;x++)
		  {
			  ind_nn = (*out_mat_ind_map).at<int>(y,x);
			  angle = pangle[ind_nn];
			  if (be_ori_pi && (angle<0))
				  (*out_mat_ori_map).at<float>(y,x) = angle+(float)CV_PI; //all angles are in [0,pi]
			  else
				  (*out_mat_ori_map).at<float>(y,x) = angle; //all angles are in [-pi,pi]
		  }
	  }

	  delete [] pangle;
	  delete pNearest;
	  delete pout;
	  return;
}

#endif
