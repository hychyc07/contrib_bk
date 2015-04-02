/**
 * \file centsur.h
 * \brief An implementation modelling the centre-surround response, used for construction of spatial uniqueness maps.
 * Based on the difference-of-Gaussians pyramid approach of Itti. A single Gaussian pyramid is created. Neighbouring pyramid entries are subtracted (eg Pyramid level 0 - Pyramid level 1, 1-2, 2-3 ...etc), and so are 2nd neighbours (eg 1-3,2-4,3-5..etc), to obtain spatial uniqueness at various spatial scales. All resultant subtraction results (the difference of Gaussian maps) are summated to yield the centre-surround map output.
 *
 * \author Andrew Dankers
 * \date 2009
 * \note Release under GNU GPL v2.0
 **/

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Andrew Dankers, maintainer Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
 * website: www.robotcub.org 
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

#ifndef __CENTSUR_H__
#define __CENTSUR_H__

#include <ipp.h>
	
class CentSur { 
public:
    /**
    * constructor
    */
	CentSur(IppiSize imsize, int nscale, double sigma = 1.0);
    /**
     * destructor
     */
	~CentSur();

    /**
     * convert image to 32f precision
     */
	void proc_im_8u(Ipp8u* im_8u, int psb_8u);

    /**
     * process 32f image creating gauss pyramids:
     */
	void proc_im_32f(Ipp32f* im_32f, int psb_32f);
    
    /**
     * returns gaussians
     */
	Ipp32f* get_gauss(int s){return gauss[s];}

    /**
     * returns pyramids
     */
	Ipp32f* get_pyramid(int s){return pyramid[s];}

    /**
     * get center surround image in 32f precision
     */
	Ipp32f* get_centsur_32f(){return cs_tot_32f;} 

    /**
     * get center surround image in 8u precision
     */
	Ipp8u*  get_centsur_norm8u(){return cs_tot_8u;}
    
    /**
     * get center surround row size in 32f precision
     */
	int get_psb_32f(){return psb_32f;}

    /**
     * get center surround row size in 8uf precision
     */
	int get_psb_8u(){return psb_8u;}

private:

    /**
     * creates pyramids
     */
	void make_pyramid(Ipp32f* im_in, int pin32_);

    
	Ipp32f **pyramid,**pyramid_gauss,**gauss,*cs_tot_32f,*tmp_im_32f,*im_in_32f;

	Ipp8u *cs_tot_8u,*pbuf;

	int *psb_p,pbufsize,psb_8u,psb_32f,ngauss;

	IppiSize srcsize,*psize;

	IppiRect *proi, *dstRect;

	double sd,su,sigma;
    Ipp8u* pBuffer;
    Ipp8u* pBufferGauss;
};
#endif
//empty line to make gcc happy
