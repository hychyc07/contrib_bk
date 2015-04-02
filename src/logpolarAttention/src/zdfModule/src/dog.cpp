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
#include "iCub/dog.h"
#include <stdio.h>

//set up DoG Kernel stuff:
const int kern_sz = 7;
const int kern_anc = 3;

const Ipp32f kern1[] ={18.0,33.0,49.0,55.0,49.0,33.0,18.0};
const Ipp32f kern2[] ={ 5.0,23.0,59.0,82.0,59.0,23.0,5.0 };

DoG::DoG(IppiSize srcsize_)
{   
    width = 0, height = 0, psb_o = 0, psb_pad = 0, psb_pad_8u = 0;
    srcsize = srcsize_;
    width=srcsize.width;
    height=srcsize.height;

    in_pad_8u = ippiMalloc_8u_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad_8u);

    in_pad    = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    tmp1      = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    tmp2      = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    tmp3      = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    dog       = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    dog_on    = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    dog_off   = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    dog_onoff = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);

    out_dog_on    = ippiMalloc_8u_C1(width,height,&psb_o);
    out_dog_off   = ippiMalloc_8u_C1(width,height,&psb_o);
    out_dog_onoff = ippiMalloc_8u_C1(width,height,&psb_o);

    psize.width  = width+PAD_BORD*2;
    psize.height = height+PAD_BORD*2;
}


DoG::~DoG()
{
    ippFree(in_pad_8u);
    ippFree(in_pad);
    ippFree(tmp1);
    ippFree(tmp2);
    ippFree(tmp3);
    ippFree(dog);
    ippFree(dog_on);
    ippFree(dog_off);
    ippFree(dog_onoff);
    ippFree(out_dog_on);
    ippFree(out_dog_off);
    ippFree(out_dog_onoff);    
}

void DoG::conv_32f_to_8u( Ipp32f*im_i, int p4_, Ipp8u*im_o, int p1_, IppiSize srcsize_) {

    Ipp32f min = 0.0;
    Ipp32f max = 0.0;
    ippiMinMax_32f_C1R( im_i, p4_,srcsize_, &min, &max);
    //if (max == min){max=255.0; min=0.0;}
    ippiScale_32f8u_C1R(im_i, p4_, im_o, p1_, srcsize_, min, max );
} 


void DoG::proc(Ipp8u*in_, int psb_in_)
{
    //pad:
    ippiCopyReplicateBorder_8u_C1R(in_,psb_in_,srcsize,in_pad_8u,psb_pad_8u,psize,PAD_BORD,PAD_BORD);

    //convert to 32f: 
    ippiConvert_8u32f_C1R(in_pad_8u,psb_pad_8u,in_pad,psb_pad,psize);

    //DOG filtering:
    ippiFilterColumn_32f_C1R(&in_pad[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern1,kern_sz,kern_anc);
    ippiFilterRow_32f_C1R(&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp2[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern1,kern_sz,kern_anc);
    ippiFilterColumn_32f_C1R(&in_pad[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern2,kern_sz,kern_anc);
    ippiFilterRow_32f_C1R(&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp3[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern2,kern_sz,kern_anc);

    ippiSub_32f_C1R(tmp2,psb_pad,tmp3,psb_pad,dog,psb_pad,psize);

    //on-centre:
    //keep only results above zero:
    ippiThreshold_LT_32f_C1R(dog,psb_pad,dog_on,psb_pad,psize,0.0);
    //off-centre:  
    //negate: 
    ippiMulC_32f_C1IR(-1.0,dog,psb_pad,psize);
    //and keep only results above zero:
    ippiThreshold_LT_32f_C1R(dog,psb_pad,dog_off,psb_pad,psize,0.0);
    //on+off:
    ippiAdd_32f_C1R(dog_on,psb_pad,dog_off,psb_pad,dog_onoff,psb_pad,psize);

    //convert to 8u and remove pad:
    conv_32f_to_8u(&dog_on[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,out_dog_on,psb_o,srcsize);
    conv_32f_to_8u(&dog_off[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,out_dog_off,psb_o,srcsize);
    conv_32f_to_8u(&dog_onoff[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,out_dog_onoff,psb_o,srcsize);

}
