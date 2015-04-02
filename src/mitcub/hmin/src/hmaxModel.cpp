#include "hmaxModel.h"

hmaxModel::hmaxModel(int _depth, dictionary *_dict)
{
		depth = _depth;
		             
		nsi = 12;
		ns1 = 12;
		nc1 = 11;
		ns2 = 11;
		nc2 = 2;

		si = new Layer *[nsi];
		s1 = new Layer *[ns1];
		c1 = new Layer *[nc1];
		s2 = new Layer *[ns2];
		c2 = new Layer *[nc2];

		fs1 = new GaborFilter(11, 0.3f, 5.6410f, 4.5128f, 12);
		fc1 = new MaxFilter(2, 10);
		fs2 = new NDPFilter(true, 0.00001f, true, _dict);
		fc2 = new GMaxFilter(6);

		s1SStep = 1;
		c1SStep = 1;
		s2SStep = 1;
		c2SStep = 5;

		sub_buffer = (float**) malloc(nsi * sizeof(float*));

		sub_buffer[0]  = (float *) malloc(256*256*1*sizeof(float));	
		sub_buffer[1]  = (float *) malloc(214*214*1*sizeof(float));	
		sub_buffer[2]  = (float *) malloc(180*180*1*sizeof(float));	
		sub_buffer[3]  = (float *) malloc(152*152*1*sizeof(float));	
		sub_buffer[4]  = (float *) malloc(128*128*1*sizeof(float));	
		sub_buffer[5]  = (float *) malloc(106*106*1*sizeof(float));	
		sub_buffer[6]  = (float *) malloc(90*90  *1*sizeof(float));	
		sub_buffer[7]  = (float *) malloc(76*76  *1*sizeof(float));	
		sub_buffer[8]  = (float *) malloc(64*64  *1*sizeof(float));	
		sub_buffer[9]  = (float *) malloc(52*52  *1*sizeof(float));	
		sub_buffer[10] = (float *) malloc(44*44  *1*sizeof(float));	
		sub_buffer[11] = (float *) malloc(38*38  *1*sizeof(float));	

		subsample = (IplImage**) malloc(nsi * sizeof(IplImage*));

		subsample[0]  = cvCreateImage(cvSize(256,256),	depth,1);
		subsample[1]  = cvCreateImage(cvSize(214,214),	depth,1);
		subsample[2]  = cvCreateImage(cvSize(180,180),	depth,1);
		subsample[3]  = cvCreateImage(cvSize(152,152),	depth,1);
		subsample[4]  = cvCreateImage(cvSize(128,128),	depth,1);
		subsample[5]  = cvCreateImage(cvSize(106,106),	depth,1);
		subsample[6]  = cvCreateImage(cvSize(90,90),	depth,1);
		subsample[7]  = cvCreateImage(cvSize(76,76),	depth,1);
		subsample[8]  = cvCreateImage(cvSize(64,64),	depth,1);
		subsample[9]  = cvCreateImage(cvSize(52,52),	depth,1);
		subsample[10] = cvCreateImage(cvSize(44,44),	depth,1);
		subsample[11] = cvCreateImage(cvSize(38,38),	depth,1);

		si[ 0] = new Layer(256, 1           , -127.5000000000000000f,   1.0000000000000000f);
	    si[ 1] = new Layer(214, 1           , -126.6505577477897901f,   1.1892071150027210f);
	    si[ 2] = new Layer(180, 1           , -126.5721138323919917f,   1.4142135623730949f);
	    si[ 3] = new Layer(152, 1           , -126.9753587033108744f,   1.6817928305074288f);
	    si[ 4] = new Layer(128, 1           , -126.9999999999999858f,   1.9999999999999998f);
	    si[ 5] = new Layer(106, 1           , -124.8667470752856872f,   2.3784142300054416f);
	    si[ 6] = new Layer( 90, 1           , -125.8650070512054242f,   2.8284271247461894f);
	    si[ 7] = new Layer( 76, 1           , -126.1344622880571649f,   3.3635856610148576f);
	    si[ 8] = new Layer( 64, 1           , -125.9999999999999716f,   3.9999999999999991f);
	    si[ 9] = new Layer( 52, 1           , -121.2991257302775239f,   4.7568284600108832f);
	    si[10] = new Layer( 44, 1           , -121.6223663640861190f,   5.6568542494923779f);
	    si[11] = new Layer( 38, 1           , -124.4526694575497032f,   6.7271713220297134f);
		
		s1[ 0] = new Layer(246, fs1->FCount(), -122.5000000000000000f,   1.0000000000000000f);
	    s1[ 1] = new Layer(204, fs1->FCount(), -120.7045221727761799f,   1.1892071150027210f);
	    s1[ 2] = new Layer(170, fs1->FCount(), -119.5010460205265161f,   1.4142135623730949f);
	    s1[ 3] = new Layer(142, fs1->FCount(), -118.5663945507737225f,   1.6817928305074288f);
	    s1[ 4] = new Layer(118, fs1->FCount(), -116.9999999999999858f,   1.9999999999999998f);
	    s1[ 5] = new Layer( 96, fs1->FCount(), -112.9746759252584809f,   2.3784142300054416f);
	    s1[ 6] = new Layer( 80, fs1->FCount(), -111.7228714274744874f,   2.8284271247461894f);
	    s1[ 7] = new Layer( 66, fs1->FCount(), -109.3165339829828753f,   3.3635856610148576f);
	    s1[ 8] = new Layer( 54, fs1->FCount(), -105.9999999999999716f,   3.9999999999999991f);
	    s1[ 9] = new Layer( 42, fs1->FCount(),  -97.5149834302231113f,   4.7568284600108832f);
	    s1[10] = new Layer( 34, fs1->FCount(),  -93.3380951166242312f,   5.6568542494923779f);
	    s1[11] = new Layer( 28, fs1->FCount(),  -90.8168128474011240f,   6.7271713220297134f);
	
	    c1[ 0] = new Layer( 47, fs1->FCount(), -115.0000000000000000f,   5.0000000000000000f);
	    c1[ 1] = new Layer( 39, fs1->FCount(), -112.9746759252584951f,   5.9460355750136049f);
	    c1[ 2] = new Layer( 33, fs1->FCount(), -113.1370849898475939f,   7.0710678118654746f);
	    c1[ 3] = new Layer( 27, fs1->FCount(), -109.3165339829828895f,   8.4089641525371448f);
	    c1[ 4] = new Layer( 21, fs1->FCount(),  -99.9999999999999858f,   9.9999999999999982f);
	    c1[ 5] = new Layer( 17, fs1->FCount(),  -95.1365692002176644f,  11.8920711500272080f);
	    c1[ 6] = new Layer( 15, fs1->FCount(),  -98.9949493661166287f,  14.1421356237309475f);
	    c1[ 7] = new Layer( 11, fs1->FCount(),  -84.0896415253714480f,  16.8179283050742896f);
	    c1[ 8] = new Layer(  9, fs1->FCount(),  -79.9999999999999858f,  19.9999999999999964f);
	    c1[ 9] = new Layer(  7, fs1->FCount(),  -71.3524269001632518f,  23.7841423000544161f);
	    c1[10] = new Layer(  5, fs1->FCount(),  -56.5685424949237756f,  28.2842712474618878f);

	    s2[ 0] = new Layer( 44, fs2->FCount(), -107.5000000000000000f,   5.0000000000000000f);
	    s2[ 1] = new Layer( 36, fs2->FCount(), -104.0556225627380798f,   5.9460355750136049f);
	    s2[ 2] = new Layer( 30, fs2->FCount(), -102.5304832720493806f,   7.0710678118654746f);
	    s2[ 3] = new Layer( 24, fs2->FCount(),  -96.7030877541771616f,   8.4089641525371448f);
	    s2[ 4] = new Layer( 18, fs2->FCount(),  -84.9999999999999858f,   9.9999999999999982f);
	    s2[ 5] = new Layer( 14, fs2->FCount(),  -77.2984624751768479f,  11.8920711500272080f);
	    s2[ 6] = new Layer( 12, fs2->FCount(),  -77.7817459305202163f,  14.1421356237309475f);
	    s2[ 7] = new Layer(  8, fs2->FCount(),  -58.8627490677600136f,  16.8179283050742896f);
	    s2[ 8] = new Layer(  6, fs2->FCount(),  -49.9999999999999929f,  19.9999999999999964f);
	    s2[ 9] = new Layer(  4, fs2->FCount(),  -35.6762134500816259f,  23.7841423000544161f);
	    s2[10] = new Layer(  2, fs2->FCount(),  -14.1421356237309439f,  28.2842712474618878f);

	    c2[ 0] = new Layer(  1, fs2->FCount(),    0.0000000000000000f,   1.0000000000000000f);
	    c2[ 1] = new Layer(  1, fs2->FCount(),    0.0000000000000000f,   1.0000000000000000f);

}


void hmaxModel::response(IplImage *img, float *out)
{
		IplImage	*tmp;
		uchar 		*tmpdata;
		float *dst;
		uchar *src;

		tmp = cvCreateImage(cvSize(img->height,img->width), img->depth, 1);
		tmpdata = (uchar*) tmp->imageData;

		for(int i = 0; i < img->height; i++)
				for(int j = 0; j < img->width; j++)
						tmpdata[i*img->width + j] = (uchar) img->imageData[i*img->width + j];

		for(int s = 0; s < nsi; s++)
				cvResize(tmp,subsample[s]);

		// Now we have the images, stick them in the buffers
		for(int s = 0; s < nsi; s++)
		{
			dst = sub_buffer[s];
			src = (uchar *) subsample[s]->imageData;
			for(int i = 0; i < subsample[s]->height; i++)
					for(int j = 0; j< subsample[s]->width; j++)
							dst[i*subsample[s]->width + j] = (float) src[i*subsample[s]->width + j];
		}

		for(int s = 0; s < nsi; s++) si[s]->SetLayer(sub_buffer[s]);

		for(int s = 0; s < ns1; s++) fs1->ComputeLayer(si + s * s1SStep, s1[s]);
		for(int s = 0; s < nc1; s++) fc1->ComputeLayer(s1 + s * c1SStep, c1[s]);
		for(int s = 0; s < ns2; s++) fs2->ComputeLayer(c1 + s * s2SStep, s2[s]);
		for(int s = 0; s < nc2; s++) fc2->ComputeLayer(s2 + s * c2SStep, c2[s]);

		for(int s = 0; s < nc2; s++) memcpy((out + s*fs2->FCount()), c2[s]->GetLayer(), fs2->FCount()*sizeof(float));
}

hmaxModel::~hmaxModel()
{
		delete fs1;
		delete fc1;
		delete fs2;
		delete fc2;

		for(int s = 0; s < nsi; s++) delete si[s];
		for(int s = 0; s < ns1; s++) delete s1[s];
		for(int s = 0; s < nc1; s++) delete c1[s];
		for(int s = 0; s < ns2; s++) delete s2[s];
		for(int s = 0; s < nc2; s++) delete c2[s];

		delete si;
		delete s1;
		delete c1;
		delete s2;
		delete c2;

		for(int s = 0; s < nsi; s++) cvReleaseImage(&subsample[s]);
		for(int s = 0; s < nsi; s++) delete sub_buffer[s];
}
