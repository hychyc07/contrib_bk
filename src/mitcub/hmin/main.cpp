#include <iostream>
#include <fstream>
#include <time.h>
#include <cv.h>
#include <highgui.h>
#include "hmaxModel.h"
//#include "gvec.h"
#include "dictionary.h"

using namespace std;
//using namespace gurls;

int main(int argc, char **argv)
{

	string input_folder("../data");
	if (argc < 2){
		cout << "========================================================================================"<< endl
			 << " WARNING: No input folder provided. " << endl
			 << " Using the default option: \'" << input_folder <<  "\'" << endl
			 <<	" Please make sure such folder exists or provide a valid path using the following syntax:" << endl
			 << " " << argv[0]
			 << " <path-to-a-valid-input-folder>" << endl
			 << "========================================================================================" << endl << endl;
	}else {
		input_folder = argv[1];
	}

	IplImage *img;
	img=cvLoadImage((input_folder+"/image.jpg").c_str(),-1);
	dictionary *dict;

	dict = (dictionary *) malloc(sizeof(dictionary));
	dict->pfCount = 12;
	dict->yxCountMax = 16;
	dict->fCount = 64;
	dict->fSizes = (double*)  malloc(dict->fCount * sizeof(double));
	dict->fVals  = (float*) malloc (dict->pfCount * dict->yxCountMax * dict->yxCountMax * dict->fCount * sizeof(float));


	ifstream ifile((input_folder+"/fSizes.txt").c_str());
	for(int s = 0; s < dict->fCount; s++) ifile >> dict->fSizes[s];
	ifile.close();

	ifile.open((input_folder+"/fVals.txt").c_str());
	for(int s = 0; s < (dict->pfCount * dict->yxCountMax * dict->yxCountMax * dict->fCount); s++) ifile >> dict->fVals[s];
	ifile.close();

	hmaxModel *model;
	model = new hmaxModel(img->depth, dict);

//	gVec<float> out(model->FCount());
//	model->response(img, out.getData());
//	cout << "C2 = " << out << endl;

	float out[model->FCount()];
	model->response(img, out);
	cout << "HMAX Response:" << endl;
	for(int s = 0; s < model->FCount(); s++) cout << out[s] << endl;
	delete model;

	return 0;
}
