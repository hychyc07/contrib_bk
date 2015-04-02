/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Vadim Tikhanoff and Matteo Santoro
 * email:  vadim.tikhanoff@iit.it  / matteo.santoro@iit.it
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

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include "GL/gl.h"
#include <highgui.h>
#include <cv.h>

#include <stdio.h>
#include <string>
#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

#define CMD_TRAIN               VOCAB4('t','r','a','i')
#define CMD_CLASSIFY            VOCAB4('c','l','a','s')
#define CMD_LOCK                VOCAB4('l','o','c','k')
#define CMD_UNLOCK              VOCAB4('u','n','l','o')
#define CMD_BUILD_DICT          VOCAB4('d','i','c','t')
#define CMD_SAVE                VOCAB4('s','a','v','e')
#define CMD_LOAD                VOCAB4('l','o','a','d')

#if !defined(SIFTGPU_STATIC) && !defined(SIFTGPU_DLL_RUNTIME)
// SIFTGPU_STATIC comes from compiler
#define SIFTGPU_DLL_RUNTIME
// Load at runtime if the above macro defined
// comment the macro above to use static linking
#endif


#ifdef SIFTGPU_DLL_RUNTIME
#include <dlfcn.h>
#define FREE_MYLIB dlclose
#define GET_MYPROC dlsym
#endif

#include "SiftGPU.h"

SiftGPU* (*pCreateNewSiftGPU)(int) = NULL;
SiftMatchGPU* (*pCreateNewSiftMatchGPU)(int) = NULL;


class SIFTLearn: public BufferedPort<Image>
{
private:
	ResourceFinder                      &rf;
	Semaphore                           mutex;
	int                                 feature_size;
	bool                                verbose;

	//cv::Ptr<cv::FeatureDetector>        detector;
	//cv::Ptr<cv::DescriptorExtractor>    extractor;

	vector<SiftGPU::SiftKeypoint>       keypoints;
	vector<float>                       descriptors;

	Port                                outPort;
	bool                                imageLocked;
	volatile bool                       gotNewImage;
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutPort;
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > siftVectorsPort;

	ComboSiftGPU                        *combo;
	SiftMatchGPU                        *matcher;
	SiftGPU                             *sift;


	// The following must be initialized using values stored in the config files

	// maximum number of descriptors randomly sampled during the training phase
	unsigned int  N ;
	// actual number of descriptors sampled since the beginning of the current work session
	unsigned int Ncurrent;
	// number of features in the descriptors
	unsigned int  D ;
	// number of atoms in the dictionary
	unsigned int  K ;
	// minimum time interval between two subsequent random samplings during the training phase
	unsigned int T ;
	// name of the file where the initial random descriptors (i.e. the training data) are stored
	std::string DataFileName ;
	// complete file path of the data file
	yarp::os::ConstString               DataFile;
	// name of the file where the learnt dictionary must be stored for subsequent usage.
	std::string DictFileName ;
	// complete file path of the dictionary file
	yarp::os::ConstString               DictFile;

	// ********
	// Shall we save these values into the config file?
	// ********
	cv::TermCriteria *termcrit ;
	int attempts ;
	int flags ;

	/*
 Boolean FLAGS representing the STATE of the module.
 All should be initialised to FALSE (except for IsComputingSIFTDescs).
  */
	bool IsSampling;
	bool IsDictionaryAvailable;
	bool IsDictionaryBuildingAllowed;
	bool IsRecognitionAllowed;
	bool IsRecognising;
	bool IsComputingSIFTDescs;
	bool IsDictionaryBuilt;
	bool Initialised;

	/*
  Main data structures representing the training data and the dictionary respectively
  */
	cv::Mat Xtrain;
	cv::Mat Dict;

	/*
 Variables used during the sub-sampling of training examples
  */
	std::vector<int> keypointIndices;
	double currentTime, lastSampling;
	int Nsamples;

	/*
 Variables used to build the super-vectors
  */
	float param_s; // non-negative constant used in the original SV formulation
	unsigned int num_levels_spm; // number of levels used to build the spatial pyramid
	unsigned int nn_used; // number of nearest dictionary elements that will not be set to zero in the SV
	float svDim; // total number of components of each super vector

	unsigned int progrIndex;


	/*
 Variable required to setup the layout of the grid of keypoints in the first layer of the architecture
  */
	//border size in pixels
	int border_x;
	int border_y;
	// spacing size in pixels
	int keypoints_x_spacing;
	int keypoints_y_spacing;
	//set how many kepoints are needed to fill width and height
	int keypointsWidth;
	int keypointsHeight;


	/*
 Variables required to map the range of supevectors to [0,255]
  */
	cv::Scalar mean, stddev;


	/*
 Variable holding the name of the robot port
  */
	std::string robot;

	virtual void onRead( Image &img )
	{
		static double in = Time::now();
		//fprintf(stdout, "the time is %lf\n",Time::now()-in);
		mutex.wait();
		if( !imageLocked )
		{
			static bool first=true;
			if( first )
			{
				if( sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED )
					fprintf( stdout,"Cannot create context\n" );
				matcher->VerifyContextGL();
				first=false;
			}

			IplImage *ipl=(IplImage*) img.getIplImage();
			IplImage *dst = 0;

			//after the image is received apply a guassian lowpass filter to avoid a lot of sift flickering

			// CV_GAUSSIAN_5x5 A 5x5 Gaussian lowpass filter.
			// This filter uses the kernel A/571,where
			//         2  7  12  7  2
			//         7 31  52 31  7
			//    A =  12 52 127 52 12
			//         7 31  52 31  7
			//         2  7  12  7  2

			float k[25] = {  2.f/571, 7.f/571, 12.f/571, 7.f/571, 2.f/571,
							 7.f/571, 31.f/571, 52.f/571, 31.f/571, 7.f/571,
							 12.f/571, 52.f/571, 127.f/571, 52.f/571, 12.f/571,
							 7.f/571, 31.f/571, 52.f/571, 31.f/571, 7.f/571,
							 2.f/571, 7.f/571, 12.f/571, 7.f/571, 2.f/571
						  };
			CvMat Km;
			Km = cvMat( 5, 5, CV_32F, k );
			dst = cvCloneImage( ipl );

			cvFilter2D( ipl, dst, &Km, cvPoint(-1,-1) );

			double t=Time::now();

			if ( !Initialised )
			{
				this->initKeypointIndices(dst->width, dst->height);
			}

			//provide the custom keypoints to the sift library
			sift->SetKeypointList( keypoints.size(), &keypoints[0], 0 );

			//run the sift for the image with custom keypoints
			sift->RunSIFT( dst->width,dst->height,dst->imageData,GL_RGB,GL_UNSIGNED_BYTE );
			int feature_num=sift->GetFeatureNum();

			if( verbose )
				fprintf(stdout,"%d features extracted in %f [s]\n",feature_num,Time::now()-t);

			//resize descriptors and get them from sift
			if( feature_num>0 )
			{
				descriptors.resize( feature_size*feature_num );
				sift->GetFeatureVector( &keypoints[0],&descriptors[0] );
			}
			gotNewImage=true;

			//display it all if connection is available
			if( outPort.getOutputCount() )
			{
				for( unsigned int i=0; i<keypoints.size(); i++ )
					cvCircle( img.getIplImage(),cvPoint(cvRound(keypoints[i].x),cvRound(keypoints[i].y)),2,cvScalar(255),2 );

				outPort.write( img );
			}

			cvReleaseImage( &dst );
		}
		mutex.post();

		if( IsSampling )
		{
			currentTime = Time::now();

			if ( currentTime - lastSampling >= T )
			{
				std::random_shuffle ( keypointIndices.begin(), keypointIndices.end() );
				for ( int idx = 0; idx < Nsamples; idx++ )
				{
					for ( int j = 0; j < keypoints.size(); j++ )
					{
						Xtrain.at<float>(Ncurrent,j) = descriptors[keypointIndices[idx]+j];
					}
					Ncurrent++;
					if ( Ncurrent >= N-1 )
					{
						IsSampling = false;
						std::cout << "****************************************************" << std::endl;
						std::cout << "Maximum number of training examples reached. Disabling random sampling of SIFT descriptors." << std::endl;
						std::cout << "****************************************************" << std::endl;
						return;
					}
				}

				std::cout << "A new set of training samples has been acquired and the sampling time has been updated." << std::endl;
				lastSampling = Time::now();

				if ( Ncurrent >= K ) {
					IsDictionaryBuildingAllowed = true;
					std::cout << "Dictionary building is now allowed." << std::endl;
				}
			}
		}

		if (IsRecognising){
			// Increment the index used to build the filename to store supervectors images
			progrIndex++;

			double startingTime = Time::now(), requiredTime = 0.0;

			int Nfeat = sift->GetFeatureNum();
			int Nmeans = 0;
			for (int ell = 0; ell < num_levels_spm; ell++){
				Nmeans += std::pow(2.0, 2.0*ell);
			}
			cv::Mat descs_tmp = cv::Mat(this->D, Nfeat, CV_32F, &descriptors[0]);
			cv::Mat descs = cv::Mat(Nfeat, this->D, CV_32F, &descriptors[0]);
			transpose(descs_tmp, descs);

			if (siftVectorsPort.getOutputCount() > 0){
				cv::Mat SVMono = cv::Mat(descs.cols, descs.rows, CV_8UC1);
				descs.convertTo(SVMono, CV_8UC1, 255, 0);
				//cv::imwrite(filename, SVMono);

				IplImage* img = new IplImage(SVMono);
				ImageOf<PixelMono> &temp = siftVectorsPort.prepare();
				temp.resize(img->width,img->height);
				cvCopyImage( img, (IplImage *) temp.getIplImage());
				siftVectorsPort.write();
				delete img;
			}


			cv::BruteForceMatcher<cv::L2<float> > matcher;
			vector<cv::DMatch> matches;
			matcher.match(this->Dict, descs, matches);

			cv::Mat supervector[Nfeat];
			cv::Mat meanvectors[Nmeans];

			for (int n = 0; n < Nfeat; n++) {
				supervector[n] = cv::Mat(this->K, this->D+1, CV_32F);
				supervector[n].setTo(0.0);
				int vstar = matches[n].trainIdx;
				for (int i = 1; i <= D; ++i){
					supervector[n].at<float>(vstar, i) = descs.at<float>(n,i-1)-this->Dict.at<float>(vstar, i-1);
				}
				supervector[n].at<float>(vstar, 0) = this->param_s;
			}
			matches.clear();

			double currLevel = 0.0;
			int n = 0;
			Nmeans = 0;
			for (int ell = 0; ell < num_levels_spm; ell++){

				currLevel=std::pow(2.0, ell);
				for (int segment_x = 1; segment_x <= currLevel; segment_x++){
					for (int segment_y = 1; segment_y <= currLevel; segment_y++){

						meanvectors[Nmeans] = cv::Mat(this->K, this->D+1, CV_32F);
						meanvectors[Nmeans].setTo(0.0f);
						int nnnn = 0;
						// WARNING this may be troublesome. Please verify the correctness of the for loops both in case of
						// even and odd number of keypoints along the two axes.
						for ( int i = (segment_x-1)*std::floor(keypointsWidth/currLevel); i< segment_x*std::floor(keypointsWidth/currLevel); i++ ){
							for ( int j = (segment_y-1)*std::floor(keypointsHeight/currLevel); j< segment_y*std::floor(keypointsHeight/currLevel); j++ ){
								n = j*keypointsWidth+i;
								meanvectors[Nmeans] += supervector[n];
								nnnn++;
							}
						}
						meanvectors[Nmeans]/=nnnn;
						Nmeans++;
					}
				}
			}

			std::cout << std::endl;
			requiredTime = Time::now() - startingTime;
			std::cout << "Second layer of the feature vector successfully built after " << requiredTime << " seconds. " << std::endl;

			int ROWS = static_cast<int>(std::pow(2.0, num_levels_spm-1));
			int COLS = 0;
			for (int i = 0; i < num_levels_spm; i++){
				COLS += static_cast<int>(std::pow(2.0, i ));
			}
			COLS *= (this->D+1);
			ROWS *= this->K;
			cv::Mat supervectorImg = cv::Mat(ROWS, COLS, CV_32F);
			supervectorImg.setTo(0.0f);
			int currSVIndex = 0;
			cv::Mat currSuperVector;

			int start_x = 0;
			int start_y = 0;
			for (int i = 0; i < num_levels_spm; i++){
				currLevel = static_cast<int>(std::pow(2.0, i ));
				for (int segment_x = 1; segment_x <= currLevel; segment_x++){
					start_y = 0;
					for (int segment_y = 1; segment_y <= currLevel; segment_y++){

						int x, y;
						int ii, jj;
						for (x = start_x*(this->D+1),ii=0; x < (start_x+1)*(this->D+1); x++, ii++) {
							for (y = start_y*(this->K),jj=0; y < (start_y+1)*this->K; y++, jj++) {
								supervectorImg.at<float>(y,x) = meanvectors[currSVIndex].at<float>(jj,ii);
							}
						}
						currSVIndex++;
						start_y+=1;
					}
					start_x+=1;
				}
			}

			if (imageOutPort.getOutputCount() > 0){
				char filename[50];
				sprintf(filename, "outimage%05i.pgm", progrIndex);

				cv::Mat SVMono = cv::Mat(supervectorImg.cols, supervectorImg.rows, CV_8UC1);
				if(progrIndex <= 1){
					meanStdDev(supervectorImg, mean, stddev);
					std::cout << "Computed mean (" << mean[0] << ") and standard deviation (" << stddev[0] << ")." << std::endl;
				}
				supervectorImg.convertTo(SVMono, CV_8UC1, 127.0/stddev[0], 127);
				//cv::imwrite(filename, SVMono);

				IplImage* img = new IplImage(SVMono);
				ImageOf<PixelMono> &temp = imageOutPort.prepare();
				temp.resize(img->width,img->height);
				cvCopyImage( img, (IplImage *) temp.getIplImage());
				imageOutPort.write();
				delete img;
			}
		}
	}

	void initKeypointIndices( int width, int height )
	{
		yarp::os::Network::connect("/siftGPULearn/img:o", "/sifts");
		yarp::os::Network::connect("/siftGPULearn/sv:o", "/supervectors");
		yarp::os::Network::connect("/siftGPULearn/sift:o", "/imgsift");

		//set how many kepoints are needed to fill width and height
		this->keypointsWidth = 1+(width-2*this->border_x) / this->keypoints_x_spacing;
		this->keypointsHeight = 1+(height-2*this->border_y) / this->keypoints_y_spacing;
		//resize the keypoints
		keypoints.resize(this->keypointsWidth*this->keypointsHeight);
		//set num keypoints and init counter
		//int numKeypoints = (keypointsWidth*keypointsHeight);
		int key_cnt=0;

		//run through width and height setting the keypoints positions
		for ( int i = 0; i<keypointsWidth; i++ )
		{
			for ( int j=0; j<keypointsHeight; j++ )
			{
				keypoints[key_cnt].x = cvRound( this->keypoints_x_spacing*i + this->border_x );
				keypoints[key_cnt].y = cvRound( this->keypoints_y_spacing*j + this->border_y );
				keypoints[key_cnt].s = 10; // need to decide which scale to use
				key_cnt++;
			}
		}

		for( int i=0; i < key_cnt; i++ )
			this->keypointIndices.push_back(i);

		this->Initialised = true;
	}




	void sample( vector<float>  descr )
	{

	}

	void lock( Bottle &reply )
	{
		mutex.wait();
		imageLocked=true;

		reply.addString("image locked");
		mutex.post();
	}

	void unlock( Bottle &reply )
	{
		mutex.wait();
		imageLocked=false;

		reply.addString("image unlocked");
		mutex.post();
	}

	void load_dictionary()
	{
		/*
	The dictionary can be loaded at any time without any prerequisites.
	After the dictionary has been loaded, any further acquisition of SIFT
	descriptors for training is disabled.
 */
		cv::FileStorage fs(this->DictFileName, cv::FileStorage::READ);
		fs["dictionary"] >> Dict;

		// WARNING: here we should add a control to check that the number of atoms in the loaded dictionary is equal to K

		std::cout << "Dictionary loaded from file:" << this->DictFileName << "." << std::endl;
		std::cout << "Suspending the training phase and preparing for recognition." << std::endl;

		IsSampling = false;
		IsDictionaryAvailable = true;
		IsRecognitionAllowed = true;
		return;
	}

	void build_dictionary()
	{
		if ( IsDictionaryBuildingAllowed )
		{
			cv::Mat Xprime;
			if ( Ncurrent == N-1 )
			{
				Xprime = Xtrain;
			}
			else
			{ // if Ncurrent < N

				std::cout << "WARNING: the current number of training examples is less that expected. "
						  << std::endl << "The dictionary will be built using only the first "
						  << Ncurrent << " descriptors acquired so far." << std::endl;
				Xprime = Xtrain.clone();
				Xprime.resize(Ncurrent);
			}
			cv::Mat labels;
			labels = cv::Mat::zeros( N,1, CV_32F );

			kmeans( Xtrain, K, labels, *(this->termcrit), this->attempts, this->flags, &Dict );

			IsSampling = false;
			IsDictionaryAvailable = true;
			IsRecognitionAllowed = true;
			IsDictionaryBuilt = true;
		}
		else
		{
			std::cout << "ERROR. The module is unable to build the dictionary." << std::endl
					  << "This may be due to a limited number of training examples available."
					  << std::endl << "The current number of samples is: " << Ncurrent
					  << ", while the size of the dictionary is: " << K << ". " << std::endl;
		}
		return;
	}

	void save_dictionary()
	{

		if( IsDictionaryBuilt )
		{
			cv::FileStorage fs( this->DictFileName, cv::FileStorage::WRITE );
			fs << "dictionary" << Dict;

			std::cout << "The dictionary built during this work session has been successfully saved in the file " << this->DictFileName << std::endl;
		}
		else
		{
			std::cout << "WARNING: no new dictionary has been created during the current work session. Ignoring the saving request." << std::endl;
		}
		return;
	}


	void save_training_data()
	{
		cv::Mat Xprime;
		if ( Ncurrent == N -1 )
		{
			Xprime = Xtrain;
		}
		else
		{ // if Ncurrent < N
			Xprime = Xtrain.clone();
			Xprime.resize(Ncurrent);
		}

		cv::FileStorage fs( this->DataFileName, cv::FileStorage::WRITE );
		fs << "data" << Xprime;
		std::cout << "The current dataset has been successfully saved in the file " << this->DataFileName << std::endl;
	}

	void load_training_data()
	{

		cv::FileStorage fs( this->DataFileName, cv::FileStorage::READ );
		fs["data"] >> Xtrain;

		Ncurrent = Xtrain.rows;

		if( Ncurrent < N -1)
		{
			Xtrain.resize(N);
			std::cout << "WARNING: the number of samples contained in the file " << this->DataFileName << " is smaller than the maximum number of training examples allowed in this work session. "
					  << "Enabled the sampling of new descriptors. If you don't want to continue and complete the training phase, please disable it." << std::endl;
			IsSampling = true;
		}
		else if ( Ncurrent == N )
		{
			IsSampling = false;
		}
		else
		{
			Xtrain.resize(N);
			IsSampling = false;
			std::cout << "WARNING: the number of samples contained in the file " << this->DataFileName << " is larger than the maximum number of training examples allowed in this work session. "
					  << std::endl << "Please verify the correctness of the input settings. " << std::endl
					  << "Using only the first " << N << "examples and ignoring the others. " << std::endl;
		}

		if ( Ncurrent >= K )
		{
			IsDictionaryBuildingAllowed = true;
		}
		std::cout << "Data loaded from file:" << this->DataFileName << "." << std::endl;
	}


	void build_super_vector(){
		//cv::flann::Index_<float> idx;
	}

public:
	SIFTLearn(ResourceFinder &_rf):BufferedPort<Image>(),rf(_rf), attempts(10),
		flags(cv::KMEANS_RANDOM_CENTERS), IsSampling(false),
		IsDictionaryAvailable(false), IsDictionaryBuildingAllowed(false),
		IsRecognising(false), IsComputingSIFTDescs(true),
		IsRecognitionAllowed(false), IsDictionaryBuilt(false), Initialised(false), progrIndex(0)
	{
		verbose=rf.check("verbose");
		termcrit = new cv::TermCriteria(cv::TermCriteria::COUNT, 10, 1e-2);

		//features parameters initialization
		Bottle &bGeneral=rf.findGroup("general");

		string feature_name=bGeneral.find("feature_name").asString().c_str();

		Bottle &bFeature=rf.findGroup(feature_name.c_str());

		feature_size=bFeature.find("feature_size").asInt();
		D = feature_size;

		Bottle &bTrain=rf.findGroup("train");
		N = bTrain.find("sample_cnt").asInt();
		Ncurrent = 0;
		T = bTrain.find("sample_spacing").asInt();
		Nsamples = bTrain.find("samples_per_image").asInt();
		K = bTrain.find("dict_size").asInt();
		DataFileName = bTrain.find("data_file").asString().c_str();
		DictFileName = bTrain.find("dict_file").asString().c_str();

		fprintf(stdout,"the number of samples is %d, with spacing %d\n", N, T);
		fprintf(stdout, "The file name is is %s \n", DataFileName.c_str());

		DataFile=rf.findFile(DataFileName.c_str()).c_str();
		DictFile=rf.findFile(DictFileName.c_str()).c_str();

		this->Xtrain = cv::Mat::zeros(this->N, this->D, CV_32F);
		this->Dict = cv::Mat::zeros(this->K, this->D, CV_32F);

		Bottle &bLayout = rf.findGroup("LAYOUT");

		this->border_x = bLayout.find("border_x").asInt();
		this->border_y = bLayout.find("border_y").asInt();
		// spacing size in pixels
		this->keypoints_x_spacing = bLayout.find("spacing_x").asInt();
		this->keypoints_y_spacing = bLayout.find("spacing_y").asInt();

		Bottle &bSV = rf.findGroup("SVC");
		this->param_s = static_cast<float>(bSV.find("param_s").asDouble());
		this->num_levels_spm = static_cast<float>(bSV.find("num_levels_spm").asDouble());
		this->nn_used  = static_cast<float>(bSV.find("nn_used").asDouble());
		this->svDim = K*(D+1);

		string name=rf.find("name").asString().c_str();


		Bottle &bRobot = rf.findGroup("ROBOT");
		this->robot = bRobot.find("type").asString();
		if (!this->robot.compare("simulator") || !this->robot.compare("sim") ) {
			this->robot = std::string("/icubSim/");
		}else {
			this->robot = std::string("/icub");
		}

		outPort.open(("/"+name+"/img:o").c_str());
		imageOutPort.open(("/"+name+"/sv:o").c_str());
		siftVectorsPort.open(("/"+name+"/sift:o").c_str());

		char * pPath;
		pPath = getenv ("SIFTGPU_DIR");
		printf ("\n\nThe current path is: %s\n\n",pPath);

		string str = pPath;

		str.append("/bin/libsiftgpu.so");
		void * hsiftgpu = dlopen(str.c_str(), RTLD_LAZY);

		pCreateNewSiftGPU = (SiftGPU* (*) (int)) GET_MYPROC(hsiftgpu, "CreateNewSiftGPU");
		pCreateNewSiftMatchGPU = (SiftMatchGPU* (*)(int)) GET_MYPROC(hsiftgpu, "CreateNewSiftMatchGPU");
		sift = pCreateNewSiftGPU(1);
		matcher = pCreateNewSiftMatchGPU(4096);

		SiftGPU  *sift = new SiftGPU;
		SiftMatchGPU *matcher = new SiftMatchGPU(4096);

		char * argv[] = {(char*)"-fo", (char*)"-1", (char*) "-v",(char*) "1", (char*)"-winpos",(char*)"-maxd", (char*)"1024"};
		int argc = sizeof(argv)/sizeof(char*);


		sift->ParseParam( argc, argv );
		sift->SetVerbose(0);
		/////////////////////////////////////////////

		BufferedPort<Image>::useCallback();

		gotNewImage=false;
		imageLocked=false;

		currentTime = Time::now();
		lastSampling = Time::now();

	}


	std::string getRobotPortName(){
		return this->robot;
	}

	virtual void close()
	{
		outPort.close();
		BufferedPort<Image>::close();
		delete matcher;
		delete sift;
	}


	bool execReq( const Bottle &command, Bottle &reply )
	{
		switch( command.get(0).asVocab() )
		{
		case( CMD_CLASSIFY ):
		{
			IsRecognising = true;
			IsSampling = false;
			reply.addString("Entered CLASSIFY mode.");
			return true;
		}

		case( CMD_TRAIN ):
		{
			int cmd = command.get(1).asInt();
			if (cmd)
			{
				std::cout << "Starting sampling ..." << std::endl;
				IsSampling = true;
				IsRecognising = false;
				reply.addString("Turned sampling ON");
			}
			else
			{
				std::cout << "Concluding sampling ..." << std::endl;
				IsSampling = false;
				reply.addString("Turned sampling OFF");
			}
			return true;
		}

		case ( CMD_SAVE ):
		{
			yarp::os::ConstString cmd = command.get(1).asString();
			if (cmd == ConstString("data"))
			{
				save_training_data();
				reply.addString("Saving data completed");
			}
			else if (cmd == ConstString("dict"))
			{
				save_dictionary();
				reply.addString("Saving dict completed");
			}
			else
			{
				reply.addString("error in command");
				std::cout << "ERROR: unrecognized parameter provided to the SAVE command." << std::endl;
			}
			return true;
		}

		case ( CMD_LOAD ):
		{
			yarp::os::ConstString cmd = command.get(1).asString();
			if (cmd == ConstString("data"))
			{
				load_training_data();
				reply.addString("loading data completed");
			} else if (cmd == ConstString("dict"))
			{
				load_dictionary();
				reply.addString("loading dict completed");
			} else
			{
				std::cout << "ERROR: unrecognized parameter provided to the LOAD command." << std::endl;
				reply.addString("error in load parameters");
			}
			return true;
		}

		case ( CMD_BUILD_DICT ):
		{
			if ( !IsDictionaryBuildingAllowed )
			{
				std::cout << "The number of training examples is less that the number of atoms in the dictionary. Please keep sampling!" << std::endl;
				return true;
			}
			IsRecognising = false;
			IsSampling = false;
			build_dictionary();
			if( IsDictionaryBuilt )
			{
				std::cout << "Dictionary successfully built. " << std::endl;
				reply.addString("Dictionary successfully built");
			}
			else
			{
				std::cout << "An error occurred while building the dictionary." << std::endl;
				reply.addString("Dictionary built error");
			}
			return true;
		}

		case( CMD_LOCK ):
		{
			lock(reply);
			return true;
		}

		case( CMD_UNLOCK ):
		{
			unlock(reply);
			return true;
		}

		default:
			return false;
		}
	}

};


class siftModule: public RFModule
{
protected:
	SIFTLearn       *siftLearn;
	Port            rpcPort;

public:
	siftModule()
	{}

	virtual bool configure( ResourceFinder &rf )
	{
		string name=rf.find("name").asString().c_str();

		Time::turboBoost();

		siftLearn=new SIFTLearn(rf);
		siftLearn->open(("/"+name+"/img:i").c_str());
		rpcPort.open(("/"+name+"/rpc").c_str());
		attach(rpcPort);

		//yarp::os::Network::connect("/icub/camcalib/left/out" , "/siftGPULearn/img:i");
		//yarp::os::Network::connect("/icubSim/cam/left", "/siftGPULearn/img:i");
		yarp::os::Network::connect(siftLearn->getRobotPortName().append("cam/left").c_str(), "/siftGPULearn/img:i");

		return true;
	}

	virtual bool close()
	{
		siftLearn->interrupt();
		siftLearn->close();
		delete siftLearn;

		rpcPort.interrupt();
		rpcPort.close();

		return true;
	}

	virtual bool respond( const Bottle &command, Bottle &reply )
	{
		if( siftLearn->execReq(command,reply) )
			return true;
		else
			return RFModule::respond( command,reply );
	}

	virtual double getPeriod()    { return 1.0;  }
	virtual bool   updateModule()
	{
		//siftLearn->update();
		return true;
	}

};

int main(int argc, char *argv[])
{
	Network yarp;

	if ( !yarp.checkNetwork() )
		return -1;

	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultContext("siftGPULearn/conf");
	rf.setDefaultConfigFile("config.ini");
	rf.configure("ICUB_ROOT",argc,argv);
	rf.setDefault("name","siftGPULearn");
	siftModule mod;

	return mod.runModule(rf);
}
