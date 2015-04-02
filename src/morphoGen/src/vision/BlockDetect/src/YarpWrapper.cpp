
#undef __GXX_EXPERIMENTAL_CXX0X__
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "yarp/os/all.h"
#include "yarp/sig/all.h"
#include "TLdetector.hpp"
#include <iostream>
#include <fstream>

using namespace yarp::os;
using namespace yarp::sig;
int main(char** argv, int argc)
{
    Network network;
    BufferedPort<ImageOf<PixelRgb> >* imageInputPort  = new BufferedPort<ImageOf<PixelRgb> >();
    BufferedPort<ImageOf<PixelRgb> >* imageOutputPort = new BufferedPort<ImageOf<PixelRgb> >();
	BufferedPort<Bottle>* bbOutputPort = new BufferedPort<Bottle>();
    yarp::os::BufferedPort<yarp::os::Bottle > dataPortMec; 

	imageInputPort->open("/img:i");
    imageOutputPort->open("/img:o");
	//bbOutputPort->open("/bb:o"); editted VM
	network.connect("/icub/camcalib/left/out", "/img:i");
      dataPortMec.open("/Shapedata:o");
     //string para_yml_file = "data/para_cmp8toys.yml"; //modified VM
     string para_yml_file = "data_blocks/para_blocks.yml";

	/////////////////////////////////////////////////
	// STEP1: initiate
	/////////////////////////////////////////////////
	bool flag;
	CTLdetector detector;
	flag = detector.initiate(para_yml_file);
	if (!flag)		return 0;

	/////////////////////////////////////////////////
	// STEP2: train
	/////////////////////////////////////////////////
	flag = detector.train();
	if (!flag)		return 0;

	/////////////////////////////////////////////////
	// STEP3: detect
	/////////////////////////////////////////////////
	int key = 0;
	cv::Mat cvMatImage;
	std::cout<<"***Detecting..."<<std::endl;
	while(key != 27)
	{
		ImageOf<PixelRgb> *img = imageInputPort->read(true);
		cvMatImage.create(img->height(), img->width(), CV_8UC3);
		unsigned char* ptr = img->getRawImage();
		memcpy(cvMatImage.data, ptr, cvMatImage.cols * cvMatImage.rows * 3);
		cv::cvtColor(cvMatImage, cvMatImage, CV_RGB2BGR);

		detector.detect(cvMatImage);
        //detector.showDetObjs(cvMatImage,Scalar(0,255,0),Scalar(255,255,255),480);  //chaged 3.2.2013 Rea changed to orinal size
		detector.showDetObjs(cvMatImage,Scalar(0,255,0),Scalar(255,255,255),0);
		//detector.dispDetObjs();
		std::vector<DetObj> objects;
		objects = detector.getDetObjs();

        //sending out through image port the image out    
        /*
        if(imageOutputPort->getOutputCount()) {
            cv::Mat ppIm = detector.getPostProcessIm();            
            //cv::cvtColor(ppIm, ppIm, CV_BGR2RGB);
    
            // printf("image %d %d \n", cvMatImage.cols, cvMatImage.rows);
            ImageOf<PixelRgb>& tmpImage = imageOutputPort->prepare();  
            tmpImage.resize(img->width(),img->height());
            tmpImage.zero();
            unsigned char* ptrd = tmpImage.getRawImage();
            unsigned char* ptrs = ppIm.data;
            int padding         = tmpImage.getPadding();

            for (int row = 0; row <  img->height(); row++) {
                for(int col = 0; col < img->width(); col++) {
                    *ptrd = 255;
                    ptrd++; 
                    ptrs++;
                    *ptrd = 255;
                    ptrd++; 
                    ptrs++;
                    *ptrd = 255;
                    ptrd++; 
                    ptrs++;
                    //ptrs++;    
                }
                ptrd += padding;
    
            }
    
            
            //memcpy(ptrd,ptrs, cvMatImage.cols * cvMatImage.rows * 3 );
            imageOutputPort->write();
        }
        */

        Bottle& ShapOp = dataPortMec.prepare();
        ShapOp.clear();
        //Bottle output = bbOutputPort->prepare();
        for (int i = 0; i < objects.size(); i++){
            ShapOp.addInt(objects[i].box_tight.x);
            ShapOp.addInt(objects[i].box_tight.y);
            ShapOp.addInt(objects[i].box_tight.width);
            ShapOp.addInt(objects[i].box_tight.height);
            ShapOp.addInt(objects[i].id_label);
            //ShapOp.addInt(10);
            //if want to know the object name: detector.all_obj_cls[objects[i].id_label]
        }

        // free the memory of the objects.
        for (int i=0;i<objects.size();i++){
               objects[i].mat_edge_NN_tr.release();
        }
        objects.clear();
        //bbOutputPort->write();
        dataPortMec.write();

        key = cv::waitKey(100);
	}
	cvMatImage.release();
	std::cout<<"***Done."<<std::endl;
	return 0;
}

