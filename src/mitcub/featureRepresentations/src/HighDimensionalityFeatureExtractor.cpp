
#include "HighDimensionalityFeatureExtractor.h"


HighDimensionalityFeatureExtractor::HighDimensionalityFeatureExtractor(ResourceFinder &rf)
    :Extractor(rf)
{
    extractor_type=rf.check("extractor_type",Value("SIFT")).asString().c_str();
    detector_type=rf.check("detector_type",Value("SIFT")).asString().c_str();

    feature_size=rf.check("feature_size",Value(128)).asInt();

    detector=cv::FeatureDetector::create(detector_type.c_str());
    extractor=cv::DescriptorExtractor::create(extractor_type.c_str());

    return;
}


bool HighDimensionalityFeatureExtractor::extract_impl()
{
    detector->detect((IplImage*)img.getIplImage(),keypoints);
    extractor->compute((IplImage*)img.getIplImage(),keypoints,descriptors);

    if(keypoints.size()>0)
    {
        if(feature_size<0)
            feature_size=descriptors.cols;

        if(feature_size!=descriptors.cols)
            return false;

        feature_vector.resize(descriptors.rows,descriptors.cols);
        for(unsigned int i=0; i<keypoints.size(); i++)
            for(int j=0; j<descriptors.cols; j++)
                feature_vector[i][j]=descriptors.at<float>(i,j);

        updated_feature_vector=true;

        return true;
    }
    else
        return false;
}


//Note: yet to decide how load/save
bool HighDimensionalityFeatureExtractor::save_impl()
{
    if(!f_dat.is_open())
    {
        return false;
    }

    int n_features=feature_vector.rows();
    f_dat.write((char*)&n_features,sizeof(int));
    for(int i=0; i<n_features; i++)
        for(int j=0; j<feature_size; j++)
            f_dat.write((char*)&feature_vector[i][j],sizeof(double));

    return true;
}


//Note: yet to decide how load/save
bool HighDimensionalityFeatureExtractor::load_impl()
{
    if(!f_dat.is_open())
    {
        return false;
    }

    int n_features;
    f_dat.read((char*)&n_features,sizeof(int));

    feature_vector.resize(n_features,feature_size);
    for(int i=0; i<n_features; i++)
        for(int j=0; j<feature_size; j++)
            f_dat.read((char*)&feature_vector[i][j],sizeof(double));

    updated_feature_vector=true;

    return true;
}


bool HighDimensionalityFeatureExtractor::update_ini_impl()
{
    f_ini << "detector_type" << TABS << detector_type << endl;
    f_ini << "extractor_size" << TABS << extractor_type << endl;

    return true;
}


