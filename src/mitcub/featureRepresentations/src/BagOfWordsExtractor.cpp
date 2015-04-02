


#include "BagOfWordsExtractor.h"
#include "ExtractorUtils.h"


float distance(cv::Mat &matA, const int &idxA, cv::Mat &matB, const int &idxB)
{
    float dist=0.0;
    for(int i=0; i<matA.cols; i++)
        dist+=(matA.at<float>(idxA,i)-matB.at<float>(idxB,i))*(matA.at<float>(idxA,i)-matB.at<float>(idxB,i));

    return sqrt(dist);
}


BagOfWordsExtractor::BagOfWordsExtractor(ResourceFinder &rf)
    :Extractor(rf)
{
    matcher=cv::DescriptorMatcher::create("BruteForce");

    string dictionary_name=rf.check("dictionary",Value("")).asString().c_str();

    if(dictionary_name.size())
    {
        ResourceFinder dic_rf;
        initResourceFinder(&dic_rf,context_path,dictionary_name);
        fstream f_dic;

        f_dic.open((context_path+"/"+dictionary_name+".dict").c_str(),fstream::in | ios_base::binary);

        n_centers=dic_rf.find("n_words").asInt();
        bow_feature_size=dic_rf.find("word_dimension").asInt();

        centers.create(n_centers,bow_feature_size,CV_32F);

        float *tmp_centers=new float[n_centers*bow_feature_size];
        f_dic.read((char*)tmp_centers,sizeof(float)*n_centers*bow_feature_size);
        for(int i=0; i<centers.rows; i++)
            for(int j=0; j<centers.cols; j++)
                centers.at<float>(i,j)=tmp_centers[i*centers.cols + j];

        dictionarized=true;



        float min_dist=-1.0;
        float max_dist=0.0;
        float mean_dist=0.0;
        float std_dist=0.0;
        int cnt=0;

        for(int i=0; i<centers.rows; i++)
        {
            for(int j=i+1; j<centers.rows; j++)
            {
               float dist=distance(centers,i,centers,j);
               if(dist<min_dist || min_dist<0)
                    min_dist=dist;

               if(dist>max_dist)
                    max_dist=dist;

                mean_dist+=dist; 
                std_dist+=dist*dist;
                cnt++;
            }
        }
        
        mean_dist=(float)(1.0/cnt)*mean_dist;
        std_dist=sqrt((float)std_dist*(1.0/cnt) - mean_dist*mean_dist);

        fprintf(stdout,"min=%f\nmax=%f\nmean=%f\nstd=%f\n\n\n",min_dist,max_dist,mean_dist,std_dist);      


        delete [] tmp_centers;
    }
    else
    {
        if(feature_size<0)
            feature_size=rf.check("n_centers",Value(256)).asInt();

        n_centers=feature_size;
        n_attempts=rf.check("n_attempts",Value(10)).asInt();
        bow_feature_size=rf.check("bow_feature_size",Value(-1)).asInt();
        max_bow_features=rf.check("max_bow_features",Value(100)).asInt();

        current_row=0;
        initialized=false;

        dictionarized=false;
    }


    feature_vector.resize(1,feature_size);
}



bool BagOfWordsExtractor::save_impl()
{
    if(!f_dat.is_open())
    {
        return false;
    }

    double *tmp_vec=new double[feature_size];
    for(int i=0; i<feature_size; i++)
        tmp_vec[i]=feature_vector[0][i];


	for(int i=0; i<feature_size; i++)
           fprintf(stdout,"%f vs %f \n",feature_vector[0][i],tmp_vec[i]); 
        fprintf(stdout,"\n\n");

    f_dat.write((char*)tmp_vec,sizeof(double)*feature_size);

    delete [] tmp_vec;

    return true;
}


//Note: yet to decide how load/save
bool BagOfWordsExtractor::load_impl()
{
    if(!f_dat.is_open())
    {
        return false;
    }

    double *tmp_vec=new double[feature_size];
    f_dat.read((char*)tmp_vec,sizeof(double)*feature_size);

    for(int i=0; i<feature_size; i++)
        feature_vector[0][i]=tmp_vec[i];

    delete [] tmp_vec;

    return true;
}





bool BagOfWordsExtractor::update_ini_impl()
{
    f_ini << "bow_feature_size" << TABS << bow_feature_size << endl;
    f_ini << "n_attempts" << TABS << n_attempts << endl;
    f_ini << "n_centers" << TABS << n_centers << endl;
    f_ini << "max_bow_features" << TABS << max_bow_features << endl;

    return true;
}


bool BagOfWordsExtractor::feedData(const Matrix &data)
{
    if(bow_feature_size<0)
        bow_feature_size=data.cols();

    if(bow_feature_size!=data.cols())
        return false;

    switch(state)
    {
        case(STATE_EXTRACT):
        {
            samples.create(data.rows(),data.cols(),CV_32F);
            for(int i=0; i<data.rows(); i++)
                for(int j=0; j<data.cols(); j++)
                    samples.at<float>(i,j)=(float)data[i][j];
            break;
        }

        case(STATE_DICTIONARIZE):
        {
            if(!initialized && max_bow_features>0)
            {
                samples.create(max_bow_features,bow_feature_size,CV_32F);
                centers.create(n_centers,bow_feature_size,CV_32F);

                initialized=true;
            }

            if(initialized)
            {
                for(int i=0; i<data.rows() && current_row<max_bow_features; i++)
                {
                    for(int j=0; j<data.cols(); j++)
                        samples.at<float>(current_row,j)=(float)data[i][j];
                    current_row++;
                }
            }
            break;
        }
    }

    updated_data=true;

    return true;
}

bool BagOfWordsExtractor::extract_impl()
{
    switch(state)
    {
        case(STATE_EXTRACT):
        {
            cv::vector<cv::DMatch> matches;
            matcher->match(samples,centers,matches);



            int cnt=0;
            for(int i=0; i<samples.rows; i++)
                if(0<=matches[i].trainIdx && matches[i].trainIdx<centers.cols)
                    cnt++;

            double unit=(1.0/cnt);


            //clear the feature vector
            feature_vector=0.0;

            for(int i=0; i<samples.rows; i++)
                if(0<=matches[i].trainIdx && matches[i].trainIdx<centers.cols)
                    feature_vector[0][matches[i].trainIdx]+=unit;

            float min_dist=-1.0;
            float max_dist=0.0;
            float mean_dist=0.0;
            float std_dist=0.0;
            int cnt_dist=0;

            for(int i=0; i<samples.rows; i++)
            {
                for(int j=0; j<centers.rows; j++)
                {
                   float dist=distance(samples,i,centers,j);
                   if(dist<min_dist || min_dist<0)
                        min_dist=dist;

                   if(dist>max_dist)
                        max_dist=dist;

                    mean_dist+=dist; 
                    std_dist+=dist*dist;
                    cnt_dist++;
                }
            }
        
        
            mean_dist=(float)(1.0/cnt_dist)*mean_dist;
            std_dist=sqrt((float)std_dist*(1.0/cnt_dist) - mean_dist*mean_dist);

            fprintf(stdout,"min=%f\nmax=%f\nmean=%f\nstd=%f\n\n\n",min_dist,max_dist,mean_dist,std_dist);  

            fprintf(stdout,"feature_vector = %s\n",feature_vector.toString().c_str());

            break;
        }

        case(STATE_DICTIONARIZE):
        {
            break;
        }
    }

    return true;
}


bool BagOfWordsExtractor::dictionarize()
{
    //adjust the shape of the sample matrix in case it was not completely filled.
    if(current_row<max_bow_features)
        return false;


    fprintf(stdout,"dictionarizing!\n");

    cv::Mat cluster_labels(n_centers,1,CV_32S);
    cv::kmeans(samples,n_centers,cluster_labels,cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1),n_attempts,cv::KMEANS_RANDOM_CENTERS,&centers);

    fprintf(stdout,"dictionarized!\n");

    //save the dictionary
    fstream f_dic;
    f_dic.open((context_path+"/"+name+"-"+time_tag+".dict").c_str(),fstream::out | ios_base::binary);
    float *tmp_centers=new float[centers.rows*centers.cols];
    for(int i=0; i<centers.rows; i++)
        for(int j=0; j<centers.cols; j++)
            tmp_centers[i*centers.cols+j]=centers.at<float>(i,j);
    f_dic.write((char*)tmp_centers,sizeof(float)*centers.rows*centers.cols);
    f_dic.close();

    fstream f_dic_ini;
    f_dic_ini.open((context_path+"/"+name+"-"+time_tag+".ini").c_str(),fstream::out);
    f_dic_ini<<"dictionary_type"<<TABS<<type<<endl;
    f_dic_ini<<"n_words"<<TABS<<n_centers<<endl;
    f_dic_ini<<"word_dimension"<<TABS<<bow_feature_size<<endl;
    f_dic_ini<<"dictionary"<<TABS<<name+"-"+time_tag<<endl;
    f_dic_ini.close();

    update_ini();

    dictionarized=true;

    return true;
}


