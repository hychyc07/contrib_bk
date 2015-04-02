

#include "Extractor.h"
#include "ExtractorUtils.h"

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>


//includes for the the extractors
//#include "DummyExtractor.h"
//#include "HmaxExtractor.h"
//#include "HighDimensionalityFeatureExtractor.h"




Extractor::Extractor(ResourceFinder &rf)
{
    context_path=rf.getContextPath().c_str();
    save_path=rf.check("save_path",Value(context_path.c_str())).asString().c_str();
    name=rf.find("name").asString().c_str();
    string path=rf.find("data_path").asString().c_str();

    type=rf.find("type").asString().c_str();

    max_samples=rf.check("max_samples",Value(-1)).asInt();
    feature_size=rf.check("feature_size",Value(-1)).asInt();

    mode=rf.find("mode").asString()=="load"?MODE_LOAD:MODE_STREAM;
    state=rf.find("state").asString()=="dictionarize"?STATE_DICTIONARIZE:STATE_EXTRACT;

    class_list.clear();
    if(mode==MODE_LOAD && rf.check("class_list"))
    {
        Bottle *bClasses=rf.find("class_list").asList();
        for(int i=0; i<bClasses->size(); i++)
            class_list.push_back(bClasses->get(i).asString().c_str());

        n_samples=rf.find("n_samples").asInt();
    }
}


bool Extractor::release()
{
    if(f_ini.is_open())
        f_ini.close();

    if(f_dat.is_open())
        f_dat.close();

    release_data();

    return true;
}



bool Extractor::init(const string &experiment_name)
{
    release();

    time_tag=generate_time_tag();

    if(mode==MODE_STREAM)
    {
        //first clear everything
        release_data();
        n_samples=0;

        string tmp_name=name+"-"+time_tag;
        if(experiment_name!="")
        tmp_name=experiment_name+"-"+tmp_name;

        f_ini.open((save_path+"/"+tmp_name+".ini").c_str(),fstream::out);
        f_dat.open((save_path+"/"+tmp_name+".dat").c_str(),fstream::out | ios_base::binary);
    }

    if(mode==MODE_LOAD)
    {
        f_dat.open((context_path+"/"+name+".dat").c_str(),fstream::in | ios_base::binary);
        n_currently_loaded=0;
    }

    return init_impl();
}


bool Extractor::update_class_list(Bottle &labels)
{
    for(int i=0; i<labels.size(); i++)
    {
        bool found=false;
        for(unsigned int j=0; j<class_list.size(); j++)
            if(strcmp(labels.get(i).asString().c_str(),class_list[j].c_str())==0)
                found=true;
        if(!found)
            class_list.push_back(labels.get(i).asString().c_str());
    }

    return true;
}


bool Extractor::update_ini()
{
    if(mode==MODE_STREAM && f_ini.is_open())
    {
        f_ini.seekp(ios_base::beg);

        f_ini << "type" << TABS << type << endl;
        f_ini << "max_samples" << TABS << max_samples << endl;
        f_ini << "feature_size" << TABS << feature_size << endl;

        if(state==STATE_EXTRACT)
            f_ini << "mode" << TABS << "load" << endl;


        f_ini << "class_list" << TABS << "(";

        for(unsigned int i=0; i<(int)class_list.size(); i++)
        {
            f_ini<<class_list[i];
            if(i+1<class_list.size())f_ini<<" ";
        }
        f_ini << ")" << endl;

        f_ini << "n_samples" << TABS << n_samples << endl;

        return update_ini_impl();
    }

    return true;
}


bool Extractor::save()
{
    bool saved=false;
    mutex.wait();

    if(max_samples<0 || n_samples<max_samples)
    {
        //encodes the current class occurrences as 0s or 1s.
        int n_classes=class_list.size();
        f_dat.write((char*)&n_classes,sizeof(int));
        for(unsigned int i=0; i<class_list.size(); i++)
        {
            bool present=false;
            for(int j=0; j<labels.size() && !present; j++)
                if(strcmp(class_list[i].c_str(),labels.get(j).asString().c_str())==0)
                    present=true;

            f_dat.write((char*)&present,sizeof(bool));
        }

        saved=save_impl();
        update_ini();
    }

    mutex.post();

    return saved;
}

bool Extractor::load()
{
    if(n_currently_loaded>=n_samples)
        return false;

    //determine the current sample labels
    labels.clear();
    int n_classes;
    f_dat.read((char*)&n_classes,sizeof(int));
    for(int i=0; i<n_classes; i++)
    {
        bool present;
        f_dat.read((char*)&present,sizeof(bool));
        if(present)
            labels.addString(class_list[i].c_str());
    }

    bool loaded=load_impl();
    if(loaded)
        updated_feature_vector=true;

    n_currently_loaded++;

    return loaded;
}

bool Extractor::extract()
{
    bool extracted=false;
    mutex.wait();
    if(is_ready() && mode==MODE_STREAM)
    {
        //check if the number of classes has increased
        update_class_list(labels);
        extracted=extract_impl();
    }
    else if(mode==MODE_LOAD)
    {
        extracted=load();
    }

    mutex.post();

    if(extracted)
    {
        n_samples++;
        release_data();
        updated_feature_vector=true;
    }
   


    return extracted;
}

