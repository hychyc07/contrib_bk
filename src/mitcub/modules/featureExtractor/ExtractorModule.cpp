

#include "ExtractorModule.h"

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

//the ExtractorFactory
#include "ExtractorFactory.h"

//includes for the the extractors
#include "DummyExtractor.h"
#include "HmaxExtractor.h"
#include "HighDimensionalityFeatureExtractor.h"
#include "BagOfWordsExtractor.h"


bool ExtractorModule::configure(ResourceFinder &rf)
{
    Bottle &bGeneral=rf.findGroup("general");

    string name=rf.check("name",Value("extractor")).asString().c_str();

    string state_name=bGeneral.find("state").asString().c_str();

    subsample=bGeneral.check("subsample",Value(1)).asInt();
    subsample_itr=0;

    max_samples=bGeneral.check("max_samples",Value(-1)).asInt();

    //determine the state of the module
    if(state_name=="extract")
        state=STATE_EXTRACT;
    if(state_name=="dictionarize")
        state=STATE_DICTIONARIZE;


    mode=bGeneral.check("mode",Value("stream")).asString()=="load"?MODE_LOAD:MODE_STREAM;

    //register all the extractors that will be used
    registerExtractors(rf);


    extractors_sample_count.resize(extractors.size());
    for(int i=0; i<extractors.size(); i++)
        extractors_sample_count[i]=0;


    if(mode==MODE_STREAM)
    {
        port_img.open(("/"+name+"/img:i").c_str());
        port_labels.open(("/"+name+"/label:i").c_str());

        port_dataset_player.open(("/"+name+"/dataset:io").c_str());

        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);
    }

    extracting=false;
    if(rf.check("start"))
        startExperiment();

    return true;
}


bool ExtractorModule::interruptModule()
{
    if(mode=MODE_STREAM)
    {
        port_img.interrupt();
        port_labels.interrupt();

        port_dataset_player.interrupt();

        rpcPort.interrupt();
    }

    return true;
}

bool ExtractorModule::close()
{
    for(unsigned int i=0; i<extractors.size(); i++)
    {
        extractors[i]->releaseExtractor();
        delete extractors[i];
    }

    if(mode==MODE_STREAM)
    {
        port_img.close();
        port_labels.close();

        port_dataset_player.close();

        rpcPort.close();
    }

    return true;
}


bool ExtractorModule::updateModule()
{
    if(isStopping())
        return true;

    bool finished=true;
    for(int i=0; i<extractors_sample_count.size(); i++)
        finished=finished&&(extractors_sample_count[i]>=max_samples || max_samples<0);

    if(finished)
    {
        for(int i=0; i<extractors_sample_count.size(); i++)
            extractors_sample_count[i]=0;

        extracting=false;
    }

    if(!extracting)
        return true;

    if(mode==MODE_STREAM)
    {
        Bottle bStep,bReply;
        bStep.addString("step");

        port_dataset_player.write(bStep,bReply);

        if(bReply.isNull() || bReply.size()==0 || bReply.get(0).asString()!="ok")
            return true;

        if(port_img.getInputCount()>0)
        {
            Image *img=port_img.read(false);

            if(img!=NULL)
            {
                Stamp ts;
                port_img.getEnvelope(ts);

                for(unsigned int i=0; i<extractors.size(); i++)
                    extractors[i]->setImage(img,ts.getTime());
            }
        }

        //labels
        if(port_labels.getInputCount()>0)
        {
            Bottle *labels=port_labels.read(false);
            if(labels!=NULL)
            {
                Stamp ts;
                port_labels.getEnvelope(ts);
                for(unsigned int i=0; i<extractors.size(); i++)
                    extractors[i]->setLabels(labels,ts.getTime());

                for(unsigned j=0; j<dictionarizers.size(); j++)
                    dictionarizers[j]->setLabels(labels,ts.getTime());
            }
        }
    }


    subsample_itr++;

    if(subsample_itr<subsample)
        return true;
    else
        subsample_itr=0;

    vector<bool>   extractors_extracted(extractors.size());
    vector<Matrix> extractors_data(extractors.size());

    //depending on the modality of each extractor the system loads from file or uses the image stream
    //and if it is the case, save the extracted data
    for(unsigned int i=0; i<extractors.size(); i++)
    {
        if(extractors_extracted[i]=extractors[i]->extract())
        {
            extractors[i]->getFeatureVector(&extractors_data[i]);
            if(state==STATE_EXTRACT)
                extractors[i]->save();

            extractors_sample_count[i]++;
            extractors[i]->release_data();
        }
    }

    if(extractors_extracted[0])
        fprintf(stdout,"extracted sift!\n");
    else
        fprintf(stdout,"extracted sift not!\n");
    
    vector<vector<bool> > dictionarizers_extracted(dictionarizers.size());
    vector<vector<Matrix> > dictionarizers_data(dictionarizers.size());
    for(unsigned int j=0; j<dictionarizers.size(); j++)
    {
        dictionarizers_extracted[j].resize(extractors.size());
        dictionarizers_data[j].resize(extractors.size());
    }

    //feed the extracted data to the dictionarizers
    for(unsigned int i=0; i<extractors.size(); i++)
    {
        if(extractors_extracted[i])
        {
            for(unsigned j=0; j<dictionarizers.size(); j++)
            {
                fprintf(stdout,"feeding data\n");
                dictionarizers[j]->feedData(extractors_data[i]);
                fprintf(stdout,"extracting bow\n");
                if(dictionarizers_extracted[j][i]=dictionarizers[j]->extract())
                {
                    fprintf(stdout,"bow extracted\n");
                    dictionarizers[j]->getFeatureVector(&dictionarizers_data[j][i]);

                    fprintf(stdout,"got bow feature vector\n");
                    if(state==STATE_EXTRACT)
                        dictionarizers[j]->save();
                }

                dictionarizers[j]->release_data();
            }
        }

        extractors[i]->release_data();
    }


    if(state==STATE_DICTIONARIZE)
    {
        bool dictionarized=true;
        for(unsigned int j=0; j<dictionarizers.size(); j++)
            if(!dictionarizers[j]->is_dictionarized())
                dictionarized=dictionarized && dictionarizers[j]->dictionarize();

        if(dictionarized)
            stopModule();
    }


    return true;
}



//this method must be modified when a new extractor is defined
bool ExtractorModule::registerExtractors(ResourceFinder &rf)
{
    //register the extractors in the factory
    ExtractorFactory::instance().registerExtractor("dummy_extractor",new DummyExtractor());
    ExtractorFactory::instance().registerExtractor("high_dimensionality_feature_extractor",new HighDimensionalityFeatureExtractor());
    ExtractorFactory::instance().registerExtractor("hmax_extractor",new HmaxExtractor());
    ExtractorFactory::instance().registerExtractor("bag_of_words_extractor",new BagOfWordsExtractor());


    //create the list of extractors that will be used for the current phase
    Bottle &bExtractors=rf.findGroup("extractors");
    for(int i=1; i<bExtractors.size(); i++)
    {
        Extractor *ext=ExtractorFactory::instance().create(rf,bExtractors.get(i).toString().c_str(),STATE_EXTRACT);
        if(ext!=NULL)
            extractors.push_back(ext);
    }


    Bottle &bDictionarizers=rf.findGroup("dictionarizers");
    for(int i=1; i<bDictionarizers.size(); i++)
    {
        Extractor *dic=ExtractorFactory::instance().create(rf,bDictionarizers.get(i).toString().c_str(),state);
        if(dic!=NULL)
            dictionarizers.push_back(dic);
    }

    return true;
}


bool ExtractorModule::initExtractors(const string &experiment_name)
{
    for(int i=0; i<extractors.size(); i++)
        extractors[i]->init(experiment_name);

    for(int i=0; i<dictionarizers.size(); i++)
        dictionarizers[i]->init(experiment_name);

    return true;
}

bool ExtractorModule::startExperiment(string experiment_name)
{
    initExtractors(experiment_name);
    extracting=true;

    return true;
}


bool ExtractorModule::respond(const Bottle &command, Bottle &reply)
{
    if(command.get(0).asString()=="status")
    {
        if(extracting)
            reply.addString("busy");
        else
            reply.addString("idle");
    }
    else if(command.get(0).asString()=="start" && command.size()>1)
    {
        if(extracting)
            reply.addString("busy");
        else
        {
            startExperiment(command.get(1).asString().c_str());
            reply.addString("started");
        }
    }
    else
        return RFModule::respond(command,reply);

    return true;
}


