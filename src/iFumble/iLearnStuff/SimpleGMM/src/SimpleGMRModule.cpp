/**
\section intro_sec Description

A wrapper for SimpleGMR.h based on GMM libraries from lasa epfl.
This wrapper written by Damien Jade Duff, as of the time of writing.
Based loosely on the kinematics solver module.

\section parameters_sec Parameters
--param \e paramarg [mandatory]
- some words \e emphasised etc

--context \e directory [optional]
- allow to specify a different path where to search the
  configuration file in the form of
  <i> $ICUB_ROOT/app/<directory></i>. Directory default value is
  \e xxxx/conf.

--from \e file [optional]
- allow to specify a different configuration file from the
  default one which is \e yyyy.ini.

\section portsa_sec Ports Accessed

The ports

\section portsc_sec Ports Created

- /iFumble/iLearnStuff/givemedata: obtains data in streaming mode in the form {\e DATAPOINT objectlabel ndim dim_1_value dim_2_value ... dim_ndim_value} (see wiki for more info)
- /<solverName>/rpc : for inference requests of form INFER objectlabel {\e ndim comp_dim_1 comp_dim_2 ... comp_dim_ndim input_val_1 input_val_2 ... input_val_ndim}(see wiki for more info) - also for load and save requests.

\section conf_file_sec Configuration Files

Here's how the configuration file will look like:

\code
No config file used at present - everything figured out online. Automatisch. 
\endcode

\note no note

\section tested_os_sec Tested OS
Umm...

\author Damien Jade Duff
*/

//#include <yarp/os/Network.h>
//#include <yarp/os/RFModule.h>
#include <yarp/os/all.h>

#include <SimpleGMR.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>

#define REPEAT_EM_TILL_DONE 13
#define DECREASE_COMPONENTS_ITER 5
#define MIN_DATASET_SIZE 3
#define NUM_EM_ITER 2000

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace MathLib;

/** 
 * Convert a std::vector of Vectors to a matrix - note there are two kinds of vectors here, the 
 * std::vector kind and the MathLib kind
 */
void vectorVectorToMatrix(const std::vector<Vector> &input,Matrix &output) {
    int num_data=input.size();
    int num_dims=input[0].Size();
    output = Matrix(num_data,num_dims);
    for (int i=0;i<num_data;i++) {
        output.SetRow(input[i],i);
    }
}

/** 
* This thread encapsulate a learner.
* It learns when any new data is coming in, and, using semaphores, swaps in the new learner when it is learnt.
* It also keeps a track of the data.
* No incremental learning, learns from scratch, hence the learning happens in this thread - in the background, so that
* if something has been learnt it can be used untill the new learner is ready..
*/
class GMMLearnerThread: public yarp::os::Thread {
public:

  /** should only be UNLEARNT before enough data arrives - thereafter it will run the learning algorithm to get a learnt model.
  * It will always have a model available thereafter, even while it's learning a new one */
    enum thread_state {UNLEARNT,LEARNT};
    bool forceRelearnFlag;

    /**
    * save map, block while doing so
    */
    bool saveMap(std::string mapname) {
        sem.wait();
	std::cout<<"Saving map to "<<mapname<<std::endl;
        learned_model->saveParams(mapname.c_str());
        sem.post();
    }

    /**
    * save dataset, block while doing so
    */
    bool saveData(std::string dataname) {
        sem.wait();
        Matrix output;
        vectorVectorToMatrix(dataset,output);
	std::cout<<"Saving data to "<<dataname<<std::endl;
        output.Save(dataname.c_str());
        sem.post();
    }

    /**
    * make it do a relearn even if it hasn't detected it is necessary
    */
    void forceRelearn() {
        sem.wait();
        forceRelearnFlag=true;
        sem.post();
    }

    /**
    * add data to the dataset, may initiate a relearn which happens as data grows.
    * Also, if the dimensions of the data are different, will clear out the data and start again
    */
    void addData(Vector data) {
        sem.wait();
        std::cout<<"Adding data "<<data<<std::endl;
        if (dataset.size()>0) {
            if (dataset[dataset.size()-1].Size()!=data.Size()) {
                curr_status=UNLEARNT;
                dataset.clear();
                forceRelearnFlag=true;
                std::cout<<"Dimensions now different - going back to start"<<std::endl;
            }
        }
        dataset.push_back(data);
        sem.post();
    }

    /**
    * start from scratch - will relearn when it gets around to it (and when enough data is added)
    */
    void clearData() {
        sem.wait();
        dataset.clear();
        forceRelearnFlag=true;
        sem.post();
    }

    /**
    * hope this takes negligible time because work is not being spun off - blocking return
    * Uses the GMM objecct to do Gaussian Mixture Regression. on the input vector (with dimensions defined by the inputDimensions vector.
    * Also calculates what the output dimensions should be.
    * Regression finds a mean of a unimodal distribution over the output dimensions.
    * Regression finds a covariance matrix for the regressed that too.
    */
    bool doInference(const Vector &inputVector,const Vector &inputDimensions,Vector &outputMu,Matrix &outputSigma) {
        sem.wait();
        std::cout<<"Doing inference on "<<inputVector;
        //inputVector.Print();
        std::cout<<" which contains dimensions "<<inputDimensions<<std::endl;
        //inputDimensions.Print();
        std::cout<<std::endl;
        if (curr_status!=LEARNT) {
            std::cout<<"But can't do anything because map not yet learnt"<<std::endl;
            sem.post();
            return false;
        }
        if(inputVector.Size()!=inputDimensions.Size()){
	 std::cerr<<"BIG PROBLEM inputVector.Size() == "<<inputVector.Size()<<"!="<<inputDimensions.Size()<<" == inputDimensions.Size()"<<std::endl;
	 
	}
        Vector outputDimensions(learned_model->dim-inputDimensions.Size());
        int ocCntr=0;
        for (int i=0;i<learned_model->dim;i++) {
            bool found=false;
            for (int j=0;j<inputDimensions.Size();j++) {
                if (inputDimensions.At(j)==i) {
                    found=true;
                    j=inputDimensions.Size()+5;
                }
            }
            if (!found) {
                outputDimensions[ocCntr]=i;
                ocCntr++;
            }
        }
        
        std::cout<<"doRegression("<<inputVector<<","<<outputSigma<<","<<inputDimensions<<","<<outputDimensions<<");"<<std::endl;
        outputMu=learned_model->doRegression( inputVector,
                                              outputSigma,
                                              inputDimensions,
                                              outputDimensions);
        sem.post();
        std::cout<<"Regression is returning"<<outputMu<<std::endl;
        return true;
    }

protected:


  /**
  * if the thread is stopping, delete the model, not using it anymore
  */
    void onStop() {
        if (learned_model)delete learned_model;
    }

    void beforeStart() {

    }
    void afterStart() {

    }
    
    /**
    * Does the learning - this happens in the thread and calls the semaphores whenever using objects that are also used by the inference.
    * This also decreases the number of Gaussians as it goes - this might not be 100% necessary since the GMM now does some component deletion.
    * So I have set it to only try a few times.
    */
    bool doLearning() {

        int dim = being_learnt.ColumnSize();
        int num_data = being_learnt.RowSize();
        int numGaussians=dim;
        if (numGaussians>num_data)numGaussians=num_data/2;
        if (numGaussians<1)numGaussians=1;
        if (dim<1)return false;
        if (learning_model)delete learning_model;
        learning_model=new NewGaussianMixture();
        std::cout<<"Preparing to learn - dimension "<<dim<<" num data "<<num_data <<" and num components/gaussians "<<numGaussians<<std::endl;

        int dono=0;
        do {
            learning_model->initEM_random(numGaussians,being_learnt);
            std::cout<<"Doing new EM run on matrix of size "<<being_learnt.RowSize()<<","<<being_learnt.ColumnSize()<<std::endl;
            //std::cout<<"That matrix is "<<being_learnt<<std::endl;
            double lik=learning_model->doEM(being_learnt,0,NUM_EM_ITER);//,int blank_run=0,int max_iter = 100);
            std::cout<<"Rebuilding iteration "<<dono<<" returned "<<lik<<std::endl;
            if (lik!=0)dono+=REPEAT_EM_TILL_DONE;
            else {
                if (numGaussians>1) {
                    //numGaussians=numGaussians/2;
		    numGaussians--;
		    if(numGaussians<1)numGaussians=1;
                    std::cout<<"Decreasing num components to "<<numGaussians<<std::endl;
                }
                dono++;
            }

        } while (dono<REPEAT_EM_TILL_DONE);
	std::cout<<"LEARNED MODEL PRINT:"<<std::endl;
	learning_model->debug();
        std::cout<<"Learnt. Replacing learnt model"<<std::endl;
        sem.wait();
        if (learned_model) delete learned_model;
        learned_model=learning_model;
        learning_model=NULL;
	curr_status=LEARNT;
        std::cout<<"Replaced learnt model"<<std::endl;
        sem.post();
    }

    /**
    * loop, relearning when necessary
    * won't relearn unless there is more data than MIN_DATASET_SIZE
    * will relearn if data amount has grown or if a relearn has been forced.
    */
    void run() {
        int old_dataset_size=0;
        while (!isStopping()) {
            Time::delay(5);
            sem.wait();
            bool dolearn=false;
            int new_dataset_size=dataset.size();
            int calculated_min;

            if (new_dataset_size>0) {
                int dimensions=(int)(dataset[0].Size());
                //calculated_min= std::max(MIN_DATASET_SIZE,dimensions);
		calculated_min= std::max(MIN_DATASET_SIZE,MIN_DATASET_SIZE);
            }
            if (new_dataset_size>=calculated_min&&(new_dataset_size>old_dataset_size||forceRelearnFlag)) {
                std::cout<<"Minimum data required to proceed with learning is "<<calculated_min<<std::endl;
                std::cout<<"Arranging a relearn - old dataset size: "<<old_dataset_size<<" new dataset size: "<<new_dataset_size<<" forceRelearnFlag: "<<forceRelearnFlag<<std::endl;
                old_dataset_size=new_dataset_size;
                forceRelearnFlag=false;
                dolearn=true;
                vectorVectorToMatrix(dataset,being_learnt);
            }
            sem.post();
            if (dolearn) {
                doLearning();
            }
        }

    }

    thread_state curr_status;
    NewGaussianMixture *learned_model;
    NewGaussianMixture *learning_model;
    std::vector<Vector> dataset;
    Matrix being_learnt;
    yarp::os::Semaphore sem;

};


class LearnerModule;




class InferrenceRequestReader : public PortReader {
    LearnerModule *parent;
public:
    virtual bool read(ConnectionReader& connection);
    void setParent(LearnerModule *par);

};
class DatasetReader : public BufferedPort<Bottle> {

    LearnerModule *parent;
public:
    virtual void onRead(Bottle& b) ;
    void setParent(LearnerModule *par);

};

class LearnerModule: public RFModule
{

protected:

    //BufferedPort<Bottle>      inPort;

    bool openPorts;
    DatasetReader dataReader;
    InferrenceRequestReader inferReader;

public:
    Port rpcPort;
    std::vector<GMMLearnerThread*> gmmthreads;
    std::vector<std::string> gmmnames;

    int numLearners() {
        int siz1=gmmnames.size();
        int siz2=gmmthreads.size();
        if (siz1!=siz2)std::cerr<<"BIG PROBLEM - SIZES DISPARATE "<<siz1<<"/"<<siz2<<std::endl;
        return siz1;
    }

    int learnerInd(std::string name) {
        int ind=-1;
        for (int i=0;i<gmmnames.size();i++) {
            if (!gmmnames[i].compare(name))return i;
        }
        return ind;
    }

    std::string learnerName(int ind) {
        return gmmnames[ind];
    }

    int addLearner(std::string name) {

        int ind = learnerInd(name);
        if (ind>=0) {
            std::cerr<<"Learner "<<name<<" Already added at index "<<ind<<std::endl;
            return ind;
        }
        ind = gmmnames.size();
        gmmnames.push_back(name);
        gmmthreads.push_back(new GMMLearnerThread());
        gmmthreads[ind]->start();
        std::cout<<"Adding learner name "<<name<<" index "<<ind<<std::endl;
        while (!gmmthreads[ind]->isRunning()) {
            Time::delay(0.001);
        }
        return ind;
    }

    void deleteLearner(std::string lname) {
        deleteLearner(learnerInd(lname));
    }
    void deleteLearner(int ind) {
        std::string lname=learnerName(ind);
        std::cout<<"Deleting learner no "<<ind<<" name "<<lname<<std::endl;
        if (gmmthreads[ind]!=NULL) {
            gmmthreads[ind]->stop();

            while (gmmthreads[ind]->isRunning()) {
                Time::delay(0.01);
            }
            delete gmmthreads[ind];
            gmmthreads[ind]=NULL;
            gmmnames[ind]="";
        }
    }
    LearnerModule()
    {
        //gmmthread=NULL;

    }

    ~LearnerModule()
    {
        //for(int i=0;i<gmmthreads.size();i++)
//        if (gmmthreads[i]!=NULL)delete gmmthreads[i];
        //for(int i=0;i<numLearners();i++)deleteLearner(i);
        close();

    }
/** 
* is this too hardcoded? should really read config from a file...
*/
    virtual bool configure(ResourceFinder &rf)
    {
 
        std::string name = "iFumble/iLearnStuff";
        dataReader.open(("/"+name+"/givemedata").c_str());
        rpcPort.open(("/"+name+"/rpc").c_str());
        dataReader.setParent(this);
        inferReader.setParent(this);
        rpcPort.setReader(inferReader);
        dataReader.useCallback();
        openPorts=true;
        return true;


    }
/** 
* clean up the threads, close the ports
*/
    virtual bool close()
    {
        for (int i=0;i<numLearners();i++)
            deleteLearner(i);

        if (openPorts) {
            dataReader.close();
            rpcPort.close();
            openPorts=false;
        }
        return true;
    }

/** 
* Module Gets polled, but actually, the polling is currently not doing anything extra.;
* Not doing anything with the polling because each learner has its own thread.
*/
    virtual double getPeriod()
    {
        return 1.0;
    }

    virtual bool updateModule()
    {
        /*if (gmmthread->isClosed() || gmmthread->getTimeoutFlag())
            return false;
        else
            return true;*/
        //if(gmmthread->isRunning()){
//
        return true;

//	}
//	return false;
    }
};


/**
* server to read rpc requests - see wiki for kinds of messages dealt with
*/

bool InferrenceRequestReader::read(ConnectionReader& connection) {

    /** do rpcport */

    printf("Waiting for an RPC message...\n");
    Bottle cmd,response;
    bool ok = cmd.read(connection);
    if (!ok) return false;
    ConnectionWriter *returnToSender = connection.getWriter();

    // process data in b

    printf("Got message: %s\n", cmd.toString().c_str());
    std::string cmdstring = std::string(cmd.get(0).asString().c_str());
    if (!cmdstring.compare("INFER")||!cmdstring.compare("infer")) {

        std::string whichLearner = std::string(cmd.get(1).asString().c_str());
        int whichLearnerN = parent->learnerInd(whichLearner);
        if (whichLearnerN<0)whichLearnerN=parent->addLearner(whichLearner);

        int numComponents=cmd.get(2).asInt();
        Vector components(numComponents);
        Vector input(numComponents);
        for (int i=0;i<numComponents;i++) {
            components[i]=cmd.get(3+i).asDouble();
        }
        for (int i=0;i<numComponents;i++) {
            input[i]=cmd.get(numComponents+3+i).asDouble();
        }
        Vector output;
        Matrix sigmaDump;
        bool res = parent->gmmthreads[whichLearnerN]->doInference(input,components,output,sigmaDump);
        if (!res) {
            response.addString("NOTREADY");
            printf("Sending reply: %s\n", response.toString().c_str());
            response.write(*returnToSender);
            return true;
        } else {
            response.addString("REGRESSED");
            response.addInt(output.Size());
            for (int i=0;i<output.Size();i++) {
                response.addDouble(output[i]);
            }
            printf("Sending reply: %s\n", response.toString().c_str());
            response.write(*returnToSender);
            return true;
        }
    }else if (!cmdstring.compare("SAVEDATA")||!cmdstring.compare("savedata")) {
        std::string whichLearner = std::string(cmd.get(1).asString().c_str());
        int whichLearnerN = parent->learnerInd(whichLearner);      
	std::string saveName = std::string(cmd.get(2).asString().c_str());
	parent->gmmthreads[whichLearnerN]->saveData(saveName);
	response.addString("OK");
	printf("Sending reply to save data request: %s\n", response.toString().c_str());
	response.write(*returnToSender);
    }else if (!cmdstring.compare("SAVEMODEL")||!cmdstring.compare("savemodel")) {
        std::string whichLearner = std::string(cmd.get(1).asString().c_str());
        int whichLearnerN = parent->learnerInd(whichLearner);      
	std::string saveName = std::string(cmd.get(2).asString().c_str());
	parent->gmmthreads[whichLearnerN]->saveMap(saveName);
	response.addString("OK");
	printf("Sending reply to save model request: %s\n", response.toString().c_str());
	response.write(*returnToSender);    
      
    } else if (!cmdstring.compare("RELEARN")||!cmdstring.compare("relearn")) {
        std::string whichLearner = std::string(cmd.get(1).asString().c_str());
        int whichLearnerN = parent->learnerInd(whichLearner);      
	std::string saveName = std::string(cmd.get(2).asString().c_str());
	parent->gmmthreads[whichLearnerN]->forceRelearn();
	response.addString("OK");
	std::cout<<"Forced relearn for learner "<<whichLearner<<": sending response "<< response.toString().c_str()<<std::endl;
	response.write(*returnToSender);    
      
    } else {
        response.addString("UNKNOWNCOMMAND");
        printf("Sending reply: %s\n", response.toString().c_str());
        response.write(*returnToSender);
        return true;
    }


}

/** 
* it's a callback object, so it needs access to the learners
*/
void InferrenceRequestReader::setParent(LearnerModule *par) {
    parent=par;
}


/**
* This callback object takes the data and sends it to the right learner to learn.
* It doesn't spend long as the learner is learning in its own thread.
*/
void DatasetReader::onRead(Bottle& b) {
    // process data in b
    /** do inPort */
    // Bottle *b=inPort.read();
    std::string instring= std::string(b.get(0).asString().c_str());
    if (!instring.compare("DATAPOINT")||!instring.compare("datapoint")) {
        std::string whichLearner = std::string(b.get(1).asString().c_str());
        int whichLearnerN = parent->learnerInd(whichLearner);
        if (whichLearnerN<0)whichLearnerN=parent->addLearner(whichLearner);
        int numDims=b.get(2).asInt();
        Vector vadd = Vector(numDims);
        for (int i=0;i<numDims;i++) {
            vadd[i]=b.get(i+3).asDouble();
        }
        parent->gmmthreads[whichLearnerN]->addData(vadd);
        return;
    }
}

/** 
* it's a callback object, so it needs access to the learners
*/
void DatasetReader::setParent(LearnerModule *par) {
    parent=par;
}


int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(true);
    //the following lines could be nice if ever i put stuff in a config file.
//   rf.setDefaultContext("iLearnStuff/conf");
    //  rf.setDefaultConfigFile("iLearnStuff.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    LearnerModule mod;

    return mod.runModule(rf);
}
