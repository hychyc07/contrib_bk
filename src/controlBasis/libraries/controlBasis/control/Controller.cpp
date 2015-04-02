// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#include "Controller.h"
#include <yarp/os/Network.h>
#include <yarp/math/Math.h>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

CB::Controller::Controller(ControlBasisResource *sen,
                           ControlBasisResource *ref,
                           ControlBasisPotentialFunction *pf,
                           ControlBasisResource *eff) : 
    ControlBasisAction(),
    potentialLast(0),
    potentialDot(0),
    needsJacobian(false),
    needsJacobianInverse(false),
    hasReference(true),
    sensor(sen),
    reference(ref),
    effector(eff),
    potentialFunction(pf),
    jacobian(NULL),
    gain_p(1.0),
    gain_i(1.0),
    gain_d(1.0),
    distributedMode(false),
    useJacobianTranspose(true),
    useDerivativeTerm(false),
    useIntegralTerm(false),
    convergenceStore(0)
{
    
    string refType = "";
    
    // get the types of the resources
    inputSpace = sensor->getResourceType();
    outputSpace = effector->getResourceType();
    pfType = potentialFunction->getSpace();
    
    if(ref != NULL) {
        cout << "Controller HAS reference" << endl;
        hasReference = true;
        reference = ref;
        refType = reference->getResourceType();
        if(refType != inputSpace) {
            cout << "Controller reference and sensor signal don't match (" << refType.c_str() << "," << inputSpace.c_str() << ")!!\n\n";
            exit(0);
        }
    }
    
    if(pfType!=outputSpace) {
        needsJacobian = true;
        cout << "Controller doesn't know how to handle jacobians yet!! -- exiting!!" << endl;
        exit(0);
    } 
    
    actionName = "/cb/controller/";
    actionName += "s:" + sensor->getResourceName() + "/";
    if(hasReference) {
        actionName += "r:" + reference->getResourceName() + "/";
    }
    actionName += "e:" + effector->getResourceName() + "/";
    actionName += "pf:" + potentialFunction->getPotentialName();
    
    numOutputs = 1;
    outputNames.push_back("u");
    
    //    int inputSize = sensor->getResourceDataSize();
    int outputSize = effector->getResourceDataSize();
    if(outputSize!=0) {
        Vout.resize(outputSize);
    }
    //    initPorts(); // mandatory initPorts() function from ControlBasisAction.h    
}

CB::Controller::Controller(ControlBasisResource *sen,
                           ControlBasisPotentialFunction *pf,
                           ControlBasisResource *eff) :
    ControlBasisAction(),
    potentialLast(0),
    potentialDot(0),
    needsJacobian(false),
    needsJacobianInverse(false),
    hasReference(false),
    sensor(sen),
    reference(NULL),
    effector(eff),
    potentialFunction(pf),
    jacobian(NULL),
    gain_p(1.0),
    gain_i(1.0),
    gain_d(1.0),
    distributedMode(false),
    useJacobianTranspose(true),
    useDerivativeTerm(false),
    useIntegralTerm(false),
    convergenceStore(0)
{    

    string refType = "";

    // get the types of the resources
    inputSpace = sensor->getResourceType();
    outputSpace = effector->getResourceType();
    pfType = potentialFunction->getSpace();
    
    if(pfType!=outputSpace) {
        needsJacobian = true;
        cout << "Controller doesn't know how to handle jacobians yet!! -- exiting!!" << endl;
        exit(0);
    } 
    
    actionName = "/cb/controller/";
    actionName += "s:" + sensor->getResourceName() + "/";
    actionName += "e:" + effector->getResourceName() + "/";
    actionName += "pf:" + potentialFunction->getPotentialName();
    
    numOutputs = 1;
    outputNames.push_back("u");
    
    //    int inputSize = sensor->getResourceDataSize();
    int outputSize = effector->getResourceDataSize();
    if(outputSize!=0) {
        Vout.resize(outputSize);
    }
    //    initPorts(); // mandatory initPorts() function from ControlBasisAction.h
}

CB::Controller::Controller(string sen, string ref, string pf, string eff) :
    ControlBasisAction(),
    potentialLast(0),   
    potentialDot(0), 
    needsJacobian(false),
    needsJacobianInverse(false),
    hasReference(true),
    sensor(NULL),
    reference(NULL),
    effector(NULL),
    potentialFunction(NULL),
    jacobian(NULL),
    gain_p(1.0),
    gain_i(1.0),
    gain_d(1.0),
    sensorName(sen),
    referenceName(ref),
    effectorName(eff),
    pfName(pf),
    distributedMode(true),
    useJacobianTranspose(true),
    useDerivativeTerm(false),
    useIntegralTerm(false),
    convergenceStore(0)
{

    cout << endl << "Creating new Controller" << endl;

    // do some string ops to get valid names 
    parseOutputResource();    

    if(!createPotentialFunction(pf)) {
        cout << "Controller can't create potential function!!" << endl;
        return;
    }    

    cout << "Controller " << inputSpace.c_str() << " -> " << outputSpace.c_str() << endl;
    if(outputSpace != inputSpace) {
        needsJacobian = true;
        if(!createJacobian()) {
            cout << "Controller can't create Jacobian!!" << endl;
            return;
        } else {
            cout << "Controller created Jacobian..." << endl;    
        }
    }

    actionName = "/cb/controller/";
    actionName += "s:" + sensorName + "/";
    actionName += "r:" + referenceName + "/";
    actionName += "e:" + effectorName + "/";
    actionName += "pf:" + pfName;

    cout << endl;    

    //    initPorts(); // mandatory initPorts() function from ControlBasisAction.h
}

CB::Controller::Controller(string sen, string pf, string eff) :
    ControlBasisAction(),
    potentialLast(0),   
    potentialDot(0), 
    needsJacobian(false),
    needsJacobianInverse(false),
    hasReference(false),
    sensor(NULL),
    reference(NULL),
    effector(NULL),
    potentialFunction(NULL),
    jacobian(NULL),
    gain_p(1.0),
    gain_i(1.0),
    gain_d(1.0),
    sensorName(sen),
    referenceName(""),
    effectorName(eff),
    pfName(pf),
    distributedMode(true),
    useJacobianTranspose(true),
    useDerivativeTerm(false),
    useIntegralTerm(false),
    convergenceStore(0)
{

    // do some string ops to get valid names 
    parseOutputResource();    

    if(!createPotentialFunction(pf)) {
        cout << "Controller can't create potential function!!" << endl;
        return;
    }
    
    cout << "Controller " << inputSpace.c_str() << " -> " << outputSpace.c_str() << endl;
    if(outputSpace != inputSpace) {
        needsJacobian = true;
        if(!createJacobian()) {
            cout << "Controller can't create Jacobian!!" << endl;
            return;
        } else {
            cout << "Controller Created Jacobian..." << endl;    
        }
    }


    if(!needsJacobian) {
        int outputSize = potentialFunction->getInputSize();
        if(outputSize!=0) {
            Vout.resize(outputSize);
        } 
    }

    actionName = "/cb/controller/";
    actionName += "s:" + sensorName + "/";
    actionName += "e:" + effectorName + "/";
    actionName += "pf:" + pfName;

    cout << endl;

    //    initPorts(); // mandatory initPorts() function from ControlBasisAction.h
}


bool CB::Controller::createPotentialFunction(string pf) {

    bool ok = true;
    vector<string> pfInputs;

    if(!distributedMode) {
        cout << "Controller can't create potential function in local mode..." << endl;
        ok = false;
    }
    
    pfInputs.clear();
    pfInputs.push_back(sensorName);
    if(hasReference) pfInputs.push_back(referenceName);

    cout << "Controller creating PotentialFunction(" << pfInputs[0];
    if(hasReference) cout << ", " << pfInputs[1];
    cout << ")" << endl;

    potentialFunction = PotentialFunctionFactory::instance().createObject(pf,pfInputs);
    if(potentialFunction==NULL) {
        ok = false;
    } else {
        inputSpace = potentialFunction->getSpace();
    }

    return ok;
}


bool CB::Controller::createJacobian() {

    bool ok = true;

    if(!needsJacobian) {
        cout << "Controller doesnt need jacobian!!" << endl;
        ok = false;
        return ok;
    }

    jacobian = JacobianFactory::instance().createObject(inputSpace,outputSpace,deviceName);
    needsJacobianInverse = JacobianFactory::instance().needsInverse(inputSpace,outputSpace);
    jacobian->setSensorName(sensorName);

    if(needsJacobianInverse) {
        cout << "Controller needs INVERSE jacobian" << endl;
    } else {
        cout << "Controller needs  jacobian" << endl;
    }

    if(jacobian==NULL) {
        cout << "Controller needs an unknown Jacobian..." << endl;
        ok = false;
        return ok;
    }

    return ok;
}


void CB::Controller::parseOutputResource() {

    string cb = "/cb/";
    deviceName = "";
    string res = effectorName;

    res.erase(0,cb.size());
    string s = "";
    outputSpace = "";
    unsigned int i=0;

    s = res[i];
    while(s != "/") {
        outputSpace += s;
        s = res[++i];
    }
    for(;i<res.size();i++) {
        deviceName += res[i];
    }
    cout << "Controller found output space = " << outputSpace.c_str() << ", device=" << deviceName.c_str() << endl;

}

bool CB::Controller::updateAction() {

    // gets the potential information (potential, gradient, type)
    // if needs a jacobian (or inverse), multiplies it here
    // computes -J^# \sigma(\tau)
    // stores (\phi, \dot{\phi})
    // sets state of controller

    Vector grad = potentialFunction->getPotentialGradient();
    potential = potentialFunction->getPotential();

    if(grad.size() == 0) return true;
    if(needsJacobian) {
        if(jacobian->getOutputSize()==0 || jacobian->getInputSize()==0) {
            return true;
        }
    }

    if(getIterations()==0) {
        t0 = Time::now();
        t_last = 0;
        dt = 0;
        errInt = 0;
    }

    if(!potentialFunction->isValid()) {
        dynamicState = UNDEFINED;
        started = true;
        return true;
    }

    if(potential != 0) started = true;

    Matrix J(1,grad.size());
    Matrix JT(grad.size(),1);
    Matrix JTinv(1,grad.size());
    Matrix Jinv(grad.size(),1);
    Matrix Jint(1,1);
        
    // copy the gradient of the pf into the jacobian (and its transpose), and take inverses.
    for(int i=0; i<grad.size(); i++) {
        J[0][i] = grad[i];
        JT[i][0] = grad[i];
    }
    JTinv = pinv(JT,0.0);

    for(int i=0; i<grad.size(); i++) {
        Jinv[i][0] = JTinv[0][i];
    }

    // compute the time elapsed since the last update
    double t = fabs(Time::now() - t0);
    dt = t-t_last;
    t_last = t;
    if(dt > 1) dt = 0.1;

    // add in the D and I terms if requested
    double sig = 0;
    sig += (gain_p*potential);
    if(useDerivativeTerm) {
        sig += gain_d*fabs(potentialDot);
    }
    if(useIntegralTerm) {
        errInt += (dt*potential);
        sig += gain_i*errInt;
    }

    /*    
    cout << "running P";
    if(useIntegralTerm) cout << "I";
    if(useDerivativeTerm) cout << "D";
    cout << " controller..." << endl;
    */

    // if there needs to be an intermediate jacobain computed, connect to it here.
    if(needsJacobian) {
        
        if(!needsJacobianInverse) {
            Jint.resize(jacobian->getOutputSize(),jacobian->getInputSize());
            Jint = jacobian->getJacobian();
        } else {
            Jint.resize(jacobian->getInputSize(),jacobian->getOutputSize());
            if(useJacobianTranspose) {
                Jint = jacobian->getJacobianTranspose();            
            } else {
                Jint = jacobian->getJacobianInverse();            
            }
        }

        if(Vout.size() != Jint.rows())
            Vout.resize(Jint.rows());

        Matrix Jfull(Vout.size(),1);
        Matrix JfullT;
        Matrix JfullInv;
        Matrix JfullInvT;

        // copy the full jacobian based on whether we are using transpose or inverse mode
        if(!useJacobianTranspose) {
            Jfull = Jint*Jinv;
        } else {
            Jfull = Jint*JT;
        }

        // compute the controller output
        for(int i=0; i<Vout.size(); i++) {
            Vout[i] = -Jfull[i][0]*sig;            
        }

        if(useJacobianTranspose) {
            Jc = Jfull.transposed();
        } else {
            Jc = pinv(Jfull,0.0);
        }
        
    } else {
        
        // make sure sizes are consistent
        if(Vout.size() != grad.size()) 
            Vout.resize(grad.size());
        
        for(int i=0; i<Vout.size(); i++) {
            if(useJacobianTranspose) {
                Vout[i] = -JT[i][0]*sig;
            } else {
                Vout[i] = -Jinv[i][0]*sig;                        
            }
        }        
        Jc = J;
    }

    // estimate the change in potential and store it
    double pdiff;
    double a = 0.2;
    double b = 0.95;
    int lag = 50;
    double tmp;
    int idx;
    potentialStore.push_back(potential);
        
    if((int)getIterations() >= lag) {

        idx = potentialStore.size() - lag;
        if(idx < 0) idx = 0;
        potentialLast = potentialStore[idx];
        pdiff = (potential - potentialLast)/dt;
        potentialDot = a*pdiff + (1.0-a)*potentialDot; 

        // set the state based on the estimated change in potential
        if(fabs(potentialDot) < 0.1) {
            tmp = 1;
            //dynamicState = CONVERGED;
        } else {
            tmp = 0;
            //dynamicState = UNCONVERGED;
        }

        // this tries to filter out spurious convergence that might only
        // last an iteration or two.  this forces long stretches of
        // convergence (or unconvergence) to occur to actually change the state. 
        convergenceStoreLast = convergenceStore;
        convergenceStore = b*convergenceStoreLast + (1.0-b)*tmp;
        if(convergenceStore > 0.9 ) {
            dynamicState = CONVERGED;
        } else {
            dynamicState = UNCONVERGED;
        }

    } else {
        
        // this is a buffer to not update too quickly
        potentialLast = potentialStore[0];
        pdiff = (potential - potentialLast)/dt;
        potentialDot = a*pdiff + (1.0-a)*potentialDot; 
        dynamicState = UNCONVERGED;

    } 

    if(getIterations()%50 == 0) {
        if(potentialDotStore.size() >= 999) {
            potentialDotStore.erase(potentialDotStore.begin());
        }
    }
    potentialDotStore.push_back(potentialDot);

    //    cout << "Controller potential: " << potential << ", potentialDot: " << potentialDot << ", sizes=["<<potentialStore.size()<<","<<potentialDotStore.size()<<"]"<<endl;

    /*    cout << "Vout: " << endl;
    for(int i=0; i<Vout.size(); i++) {
        cout << Vout[i] << endl;
    }
    */
    return true;
}

void CB::Controller::startAction() {

    cout << "Controller::startAction()" << endl;

    if(isRunning()) {
        if(isSuspended()) {
            cout << "resuming Controller" << endl;
            
            if(!distributedMode) {
                sensor->resume();
                if(hasReference) {
                    reference->resume();
                }
                effector->resume();
            }
            
            if(needsJacobian) {
                if(jacobian->isSuspended()) {
                    jacobian->resume();
                }
            }
            if(potentialFunction->isSuspended()) {
                potentialFunction->resume();
            }
            Time::delay(0.1);
            resume();
        } 

    } else {

        // if in local mode, make sure resources are running
        if(!distributedMode) {
    
            // check to make sure everything is running?
            if(!sensor->isRunning()) {
                cout << "Controller starting sensor..." << endl;
                sensor->startResource();
            }
            if(hasReference) {
                if(!reference->isRunning()) {
                    cout << "Controller starting reference..." << endl;
                    reference->startResource();
                }
            }        
            Time::delay(0.1);
        }
        
        if(needsJacobian) {
            if(!jacobian->isRunning()) {
                cout << "Controller starting Jacobian..." << endl;
                jacobian->startJacobian();
            }
        }
        
        if(!potentialFunction->isRunning()) {
            cout << "Controller starting potential function..." << endl;
            potentialFunction->startPotentialFunction();
            Time::delay(0.1);
        } else {
            cout << "Controller already running potential function..." << endl;
        }
        
        start();     // mandatory start function
    }

}

void CB::Controller::pauseAction() {
    cout << "Controller::pause() -- pausing thread" << endl;
    suspend();
    resetStat();

    potentialFunction->suspend();
    if(needsJacobian) {
        jacobian->suspend();
    }

    if(!distributedMode) {
        sensor->suspend();
        if(hasReference) {
            reference->suspend();
        }
        effector->suspend();
    }

    // clear data
    convergenceStore=0;
    potentialDot=1000;
    potentialLast=0;
    dynamicState = UNKNOWN;

    // clear the output
    Vout.zero();    
    

}

void CB::Controller::stopAction() {
    stop();
    stopResources();    
    started = false;
    cout << "Controller::stop() -- finished" << endl;
}

void CB::Controller::resetController() {

    cout << "Controller::resetController()" << endl;

    // reset the storage data
    potentialDotStore.clear();
    potentialStore.clear();
    
    // stop the controller and the resources if in non-distributed mode
    pauseAction(); 
    stopAction();

    // delete the resources
    deleteResources();

    cout << "Controller::resetController() -- done" << endl;
}

void CB::Controller::stopResources() {
    // stop resources
    if(!distributedMode) {
        sensor->stopResource();
        if(hasReference) {
            reference->stopResource();
        }
        effector->stopResource();
    }

    // stop jacobian (if necessary)
    if(needsJacobian) {
        jacobian->stopJacobian();    
    }

    // stop potential function
    potentialFunction->stopPotentialFunction();
}

void CB::Controller::deleteResources() {
    if(jacobian!=NULL) {
        cout << "Controller::resetController() -- deleting jacobian" << endl;
        delete jacobian; jacobian=NULL;
        cout << "Controller::resetController() -- jacobian deleted" << endl;
    }
    if(potentialFunction!=NULL) {
        cout << "Controller::resetController() -- deleting potentialFunction" << endl;
        delete potentialFunction; potentialFunction=NULL;
        cout << "Controller::resetController() -- potentialFunction deleted" << endl;
    }
}

void CB::Controller::postData() 
{
    
    Bottle &b = outputPorts[0]->prepare();
    b.clear();                            
    b.addInt(Vout.size());
    
    cout << "Vout: (";
    for(int k = 0; k < Vout.size(); k++) {
        cout << Vout[k] << " ";
        
        // add position to output port
        b.addDouble(Vout[k]);
    }
    cout << endl;
    outputPorts[0]->write();      
}


