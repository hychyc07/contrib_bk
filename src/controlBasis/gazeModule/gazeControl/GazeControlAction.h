// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _GAZE_CONTROL_ACTION__H_
#define _GAZE_CONTROL_ACTION__H_

#include <iostream>
#include <string>
#include <map>
#include <math.h>

#include <yarp/os/RateThread.h>
#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>

#include <controlBasis/YARPBlobTrackerHeading.h>
#include <controlBasis/YARPConfigurationVariables.h>

namespace gc {
    
    class GazeControlAction : public yarp::os::RateThread {   

    public:
        
        enum PredicateValue {
            UNDEFINED=0,
            UNCONVERGED,
            CONVERGED,
            NUM_PREDICATE_VALUES
        };
        
        std::map<PredicateValue, std::string> predicateMap;

    protected:

        bool composable;
        bool isRewardingAction;

        double potential;
        double potentialDot;
        std::vector<double> potentialStore;

        double potentialThreshold;
        double potentialDotThreshold;

        CB::YARPBlobTrackerHeading *blobTracker;
        CB::YARPConfigurationVariables *device;

        PredicateValue statePredicate;

        yarp::sig::Vector blobData;

        double cost;
        double dW, dT;
        double t0, t1, dt;
        double mass;
        yarp::sig::Vector VpLast;
        yarp::sig::Vector VdLast;

        int id;

    public:
        
        GazeControlAction(CB::YARPBlobTrackerHeading *b=NULL, CB::YARPConfigurationVariables *d=NULL, int period=10) :
            RateThread(period),
            blobTracker(b),
            potentialDot(0.0),
            device(d),
            potentialThreshold(0),
            potentialDotThreshold(1e-6),
            VpLast(6),
            VdLast(6)
        {
            predicateMap[UNDEFINED]   = "-";
            predicateMap[UNCONVERGED] = "0";
            predicateMap[CONVERGED]   = "1";            
        }

        ~GazeControlAction() { }
        
        virtual void run() { }
        virtual void setVelocities() {}
        virtual void evaluateCost() {}

        int getID() { return id; }
        void setID(int i) { id=i; }

        double getCost() { 
            double tmp  = dT/10;
            std::cout << "asking for cost for action: (dW = " << dW << ", dT = " << tmp << ")" << std::endl;
            if(tmp > 1) tmp= 1;
            cost = tmp;
            //            if(cost > 1) cost = 1;
            //if(cost < 0) cost = 0;
            return cost; 
        }

        void beforeStart() {
            cost = 0;
            potentialStore.clear();
            potential = 0;
            dT = dW = 0;
            t0=yarp::os::Time::now();
            t1 = t0;
        }

        bool isComposable() {
            return composable;
        }

        void stopAction() {
            potentialStore.clear();
            potential = 0;
            stop();
            setVelocities();
            dT = dW = 0;
        }
        void evaluateState() {

            statePredicate = UNCONVERGED;

            if(blobTracker==NULL) {
                std::cout << "Action::run() -- BlobTracker not set..." << std::endl;
                statePredicate = UNDEFINED;
                return;
            }

            if(device==NULL) {
                std::cout << "Action::run() -- Device not set..." << std::endl;
                statePredicate = UNDEFINED;
                return;
            }
            
            if(!blobTracker->isValid()) {
                statePredicate = UNDEFINED;
                return;
            }
            
            blobData = blobTracker->getResourceData();

            if(blobData.size()!=2) {
                std::cout << "Action::run() blob data wrong size -- " << blobData.size() << std::endl;
                statePredicate = UNDEFINED;
                return;
            }

            if(!isRunning()) {
                statePredicate = UNCONVERGED;
                return;
            }

            // can calculate admissibility here too....
            //yarp::os::Time::delay(0.1);
           
            // calculate potential
            potential = calculatePotential(blobData);
            potentialDot = updatePotentialDotEstimate();

        }

        PredicateValue getState() { 
            //evaluateState();
            return statePredicate; 
        }

        std::string getStateString() { 
            //evaluateState();
            return getPredicateString(statePredicate); 
        }
        
        bool isRewarding() { return isRewardingAction; }
        
        std::string getPredicateString(PredicateValue v) {
            return predicateMap[v];
        }       
        
        double calculatePotential(const yarp::sig::Vector v) {
            double pot = sqrt(yarp::math::dot(v,v));
            potentialStore.push_back(pot);
            if(potentialStore.size() > 1000) {
                potentialStore.erase(potentialStore.begin());
            }
            return pot;
        }

        double getPotentialDot() { return potentialDot; }

        void setConvergenceConditions(double pt, double pdt) {
            potentialThreshold = pt;
            potentialDotThreshold = pdt;
        }
        
        double updatePotentialDotEstimate() {

            double pdiff, p0;
            double lag = 10;
            double alpha = 0.2;
            double T = (1.0/getEstPeriod())*lag;

            if(potentialStore.size() < lag) {
                p0 = 10;
                pdiff = (potential - p0)/T;
                potentialDot = alpha*pdiff + (1.0-alpha)*potentialDot; 
                statePredicate = UNCONVERGED;
            } else {
                p0 = potentialStore[potentialStore.size() - lag];
                pdiff = (potential - p0)/T;
                potentialDot = alpha*pdiff + (1.0-alpha)*potentialDot; 
                if(fabs(potentialDot) < potentialDotThreshold) {
                    statePredicate = CONVERGED;
                }            
            }

            if(potential < potentialThreshold) {       
                statePredicate = CONVERGED;
            }

            /*
            std::cout << "pdiff: " << pdiff << std::endl;
            std::cout << "p0: " << p0 << std::endl;
            std::cout << "|p|: " << potentialStore.size() << std::endl;
            std::cout << "p: " << potential << std::endl;
            */

            //std::cout << "action[" << id << "]: pot = (" << potential << "," << potentialDot << "), T = " << T << ", STATE: " << getStateString().c_str() << std::endl;
            return potentialDot;
        }

    };

}

#endif
