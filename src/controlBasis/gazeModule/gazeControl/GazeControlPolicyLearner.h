// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _GAZE_CONTROL_POLICY_LEARNER__H_
#define _GAZE_CONTROL_POLICY_LEARNER__H_

#include <iostream>
#include <vector>
#include <map>

#include <yarp/os/RateThread.h>

#include <MarkovDecisionProcess.h>

#include "GazeControlAction.h"

namespace gc {

    class GazeControlPolicyLearner : public yarp::os::RateThread {
    
    protected:

        std::map<int, GazeControlAction *> actions;
        std::vector< std::pair<GazeControlAction *, GazeControlAction *> > actionPairs;

        MarkovDecisionProcess * mdp;
    
        int numStates;
        int numActions;
        int numEpisodes;

        int numPrimitives;
        int numComposites;

        int iteration;
        int episodeID;
        int timeoutThreshold;

        int lastState;
        int lastAction;
       
        double totalReward;
        int action;
        int stateID;
        std::string stateString;
        
        std::map<int, std::string> stateStringMap;
        std::map<std::string, int> stateIntMap;

        std::string fName;
        
        int evaluateState();
        bool isGoalState(int s);
        double getReward(int sp, int a, int s);
        double getReward();

    public:
        GazeControlPolicyLearner(std::map<int, GazeControlAction *> a, 
                                 std::vector< std::pair<GazeControlAction *, GazeControlAction *> > ap,
                                 int nEpisodes=10);
        GazeControlPolicyLearner(std::map<int, GazeControlAction *> a, 
                                 int nEpisodes=10);

        ~GazeControlPolicyLearner();
    
        // inherited function from RateThread
        void run();

        void startLearning() {

            // reset the state info
            episodeID = 0;
            iteration = 0;
            totalReward = 0;
            action = -1;
            stateID = 0;

            std::cout << "GazeControlPolicyLearner::startLearning()" << std::endl;

            // start the thread
            start();

        }

        void stopLearning() {

            // stop the thread
            if(isRunning()) stop();

            // reset the state info
            iteration = 0;
            episodeID = 0;
            totalReward = 0;
            lastAction = -1;
            stateID = 0;
            lastState = -1;

            for(int i=0; i<actions.size(); i++) {
                actions[i]->stop();
            }

            // write out the value function
            //            mdp->writeValueFunction();

        }

        void startAction(int a);
        void stopAction(int a);

        void setEpisodes(int n) { numEpisodes = n; }
        
    protected:
        void init();

    };

}

#endif
