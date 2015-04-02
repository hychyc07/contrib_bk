// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include <stdlib.h>
#include <math.h>
#include <yarp/os/Time.h>

#include "GazeControlPolicyLearner.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace gc;

GazeControlPolicyLearner::GazeControlPolicyLearner(map<int, GazeControlAction *> a, 
                                                   vector< pair<GazeControlAction *, GazeControlAction *> > ap,
                                                   int nEpisodes) :
    actions(a),
    actionPairs(ap),
    numEpisodes(nEpisodes),
    RateThread(10)
{
    init();
}

GazeControlPolicyLearner::GazeControlPolicyLearner(map<int, GazeControlAction *> a, int nEpisodes) :
    actions(a),
    numEpisodes(nEpisodes),
    RateThread(10)
{
    actionPairs.clear();
    init();
}


void GazeControlPolicyLearner::init() {

    iteration = 0;
    episodeID = 0;
    totalReward = 0;
    lastAction = -1;
    stateID = -1;
    lastState = -1;

    timeoutThreshold = 10;

    numPrimitives = actions.size();
    numComposites = actionPairs.size();
    numActions = numPrimitives+numComposites;

    numStates = pow(GazeControlAction::NUM_PREDICATE_VALUES,numPrimitives);
    GazeControlAction gca;
    mdp = new MarkovDecisionProcess(numStates,numActions);
    fName = "gazeControlPolicy.xml";    
    mdp->setValueFile(fName);
    mdp->loadValueFunction();

    string s;
    int np = (int)(GazeControlAction::NUM_PREDICATE_VALUES);
    int tmp;
    for(int n=0; n<numStates; n++) {
        s = "";
        tmp = n;
        for(int k=0; k<numPrimitives; k++) {
            s = gca.getPredicateString((GazeControlAction::PredicateValue)(tmp%np)) + s;
            tmp = tmp/np;
        }
        stateStringMap[n] = s;
        stateIntMap[s] = n;
    }

}

GazeControlPolicyLearner::~GazeControlPolicyLearner() {
    delete mdp;
}

void GazeControlPolicyLearner::run() { 
    
    double reward;
    double avgReward;

    if(episodeID==numEpisodes) {

        cout << "Learning episodes finished..." << endl;
        // reset everything
        action = -1;
        lastAction = -1;
        lastState = -1;
        totalReward = 0;
        iteration = 0;

        // write out the value function
        //mdp->writeValueFunction(avgReward);

        stop();
        return;
    }

    if(iteration==timeoutThreshold) {

        cout << "reached iteration timeout for learning episode..." << endl;
        cout << "total reward: " << totalReward << ", iterations: " << iteration << ", avgReward: " << avgReward << endl;

        // write out the value function
        //mdp->writeValueFunction(avgReward);

        // reset everything
        action = -1;
        lastAction = -1;
        lastState = -1;
        totalReward = 0;
        iteration = 0;
        episodeID++;

        return;
    }

    // update state
    lastState = stateID;

    //    cout << "updating state.." << endl;
    stateID = evaluateState();

    //    cout << "GazeControlPolicyLearner::run() -- in state: " << stateID << endl;

    // check to see if there has been a state transition
    if(stateID != lastState) {

        cout << "GazeControlPolicyLearner::run() -- entered state: " << stateID << endl;

        // update the value function if a valid transition under a valid action has occurred
        if(action != -1) {

            cout << "new state[" << stateID << "] -- <" << stateString << ">" << endl;            

            // get the reward
            reward = getReward(lastState, action, stateID);                                        
            //reward = getReward();                                        
            totalReward += reward;

            // stop the action
            stopAction(action);

            // update the MDP
            mdp->updateRule(lastState, action, reward, stateID, MarkovDecisionProcess::Q_LEARNING);

        } 

        // check if we have reached a goal state
        if(isGoalState(stateID)) {

            cout << "stopping action[" << action << "]" << endl;

            avgReward = totalReward/iteration;
            cout << "reached goal state[" << stateID << "] -- <" << stateString << ">" << endl;            
            cout << "total reward: " << totalReward << ", iterations: " << iteration << ", avgReward: " << avgReward << endl;

            cout << endl << endl;

            // stop the action
            stopAction(action);

            // write out the value function
            //mdp->writeValueFunction(avgReward);

            // reset everything
            action = -1;
            lastAction = -1;
            lastState = -1;
            totalReward = 0;
            iteration = 0;
            stateID = 0;

            episodeID++;

            return;

        }

        // choose best action in the state
        lastAction = action;
        action = mdp->getMaxAction(stateID,1);
            
        // perform the action
        cout << "starting new action[" << action << "]" << endl;
        startAction(action);

        // update the transition (iteration) count
        iteration++;
        
    }

}

void GazeControlPolicyLearner::startAction(int a) {
    int id;
    if(a < numPrimitives) {
        actions[a]->start();
    } else if(a < numActions) {
        id = a - numPrimitives;
        actionPairs[id].first->start();
        actionPairs[id].second->start();
    } else {
        cout << "GazeControlPolicyLearner::startAction("<<a<<") action out of range!!" << endl;
    }
}

void GazeControlPolicyLearner::stopAction(int a) {
    int id;
    if(a < numPrimitives) {
        actions[a]->stopAction();
    } else if(a < numActions) {
        id = a - numPrimitives;
        actionPairs[id].first->stopAction();
        actionPairs[id].second->stopAction();
    } else {
        cout << "GazeControlPolicyLearner::stopAction("<<a<<") action out of range!!" << endl;
    }
}

bool GazeControlPolicyLearner::isGoalState(int s) {

    // get the state string
    string str = stateStringMap[s];

    // make sure hte string and the action vector are the same size
    if(str.size() != actions.size()) 
        return false;

    // go through and see if any rewarding action has converged
    // if so, add the state to the mdp's list of rewarding states
    // and return true;
    for(int k=0; k<actions.size(); k++) {
        if( (str[k] == '1') && (actions[k]->isRewarding()) ) {
            if(!(mdp->isGoalState(s))) {
                mdp->addGoalState(s);
            }
            cout << "found goal state: " << str.c_str() << endl;
            return true;
        }
    }

    return false;
}

int GazeControlPolicyLearner::evaluateState() {
    stateString = "";
    string sstr;
    //    cout << "evaluating " << actions.size() << " actions" << endl;
    for(int i=0; i<actions.size(); i++) {
        actions[i]->evaluateState();
        sstr = actions[i]->getStateString();
        //cout << "p["<<i<<"]: " << sstr.c_str() << ", pdot: " << actions[i]->getPotentialDot() << endl;
        stateString += sstr;
    }    
    int s = stateIntMap[stateString]; 
    //    cout << "state[" << s << "] -- <" << stateString.c_str() << ">" << endl;
    return s;
}

double GazeControlPolicyLearner::getReward(int sp, int a, int s) {
    int id;
    double r = 0;
    double t;
    if(a < numPrimitives) {

        cout << "action[" << a << "]: rewarding: " << actions[a]->isRewarding() << endl;
        if(actions[a]->isRewarding() && (actions[a]->getState()==GazeControlAction::CONVERGED)) {
            t = (1.0 - actions[a]->getCost());
            cout << "reward for action[" << a << "]: " << t << endl;
            r += t;
        }

    } else if(a < numActions) {
        id = a - numPrimitives;

        if(actionPairs[id].first->isRewarding() && (actionPairs[id].first->getState()==GazeControlAction::CONVERGED)) {
            r += (1.0 - actionPairs[id].first->getCost());
        }

        if(actionPairs[id].second->isRewarding() && (actionPairs[id].second->getState()==GazeControlAction::CONVERGED)) {
            r += (1.0 - actionPairs[id].second->getCost());
        }

    } else {
        cout << "GazeControlPolicyLearner::getReward("<<a<<") action out of range!!" << endl;
    }

    return r;
}


double GazeControlPolicyLearner::getReward() {

    double r = 0;
    double t;

    for(int i=0; i<actions.size(); i++) {

        if(actions[i]->isRewarding() && (actions[i]->getState()==GazeControlAction::CONVERGED)) {
            t = 1.0;//(1.0 - actions[i]->getCost());
            cout << "reward for action[" << i << "]: " << t << endl;
            r += t;
        }

    }

    return r;
}
