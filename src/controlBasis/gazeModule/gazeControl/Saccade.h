// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _SACCADE__H_
#define _SACCADE__H_

#include "GazeControlAction.h"

namespace gc {

    class Saccade : public GazeControlAction {
    
    protected:

    public:
    
        Saccade(CB::YARPBlobTrackerHeading *b, CB::YARPConfigurationVariables *d) :
            GazeControlAction(b,d) 
        {
            isRewardingAction = false;
            composable = false;
            setConvergenceConditions(0.03, 0);
        }

        ~Saccade() { }
        
        void run();
        void setVelocities();
        void evaluateCost();

    };

}

#endif
