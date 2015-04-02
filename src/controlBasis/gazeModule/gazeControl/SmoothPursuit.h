// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _SMOOTH_PURSUIT__H_
#define _SMOOTH_PURSUIT__H_

#include "GazeControlAction.h"

namespace gc {

    class SmoothPursuit : public GazeControlAction {
    
    protected:
  
    public:
        
        SmoothPursuit(CB::YARPBlobTrackerHeading *b, CB::YARPConfigurationVariables *d) :
            GazeControlAction(b,d) 
        {
            isRewardingAction = true;
            composable = true;
            setConvergenceConditions(0.0, 1e-10);
        }

        ~SmoothPursuit() { }
    
        void run();
        void setVelocities();
        void evaluateCost();

    };

}

#endif
