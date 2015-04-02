// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
 * This class implementes a 'model' Control Basis Schema whose state is 
 * evaluated from the models of the run-time dynamics of relevent control actions.
 *
 * !!!BEWARE!!!! This is work in progress (and not all the pieces are implemented yet...)
 *
 * Author: Stephen Hart
 **/
#ifndef _MODEL_SCHEMA__H_
#define _MODEL_SCHEMA__H_

#include "Schema.h"

namespace CB {
    
    class ModelSchema : public Schema {

    public:
    
        /**
         * Constructor
         **/
        ModelSchema(std::vector<ControllerParameters> controllerParams, std::vector<std::string> subSchema, int limit) {   
        }
        
        /**
         * Destructor
         **/
        ~ModelSchema() { }

        virtual double evaluateReward(int state, int last_state, int action, int last_action)=0;
        virtual int evaluateState()=0;
        virtual std::string getStringFromState(int s)=0;
        virtual int getStateFromString(std::string str)=0;     

  };

}


#endif
