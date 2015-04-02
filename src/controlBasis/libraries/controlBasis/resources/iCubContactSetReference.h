// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _ICUB_CONTACT_SET_REFERENCE__H_
#define _ICUB_CONTACT_SET_REFERENCE__H_

#include "ContactSet.h"

namespace CB {

    /**
     * this class impliments a ContactSet resource to provide references values to the iCub.
     **/    
    class iCubContactSetReference : public ContactSet {
        
    public:
       
        /** 
         * constructor
         **/
        iCubContactSetReference(std::string name, int dofs=6);        

        /** 
         * destructor
         **/
        ~iCubContactSetReference() { }

        /** 
         * implements update for the parent class (does nothing here)
         **/
        bool updateResource() { return true; }
        
        /** 
         * starts the resource
         **/
        void startResource() { start(); }

        /** 
         * stops the resource
         **/
        void stopResource() { stop(); }
        
        /** 
         * sets the value of the referene from the input
         * \param ref the reference vector
         **/
        void setVals(yarp::sig::Vector ref); 

    };
    
}

#endif
