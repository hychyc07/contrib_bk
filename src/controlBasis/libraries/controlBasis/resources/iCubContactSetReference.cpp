// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#include "iCubContactSetReference.h"

CB::iCubContactSetReference::iCubContactSetReference(std::string name, int dofs) {
    
    // set ContactSet info
    deviceName = name + "/ref";
    size = dofs;
    values.resize(size); 

    for(int i=0; i<values.size(); i++) 
        values[i] = 1;  
    
    std::cout << "Creating new iCubContactSetReference(name=" << deviceName.c_str() << ",dof=" << dofs << ")" << std::endl;
    
}

void CB::iCubContactSetReference::setVals(yarp::sig::Vector ref) {
    if(ref.size() != values.size()) {
        std::cout << "Couldn't set reference values for " << deviceName.c_str() << "!!" << std::endl;
        return;
    } else {
        values = ref;          
    }
}
