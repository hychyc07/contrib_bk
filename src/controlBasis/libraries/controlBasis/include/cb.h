// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
 * \defgroup icub_controlbasis_libraries Control Basis Libraries
 * @ingroup icub_controlbasis_libraries
 *
 */
#ifndef _CB__H_
#define _CB__H_

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

#ifndef TODEG
#define TODEG 180.0/M_PI
#endif

#ifndef TORAD
#define TORAD M_PI/180.0
#endif

namespace CB {
    
    /**                                                                                                                                                                          
     * DH Parameter names                                                                                                                                                        
     **/
    enum ParamType
        {
            DH_ALPHA = 0,
            DH_A,
            DH_D,
            DH_THETA,
        };
    
    /**                                                                                                                                                                          
     * the DH Parameter link types                                                                                                                                               
     **/
    enum LinkType
        {
            LINK_TYPE_CONSTANT = 0,
            LINK_TYPE_PRISMATIC,
            LINK_TYPE_REVOLUTE,
            LINK_TYPE_NONINTERFERING,
        };
    
}

#endif
