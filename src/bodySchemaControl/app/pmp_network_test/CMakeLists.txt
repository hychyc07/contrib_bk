# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Dalia De Santis, Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname pmp_network_test)
icub_app(${appname})

file(GLOB conf    ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

icub_app_install(${appname} 
                 FILES ${conf}
                 DESTINATION conf)

icub_app_install(${appname} 
                FILES ${scripts}
                DESTINATION scripts)


