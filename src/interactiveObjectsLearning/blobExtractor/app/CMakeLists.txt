# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Carlo Ciliberto
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname ${PROJECTNAME})
icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

icub_app_install(${appname} 
                 FILES ${conf}
                 DESTINATION conf/)
