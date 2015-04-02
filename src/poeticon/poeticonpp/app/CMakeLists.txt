# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

icub_app(${PROJECTNAME})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.ini ${CMAKE_CURRENT_SOURCE_DIR}/scripts/modules/*.xml ${CMAKE_CURRENT_SOURCE_DIR}/scripts/modules/*.xml)
file(GLOB conf    ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
				 
icub_app_install(${PROJECTNAME}
                 FILES ${scripts}
                 DESTINATION scripts)

icub_app_install(${PROJECTNAME}
                 FILES ${conf}
                 DESTINATION conf)


### create a single target that installs all applications at once
icub_app_all()
