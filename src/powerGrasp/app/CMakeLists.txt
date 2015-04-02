# Copyright: 2010 RobotCub Consortium
# Authors: Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
set(appname powerGrasp)
icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

icub_app_install(${appname} 
                FILES ${conf}
                DESTINATION conf)

icub_app_install(${appname} 
                FILES ${scripts}
                DESTINATION scripts)

