# Copyright: 2010 RobotCub Consortium
# Authors: Sean Ryan Fanello, Ilaria Gori
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
set(appname allGesturesYouCan2.0)
icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB rad     ${CMAKE_CURRENT_SOURCE_DIR}/RAD/*.*)

icub_app_install(${appname} 
                FILES ${conf}
                DESTINATION conf)

icub_app_install(${appname} 
                FILES ${scripts}
                DESTINATION scripts)
				
icub_app_install(${appname} 
                FILES ${rad}
                DESTINATION RAD)
