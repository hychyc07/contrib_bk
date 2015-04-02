cmake_minimum_required(VERSION 2.6)

project(iFumble)

find_package(ICUB)
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

include(iCubHelpers)

set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE TYPE PATH)
				
set(appname iFumbleApplication)
icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

icub_app_install(${appname} 
                 FILES ${conf}
                 DESTINATION conf/)

icub_app_install(${appname} 
                 FILES ${scripts}
                 DESTINATION scripts/)

icub_app_all()


