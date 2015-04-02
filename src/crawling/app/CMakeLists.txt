cmake_minimum_required(VERSION 2.6)

project(crawling)

include(iCubHelpers)
# default prefix for applications
SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE PATH "Application path prefix")
				

## make all applications show up as single target
#icub_app_all()
set(appname crawlingApplication)
icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB logconf ${CMAKE_CURRENT_SOURCE_DIR}/conf/logconf/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

icub_app_install(${appname} 
                 FILES ${conf}
                 DESTINATION conf/)

icub_app_install(${appname} 
                 FILES ${logconf}
                 DESTINATION conf/logconf)

icub_app_install(${appname} 
                FILES ${scripts}
                DESTINATION scripts/)

icub_app_install(${appname} 
                FILES ${logscripts}
                DESTINATION scripts/logscripts)

icub_app_all()
