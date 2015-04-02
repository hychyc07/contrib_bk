set(appname portScope)

project(${appname})

include(iCubHelpers)
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE TYPE PATH)
				
#icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB conf_prediction ${CMAKE_CURRENT_SOURCE_DIR}/conf/predictionStimulus/*.ini)
file(GLOB conf_imitation ${CMAKE_CURRENT_SOURCE_DIR}/conf/imitationStimulus/*.ini)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

icub_app_install(${appname} 
                 FILES ${conf}
                 DESTINATION conf/)

icub_app_install(${appname} 
                 FILES ${conf_prediction}
                 DESTINATION conf/predictionStimulus/)

icub_app_install(${appname} 
                 FILES ${conf_imitation}
                 DESTINATION conf/imitationStimulus/)

icub_app_install(${appname} 
                 FILES ${scripts}
                 DESTINATION scripts/)

icub_app_all()



