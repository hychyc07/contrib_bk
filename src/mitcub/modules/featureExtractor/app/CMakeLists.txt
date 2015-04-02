
set(appname featureExtractor)
icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB extractors ${CMAKE_CURRENT_SOURCE_DIR}/extractors/*)
file(GLOB dictionarizers ${CMAKE_CURRENT_SOURCE_DIR}/dictionarizers/*)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

icub_app_install(${appname} 
                 FILES ${conf}
                 DESTINATION conf/)

icub_app_install(${appname} 
                 FILES ${extractors}
                 DESTINATION conf/extractors)
				 
icub_app_install(${appname} 
                 FILES ${dictionarizers}
                 DESTINATION conf/dictionarizers)

icub_app_install(${appname} 
                FILES ${scripts}
                DESTINATION scripts/)
