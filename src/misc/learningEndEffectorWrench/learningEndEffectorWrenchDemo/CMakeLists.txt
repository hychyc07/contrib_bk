set(appname learningEndEffectorWrench)

icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.machine ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.transf)

icub_app_install(${appname} 
				FILES ${scripts}
				DESTINATION scripts)

icub_app_install(${appname} 
				FILES ${conf}
				DESTINATION conf)

