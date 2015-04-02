set(appname attentionNetworkTest)

## declare new application command
icub_app(${appname})

## add template scripts, build list
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

## install rules
icub_app_install(${appname} 
				FILES ${conf}
				DESTINATION conf)			
				
icub_app_install(${appname} 
				FILES ${scripts}
				DESTINATION scripts)	


