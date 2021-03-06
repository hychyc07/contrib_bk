FIND_PACKAGE(LEMON)

if (LEMON_FOUND)
	message( STATUS "found LEMON")
	# so LEMON_DIR, LEMON_INCLUDE_DIR, LEMON_LIBRARY are now defined:
	
	set( EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} ${LEMON_LIBRARY} )
	set( EXTERNAL_INCLUDES  ${EXTERNAL_INCLUDES} ${LEMON_INCLUDE_DIR} )
	
else(LEMON_FOUND)

	message( STATUS "LEMON package not found: looking for environment variables")
	
	set(PMP_HAS_LEMON_LIB FALSE)
	if( NOT EXISTS $ENV{LEMON_DIR} )
		message ( FATAL_ERROR "no LEMON ENVIRONMENT VARIABLE found")

	else( NOT EXISTS $ENV{LEMON_DIR} )
		set(PMP_HAS_LEMON_LIB TRUE)
		set (GSL_DIR "$ENV{LEMON_DIR}" CACHE PATH "LEMON_DIR path")
		message (STATUS "found LEMON: $ENV{LEMON_DIR}")

	endif( NOT EXISTS $ENV{LEMON_DIR} )

	if(PMP_HAS_LEMON_LIB)
		set(LEMON_LIB_PATH "$ENV{LEMON_DIR}/lemon/Release" CACHE PATH "LEMON_LIBRARIES path")
		if (NOT IS_DIRECTORY ${LEMON_LIB_PATH})
			message ( FATAL ERROR "SPECIFY LEMON libraries PATH")
		endif (NOT IS_DIRECTORY ${LEMON_LIB_PATH})

		file (GLOB LEMON_LIBRARY ${LEMON_LIB_PATH}/*.lib)
		message (STATUS "found LEMON library: ${LEMON_LIBRARY}")
	
		# find LEMON headers
		set (LEMON_INCLUDE_DIR "$ENV{LEMON_ROOT}/lemon" CACHE PATH "LEMON_INCLUDE_DIR path")
		if (NOT IS_DIRECTORY ${LEMON_INCLUDE_DIR})
			message ( FATAL ERROR "SPECIFY LEMON headers PATH")
		endif (NOT IS_DIRECTORY ${LEMON_INCLUDE_DIR})
	
	endif (PMP_HAS_LEMON_LIB)
	
	set( EXTERNAL_LIBRARIES ${EXTERNAL_LIBRARIES} ${LEMON_LIBRARY} )
	set( EXTERNAL_INCLUDES  ${EXTERNAL_INCLUDES} ${LEMON_INCLUDE_DIR} )
	
endif (LEMON_FOUND)
	