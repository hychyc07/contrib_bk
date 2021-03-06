###### This file is automatically generated by CMake.
## see macro icub_app defined in conf/iCubHelpers.cmake

execute_process(COMMAND "${CMAKE_COMMAND}" -E make_directory "")
execute_process(COMMAND "${CMAKE_COMMAND}" -E make_directory "/app")
execute_process(COMMAND "${CMAKE_COMMAND}" -E make_directory "/app/protoObjectVisualAttention/conf")

set(files "/usr/local/robot/iCub/contrib/src/logpolarAttention/app/protoObjectVisualAttention/conf/imageProcessor.ini;/usr/local/robot/iCub/contrib/src/logpolarAttention/app/protoObjectVisualAttention/conf/saliencyBlobFinder.ini;/usr/local/robot/iCub/contrib/src/logpolarAttention/app/protoObjectVisualAttention/conf/icubEyes.ini;/usr/local/robot/iCub/contrib/src/logpolarAttention/app/protoObjectVisualAttention/conf/iKinHead.ini;/usr/local/robot/iCub/contrib/src/logpolarAttention/app/protoObjectVisualAttention/conf/controlGaze2.ini;/usr/local/robot/iCub/contrib/src/logpolarAttention/app/protoObjectVisualAttention/conf/colourProcessor.ini;/usr/local/robot/iCub/contrib/src/logpolarAttention/app/protoObjectVisualAttention/conf/protoObjectVisualAttention.ini")
foreach(f ${files})
	get_filename_component(fname "${f}" NAME)
	if (NOT IS_DIRECTORY "${f}")
		if (NOT EXISTS "/app/protoObjectVisualAttention/conf/${fname}")
			execute_process(COMMAND "${CMAKE_COMMAND}" -E copy "${f}" "/app/protoObjectVisualAttention/conf")
		else()
			execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files "${f}" "/app/protoObjectVisualAttention/conf/${fname}" RESULT_VARIABLE test_not_successful OUTPUT_QUIET ERROR_QUIET)
			if ( test_not_successful )
				message("Preserving locally modified file: /app/protoObjectVisualAttention/conf/${fname}")
				message("--> saving new file to: /app/protoObjectVisualAttention/conf/${fname}.new")
				execute_process(COMMAND "${CMAKE_COMMAND}" -E copy "${f}" "/app/protoObjectVisualAttention/conf/${fname}.new")
			endif()
		endif()
	endif()
endforeach(f "${files}")

execute_process(COMMAND "${CMAKE_COMMAND}" -E make_directory "/app/protoObjectVisualAttention/scripts")

set(files "/usr/local/robot/iCub/contrib/src/logpolarAttention/app/protoObjectVisualAttention/scripts/appConfig-GazeController.xml.template;/usr/local/robot/iCub/contrib/src/logpolarAttention/app/protoObjectVisualAttention/scripts/appConfig-visualAtt.xml.template;/usr/local/robot/iCub/contrib/src/logpolarAttention/app/protoObjectVisualAttention/scripts/appConfig-GazeIKIn.xml.template")
foreach(f ${files})
	get_filename_component(fname "${f}" NAME)
	if (NOT IS_DIRECTORY "${f}")
		if (NOT EXISTS "/app/protoObjectVisualAttention/scripts/${fname}")
			execute_process(COMMAND "${CMAKE_COMMAND}" -E copy "${f}" "/app/protoObjectVisualAttention/scripts")
		else()
			execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files "${f}" "/app/protoObjectVisualAttention/scripts/${fname}" RESULT_VARIABLE test_not_successful OUTPUT_QUIET ERROR_QUIET)
			if ( test_not_successful )
				message("Preserving locally modified file: /app/protoObjectVisualAttention/scripts/${fname}")
				message("--> saving new file to: /app/protoObjectVisualAttention/scripts/${fname}.new")
				execute_process(COMMAND "${CMAKE_COMMAND}" -E copy "${f}" "/app/protoObjectVisualAttention/scripts/${fname}.new")
			endif()
		endif()
	endif()
endforeach(f "${files}")

