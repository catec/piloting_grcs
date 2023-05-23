FIND_PROGRAM(GCOVR_PATH gcovr)

# Param _targetname     The name of new the custom make target
# Param _testrunner     The name of the target which runs the tests
# Param _outputname     cobertura output is generated as _outputname.xml
# Optional fourth parameter is passed as arguments to _testrunner
#   Pass them in list form, e.g.: "-j;2" for -j 2
FUNCTION(SETUP_TARGET_FOR_COVERAGE_XML _targetname _testrunner _outputname)

   IF(NOT GCOVR_PATH)
      MESSAGE(FATAL_ERROR "gcovr not found! Aborting...")
   ENDIF() # NOT GCOVR_PATH

	ADD_CUSTOM_TARGET(${_targetname}

		# Run tests
        COMMAND ${_testrunner} ${ARGV3} ${ARGV4} ${ARGV5} ${ARGV6}

		# Running gcovr
      COMMAND ${GCOVR_PATH} -x -r ".." -e "tests" -e "build*" -o ${_outputname}.xml
        #COMMAND ${GCOVR_PATH} -x -r ${CMAKE_SOURCE_DIR} -e '${CMAKE_SOURCE_DIR}/scripts/'  -o ${_outputname}.xml
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
		COMMENT "Running gcovr to produce code coverage report."
	)

	# Show info where to find the report
	ADD_CUSTOM_COMMAND(TARGET ${_targetname} POST_BUILD
		COMMAND ;
		COMMENT "code coverage report saved in ${_outputname}.xml."
	)

ENDFUNCTION()
