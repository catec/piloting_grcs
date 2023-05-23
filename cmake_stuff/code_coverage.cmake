#
# 2012-01-31, Lars Bilke
# - Enable Code Coverage
#
# 2013-09-17, Joakim Söderberg
# - Added support for Clang.
# - Some additional usage instructions.
#
# Do not forget to use the flags -fprofile-arcs -ftest-coverage (or --coverage)

FIND_PROGRAM(GCOV_PATH     gcov)
FIND_PROGRAM(LCOV_PATH     lcov)
FIND_PROGRAM(GENHTML_PATH  genhtml)

IF(NOT GCOV_PATH)
   MESSAGE(FATAL_ERROR "gcov not found! Aborting...")
ENDIF()  # NOT GCOV_PATH

IF(NOT CMAKE_COMPILER_IS_GNUCXX)
   # Clang version 3.0.0 and greater now supports gcov as well.
   MESSAGE(WARNING "Compiler is not GNU gcc! Clang Version 3.0.0 and greater supports gcov as well, but older versions don't.")

   IF(NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
      MESSAGE(FATAL_ERROR "Compiler is not GNU gcc! Aborting...")
   ENDIF()
ENDIF()

IF(NOT CMAKE_BUILD_TYPE STREQUAL "Debug" )
   MESSAGE( WARNING "Code coverage results with an optimized (non-Debug) build may be misleading" )
ENDIF()

# Param _targetname  The name of new the custom make target
# Param _testapp     The name of the target which runs the tests.
#     MUST return ZERO always, even on errors.
#     If not, no coverage report will be created!
# Param _outputname  lcov output is generated as _outputname.info
#     HTML report is generated in _outputname/index.html
# Optional fourth parameter is passed as arguments to _testapp
#     Pass them in list form, e.g.: "-j;2" for -j 2
FUNCTION(SETUP_TARGET_FOR_COVERAGE _targetname _testapp _outputname)

   SET(NEED_LCOV_VERSION 1.10)
   IF(NOT LCOV_PATH)
      MESSAGE(FATAL_ERROR "lcov not found! Aborting...")
   ELSE()
      EXEC_PROGRAM(lcov ARGS --version OUTPUT_VARIABLE LCOV_V)
      STRING(REGEX REPLACE "lcov: LCOV version ([0-9].[0-9])" "\\1" LCOV_VERSION "${LCOV_V}")
      IF(${LCOV_VERSION} VERSION_LESS NEED_LCOV_VERSION)
         MESSAGE(FATAL_ERROR "GCOV VERSION MINOR THAN ${NEED_LCOV_VERSION}")
      ENDIF()
   ENDIF()

   IF(NOT GENHTML_PATH)
      MESSAGE(FATAL_ERROR "genhtml not found! Aborting...")
   ENDIF()  # NOT GENHTML_PATH
   MESSAGE(STATUS "Coverage arguments: ${ARGV3} ${ARGV4}")

   # Setup target
   ADD_CUSTOM_TARGET(${_targetname}
      # Remove previous files if they exist
      COMMAND ${CMAKE_COMMAND} -E remove base.info total.info total.info.cleaned

      # Coverage data is accumulated during subsequent application runs. Reset the counters
      COMMAND ${LCOV_PATH} --zerocounters --directory ${PROJECT_BINARY_DIR}

      # Capture initial zero coverage data
      COMMAND ${LCOV_PATH} --capture --initial --directory ${PROJECT_BINARY_DIR} -o base.info

      # Perform tests
      COMMAND ${_testapp} ${ARGV3} ${ARGV4}

      # Create test coverage data file
      COMMAND ${LCOV_PATH} --capture --directory ${PROJECT_BINARY_DIR} -o tests.info

      # Combine base and tests coverage data
      COMMAND ${LCOV_PATH} -a base.info -a tests.info -o total.info

      # Removing useless coverage data
      COMMAND ${LCOV_PATH} --remove total.info '${PROJECT_BINARY_DIR}/*' '/opt/*' '/usr/*' 'tests/*' 'utils/*' -o total.info.cleaned

      # Generate report
      COMMAND ${GENHTML_PATH} -o ${_outputname} -s --highlight --legend --demangle-cpp total.info.cleaned

      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMENT "Processing code coverage counters and generating report."
   )

   # Show info where to find the report
   ADD_CUSTOM_COMMAND(TARGET ${_targetname} POST_BUILD
      COMMAND ;
      COMMENT "Open ./${_outputname}/index.html in your browser to view the coverage report."
   )

ENDFUNCTION()  # SETUP_TARGET_FOR_COVERAGE
