#------------------------------------------------------
#   Build type
#------------------------------------------------------
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)
IF(DEFINED CMAKE_BUILD_TYPE)
   SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
ENDIF()

IF(NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE "Debug")
ENDIF()

IF(NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug" AND
   NOT ${CMAKE_BUILD_TYPE} STREQUAL "Release")
   MESSAGE(FATAL_ERROR "Only Release and Debug build types are allowed.")
ENDIF()

# ----------------------------------------------------------------------------
#   PROJECT CONFIGURATION
#   force some variables that could be defined in the command line to be written to cache
# ----------------------------------------------------------------------------
OPTION(USE_CLANG           "Use Clang compiler"                                              OFF)
OPTION(USE_ANALYZE         "Use Clang compiler with analyze mode"                            OFF)
OPTION(ANALYZE_MEMORY      "Clang dynamic analyzer: is a detector of uninitialized reads."   OFF)
OPTION(ANALYZE_ADDRESS     "Clang dynamic analyzer: is a fast memory error detector. "       OFF)
OPTION(ANALYZE_THREAD      "Clang dynamic analyzer: is a tool that detects data races. "     OFF)
OPTION(ANALYZE_UNDEFINED   "Clang dynamic analyzer: undefined behavior checker. "            OFF)
OPTION(ANALYZE_DATAFLOW    "Clang dynamic analyzer: is a general dynamic dataflow analysis." OFF)

OPTION(WARNINGS_ARE_ERRORS "Treat warnings as errors"                                        ON)
OPTION(WARNINGS_ANSI_ISO   "Issue all the mandatory diagnostics Listed in C standard"        ON)
OPTION(WARNINGS_EFFCPP     "Issue all the warnings listed in the book of Scot Meyers"        OFF)

OPTION(ENABLE_PROFILING    "Enable profiling in Valgrind (Add flags: -g -fno_inline)"        OFF)

OPTION(ENABLE_COVERAGE     "Perform code coverage in HTML"                                   OFF)
OPTION(ENABLE_COVERAGE_XML "Perform code coverage in XML for jenkins integration"            OFF)

OPTION(BUILD_SHARED_LIBS   "Build shared libraries"                                          ON)
OPTION(BUILD_TESTS         "Build unitary & integration tests with gtest"                    OFF)
OPTION(BUILD_UTILS         "Build applications using the different modules"                  ON)

OPTION(DISABLE_LOG         "Logging code is replaced with a no-op"                           OFF)
OPTION(LINE_NUMBERS_LOG    "Automatically writes the file and line for each log message"     OFF)

# ----------------------------------------------------------------------------
#   Code coverage
# ----------------------------------------------------------------------------
IF(${ENABLE_COVERAGE})
   INCLUDE(cmake_stuff/code_coverage.cmake      REQUIRED)
ENDIF()

IF(${ENABLE_COVERAGE_XML})
   INCLUDE(cmake_stuff/code_coverage_xml.cmake  REQUIRED)
ENDIF()

IF(${ENABLE_COVERAGE} AND NOT ${BUILD_TESTS})
   MESSAGE(WARNING "It's necessary to build unitary tests for checking code coverage.")
ENDIF()

IF(${ENABLE_COVERAGE} OR ${ENABLE_COVERAGE_XML})
   ADD_CUSTOM_TARGET(cleanCoverage
      COMMAND find -name *.gcda -delete
      COMMAND find -name *.gcno -delete
      COMMAND rm -rf coverage*
      COMMAND rm -f *info*
      COMMAND make clean)
ENDIF()

SET(CMAKE_INCLUDE_DIRS_CONFIGCMAKE     ${CMAKE_INSTALL_PREFIX}/include
   CACHE PATH "Output directory for headers")
SET(CMAKE_LIB_DIRS_CONFIGCMAKE         ${CMAKE_INSTALL_PREFIX}/lib
   CACHE PATH "Output directory for libraries")

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY     ${PROJECT_BINARY_DIR}/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY     ${PROJECT_BINARY_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY     ${PROJECT_BINARY_DIR}/lib)

SET(CMAKE_INSTALL_RPATH                ${CMAKE_INSTALL_PREFIX}/lib)
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH  TRUE)

INCLUDE(GenerateExportHeader)
