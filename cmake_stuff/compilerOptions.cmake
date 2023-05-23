SET(EXTRA_C_FLAGS                   "")
SET(EXTRA_C_FLAGS_RELEASE           "")   # Already contain "-O3 -DNDEBUG"
SET(EXTRA_C_FLAGS_DEBUG             "")

SET(EXTRA_EXE_LINKER_FLAGS          "")
SET(EXTRA_EXE_LINKER_FLAGS_RELEASE  "")
SET(EXTRA_EXE_LINKER_FLAGS_DEBUG    "")

IF(${USE_CLANG})
   SET(CMAKE_C_COMPILER_ID       "Clang")
   SET(CMAKE_CXX_COMPILER_ID     "Clang")
   SET(CMAKE_C_COMPILER          "/usr/bin/clang")
   SET(CMAKE_CXX_COMPILER        "/usr/bin/clang++")

   SET(CMAKE_C_FLAGS             "-std=c99")
   SET(CMAKE_C_FLAGS_DEBUG       "-g")
   SET(CMAKE_C_FLAGS_RELEASE     "-O2")

   SET(CMAKE_CXX_FLAGS           "")
   SET(CMAKE_CXX_FLAGS_DEBUG     "-g")
   SET(CMAKE_CXX_FLAGS_RELEASE   "-O2")
ENDIF()

IF(${USE_ANALYZE})
   SET(CMAKE_C_COMPILER_ID       "ccc-analyzer")
   SET(CMAKE_CXX_COMPILER_ID     "c++-analyzer")
   SET(CMAKE_C_COMPILER          "/usr/share/clang/scan-build/ccc-analyzer")
   SET(CMAKE_CXX_COMPILER        "/usr/share/clang/scan-build/c++-analyzer")

   SET(CMAKE_C_FLAGS             "-std=c99")
   SET(CMAKE_C_FLAGS_DEBUG       "-g")
   SET(CMAKE_C_FLAGS_RELEASE     "-O2")

   SET(CMAKE_CXX_FLAGS           "")
   SET(CMAKE_CXX_FLAGS_DEBUG     "-g")
   SET(CMAKE_CXX_FLAGS_RELEASE   "-O2")
ENDIF()

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "c++-analyzer")

   # If CMake version is >=3.1, must be used this variable instead of -std=c++17
   SET(CMAKE_CXX_STANDARD  17)

   # When using GCC greater than 7.4, its necessary add this option
   IF(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.4)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-copy")
   ENDIF()

   SET(EXTRA_C_FLAGS       "${EXTRA_C_FLAGS} -Wall")

   IF(${WARNINGS_ANSI_ISO})
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -Wcast-align")
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -pedantic")
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -Wextra")
   ENDIF()

   IF(${WARNINGS_ARE_ERRORS})
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -Werror")
   ENDIF()

   IF(${WARNINGS_EFFCPP})
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -Weffc++")
   ENDIF()

   SET(EXTRA_C_FLAGS_DEBUG "${EXTRA_C_FLAGS_DEBUG} -O0 -DDEBUG -D_DEBUG")

ENDIF()

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")

   ### Check C++11 ###
   EXECUTE_PROCESS(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
   IF(NOT (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7))
      MESSAGE(FATAL_ERROR "${PROJECT_NAME} C++11 support requires g++ 4.7 or greater.")
   ENDIF()

   # Makes all your symbols hidden by default.
   # Not valid for clang because cmake doesn't generate proper export files
   SET(EXTRA_C_FLAGS       "${EXTRA_C_FLAGS} -fvisibility=hidden")

   # Necessary for Qt
   SET(EXTRA_C_FLAGS       "${EXTRA_C_FLAGS} -Wno-long-long")

   # Necessary for Simulator Antonio
   SET(EXTRA_C_FLAGS       "${EXTRA_C_FLAGS} -Wno-maybe-uninitialized")

   IF(NOT ${WARNINGS_ANSI_ISO})
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -Wno-narrowing")
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -Wno-delete-non-virtual-dtor")
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -Wno-unnamed-type-template-args")
   ENDIF()

   IF(${ENABLE_PROFILING})
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -pg -g")
      # Turn off incompatible options
      FOREACH(flags CMAKE_CXX_FLAGS CMAKE_C_FLAGS CMAKE_CXX_FLAGS_RELEASE CMAKE_C_FLAGS_RELEASE
                    CMAKE_CXX_FLAGS_DEBUG CMAKE_C_FLAGS_DEBUG EXTRA_C_FLAGS_RELEASE)
         STRING(REPLACE "-fomit-frame-pointer" "" ${flags} "${${flags}}")
         STRING(REPLACE "-ffunction-sections" "" ${flags} "${${flags}}")
      ENDFOREACH()
   ELSEIF(NOT APPLE AND NOT ANDROID)
      # Remove unreferenced functions: function level linking
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -ffunction-sections")
   ENDIF()

   # Code Coverage
   IF(${ENABLE_COVERAGE} OR ${ENABLE_COVERAGE_XML})
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} --coverage")
   ENDIF()

   IF(NOT ${BUILD_SHARED_LIBS})
      SET(EXTRA_C_FLAGS    "${EXTRA_C_FLAGS} -fPIC")
   ENDIF()

ELSEIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")

   IF(NOT EXISTS ${CMAKE_CXX_COMPILER})
      MESSAGE(FATAL_ERROR "Clang++ not found.")
   ENDIF()

   IF(${ENABLE_COVERAGE} OR ${ENABLE_COVERAGE_XML})
      MESSAGE(FATAL_ERROR "Not use clang for generate code coverage. Use gcc.")
   ENDIF()

ELSEIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "c++-analyzer")

   IF(NOT EXISTS ${CMAKE_CXX_COMPILER})
      MESSAGE(FATAL_ERROR "c++-analyzer not found.")
   ENDIF()

   IF(${ENABLE_COVERAGE} OR ${ENABLE_COVERAGE_XML})
      MESSAGE(FATAL_ERROR "Not use c++-analyzer for generate code coverage. Use gcc.")
   ENDIF()

ENDIF()

INCLUDE(cmake_stuff/dynamic_analyzer_options.cmake REQUIRED)

### Add user supplied extra options (optimization, etc...)
# ==========================================================
SET(EXTRA_C_FLAGS                   "${EXTRA_C_FLAGS}"
   CACHE INTERNAL "Extra compiler options")
SET(EXTRA_C_FLAGS_RELEASE           "${EXTRA_C_FLAGS_RELEASE}"
   CACHE INTERNAL "Extra compiler options for Release build")
SET(EXTRA_C_FLAGS_DEBUG             "${EXTRA_C_FLAGS_DEBUG}"
   CACHE INTERNAL "Extra compiler options for Debug build")
SET(EXTRA_EXE_LINKER_FLAGS          "${EXTRA_EXE_LINKER_FLAGS}"
   CACHE INTERNAL "Extra linker flags")
SET(EXTRA_EXE_LINKER_FLAGS_RELEASE  "${EXTRA_EXE_LINKER_FLAGS_RELEASE}"
   CACHE INTERNAL "Extra linker flags for Release build")
SET(EXTRA_EXE_LINKER_FLAGS_DEBUG    "${EXTRA_EXE_LINKER_FLAGS_DEBUG}"
   CACHE INTERNAL "Extra linker flags for Debug build")

### Combine all "extra" options
SET(CMAKE_C_FLAGS             "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")
SET(CMAKE_C_FLAGS_RELEASE     "${CMAKE_C_FLAGS_RELEASE} ${EXTRA_C_FLAGS_RELEASE}")
SET(CMAKE_C_FLAGS_DEBUG       "${CMAKE_C_FLAGS_DEBUG} ${EXTRA_C_FLAGS_DEBUG}")

SET(CMAKE_CXX_FLAGS           "${CMAKE_CXX_FLAGS} ${EXTRA_C_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELEASE   "${CMAKE_CXX_FLAGS_RELEASE} ${EXTRA_C_FLAGS_RELEASE}")
SET(CMAKE_CXX_FLAGS_DEBUG     "${CMAKE_CXX_FLAGS_DEBUG} ${EXTRA_C_FLAGS_DEBUG}")

SET(CMAKE_EXE_LINKER_FLAGS          "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_EXE_LINKER_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE  "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${EXTRA_EXE_LINKER_FLAGS_RELEASE}")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG    "${CMAKE_EXE_LINKER_FLAGS_DEBUG} ${EXTRA_EXE_LINKER_FLAGS_DEBUG}")
