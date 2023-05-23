CMAKE_MINIMUM_REQUIRED(VERSION 3.5.1)
PROJECT(piloting_grcs)

IF(CMAKE_MAJOR_VERSION VERSION_GREATER 2)
   CMAKE_POLICY(SET CMP0054 OLD)
   CMAKE_POLICY(SET CMP0020 OLD)
ENDIF()

INCLUDE(cmake_stuff/versions.cmake           REQUIRED)
INCLUDE(cmake_stuff/options.cmake            REQUIRED)
INCLUDE(cmake_stuff/compilerOptions.cmake    REQUIRED)
INCLUDE(cmake_stuff/findDependencies.cmake   REQUIRED)

ADD_SUBDIRECTORY(data)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(utils)

INCLUDE(cmake_stuff/buildFiles.cmake         REQUIRED)
INCLUDE(cmake_stuff/packageGenerator.cmake   REQUIRED)
INCLUDE(cmake_stuff/printInfo.cmake          REQUIRED)
