IF(NOT DEFINED CMAKE_PREFIX_PATH)
   IF(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
      IF(NOT WIN32)
         STRING(REPLACE ":" ";" CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      ELSE()
         SET(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      ENDIF()
   ENDIF()
ENDIF()

## Find includes in corresponding build directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake/)

#######################
## Configuring QsLog ##
#######################

FIND_PACKAGE(QsLog   2.1.1 REQUIRED)

IF(${DISABLE_LOG})
   ADD_DEFINITIONS(-DQS_LOG_DISABLE)
ENDIF()

IF(${LINE_NUMBERS_LOG})
   ADD_DEFINITIONS(-DQS_LOG_LINE_NUMBERS)
ENDIF()

#####################
## Configuring Qt5 ##
#####################

IF(DEFINED ENV{QTDIR})
   IF(NOT $ENV{QTDIR} IN_LIST CMAKE_PREFIX_PATH)
      SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{QTDIR})
   ENDIF()
ENDIF()
IF(CMAKE_PREFIX_PATH STREQUAL "")
   MESSAGE(FATAL_ERROR "CMAKE_PREFIX_PATH not defined. Define the path where Qt5 is.")
ENDIF()

SET(CMAKE_AUTOMOC ON)

FIND_PACKAGE(Qt5Core       5.9.5 REQUIRED)
FIND_PACKAGE(Qt5Widgets    5.9.5 REQUIRED)
FIND_PACKAGE(Qt5Network    5.9.5 REQUIRED)
FIND_PACKAGE(Qt5Concurrent 5.9.5 REQUIRED)

IF(${BUILD_SHARED_LIBS})
   ADD_DEFINITIONS(-DQT_SHARED)
ELSE()
   ADD_DEFINITIONS(-DQT_STATIC)
ENDIF()

IF(${CMAKE_BUILD_TYPE} STREQUAL "Release")
   ADD_DEFINITIONS(-DQT_NO_DEBUG_OUTPUT)
   ADD_DEFINITIONS(-DQT_NO_WARNING_OUTPUT)
ENDIF()

########################
## Configuring MAVSDK ##
########################

FIND_PACKAGE(MAVSDK REQUIRED)

SET(MAVSDK_LIBRARIES
   MAVSDK::mavsdk
   MAVSDK::mavsdk_telemetry
   MAVSDK::mavsdk_alarm
   MAVSDK::mavsdk_inspection
   MAVSDK::mavsdk_checklist
   MAVSDK::mavsdk_hl_action
   MAVSDK::mavsdk_command)

#####################
## Configuring ROS ##
#####################

FIND_PACKAGE(catkin REQUIRED 
   COMPONENTS rviz roscpp pcl_ros geometry_msgs sensor_msgs)

catkin_package(
   INCLUDE_DIRS   include
   LIBRARIES      ${PROJECT_NAME}
   CATKIN_DEPENDS rviz roscpp pcl_ros geometry_msgs sensor_msgs
   DEPENDS        MAVSDK Qt5Core Qt5Widgets Qt5Network)

INCLUDE_DIRECTORIES(SYSTEM ${catkin_INCLUDE_DIRS})

