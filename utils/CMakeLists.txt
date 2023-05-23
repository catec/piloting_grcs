IF(${BUILD_UTILS})

   INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
   INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
   INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

   FILE(GLOB SRCS "*.cpp")
   FILE(GLOB HDRS "*.h")

   SET(APP_NAME   ${PROJECT_NAME}_node)

   ADD_EXECUTABLE(${APP_NAME} ${SRCS} ${HDRS})

   TARGET_LINK_LIBRARIES(${APP_NAME}
      ${PROJECT_NAME}_common
      ${PROJECT_NAME}_communications
      ${PROJECT_NAME}_core
      ${PROJECT_NAME}_dataModel
      ${PROJECT_NAME}_gui
      ${PROJECT_NAME}_maps)

   SET(ICON_NAME   "PILOTING GRCS")

   CONFIGURE_FILE("${PROJECT_SOURCE_DIR}/cmake_stuff/app.desktop.in"
      "${PROJECT_BINARY_DIR}/${APP_NAME}.desktop" @ONLY)

   CONFIGURE_FILE("${PROJECT_SOURCE_DIR}/cmake_stuff/postinst.in"
      "${PROJECT_BINARY_DIR}/postinst" @ONLY)
   CONFIGURE_FILE("${PROJECT_SOURCE_DIR}/cmake_stuff/postrm.in"
      "${PROJECT_BINARY_DIR}/postrm" @ONLY)

   INSTALL(FILES "${PROJECT_BINARY_DIR}/${APP_NAME}.desktop"
      DESTINATION share/applications)

   INSTALL(TARGETS ${APP_NAME}
      RUNTIME DESTINATION  bin
      COMPONENT            applications)

ENDIF()
