# ----------------------------------------------------------------------------
#   Uninstall target, for "make uninstall"
# ----------------------------------------------------------------------------
CONFIGURE_FILE("${PROJECT_SOURCE_DIR}/cmake_stuff/cmake_uninstall.cmake.in"
   "${PROJECT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall "${CMAKE_COMMAND}"
   -P "${PROJECT_BINARY_DIR}/cmake_uninstall.cmake")

# ----------------------------------------------------------------------------
#   Create Find${PROJECT_NAME}.cmake file
# ----------------------------------------------------------------------------
CONFIGURE_FILE("${PROJECT_SOURCE_DIR}/cmake_stuff/config.cmake.in"
   "${PROJECT_BINARY_DIR}/Find${PROJECT_NAME}.cmake" @ONLY)
INSTALL(FILES "${PROJECT_BINARY_DIR}/Find${PROJECT_NAME}.cmake"
   DESTINATION lib/cmake/ COMPONENT headers)

# ----------------------------------------------------------------------------
#   Create config.h file
# ----------------------------------------------------------------------------
CONFIGURE_FILE("${PROJECT_SOURCE_DIR}/cmake_stuff/config.h.in"
   "${PROJECT_BINARY_DIR}/config.h" @ONLY)
