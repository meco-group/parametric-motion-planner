cmake_minimum_required(VERSION 3.10)
project(parametric_motion_planner LANGUAGES CXX)

# Specify the version of the library
set(MY_LIBRARY_VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find CasADi package (Assumes CasADi is installed or its path is provided)
find_package(CasADi REQUIRED)

# Specify the include directories for the library
include_directories(include ${CASADI_INCLUDE_DIRS}/casadi)

# Collect source files
file(GLOB SOURCES "src/*.cpp")

### Create example
add_executable(my_executable ${SOURCES})
target_link_libraries(my_executable casadi)
###

# Create the library
add_library(${PROJECT_NAME} STATIC ${SOURCES})

# Link the CasADi library as a public dependency
target_link_libraries(${PROJECT_NAME} PUBLIC ${CASADI_LIBRARIES})

# Install master header
install(FILES include/${PROJECT_NAME}.hpp
    DESTINATION include/${PROJECT_NAME})

# Install core headers
install(DIRECTORY include/core
    DESTINATION include/${PROJECT_NAME})

set(INSTALL_LIB_DIR lib)
set(INSTALL_BIN_DIR bin) 

# Specify the installation paths for the library and header files
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${INSTALL_LIB_DIR}
    ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
    RUNTIME DESTINATION ${INSTALL_BIN_DIR})

# Export the target for usage in other projects
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Config.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${INSTALL_LIB_DIR}/cmake/${PROJECT_NAME})

# Provide a version file for the package
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION "${INSTALL_LIB_DIR}/cmake/${PROJECT_NAME}"
)

# Install the config file
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    DESTINATION "${INSTALL_LIB_DIR}/cmake/${PROJECT_NAME}")

# Make the package available
include(CMakePackageConfigHelpers)
