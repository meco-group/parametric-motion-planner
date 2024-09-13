# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name and language
project(parametric-motion-planner LANGUAGES CXX)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add include directory (for header files)
include_directories(${PROJECT_SOURCE_DIR}/include)

# Find the CasADi library (you can specify the path if needed)
find_package(CasADi REQUIRED)

# Look for casadi
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(CASADI REQUIRED)
include_directories(${CASADI_INCLUDE_DIR})

# Collect all the source files in the src directory
file(GLOB_RECURSE SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")

# Add the executable target
add_executable(${PROJECT_NAME} ${SOURCES})

# Link any libraries here if needed, for example:
# target_link_libraries(${PROJECT_NAME} <library-name>)
target_link_libraries(${PROJECT_NAME} ${CASADI_LIBRARIES})
