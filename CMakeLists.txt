cmake_minimum_required(VERSION 3.10)
project(parametric_motion_planner LANGUAGES CXX)

# Specify the version of the library
set(MY_LIBRARY_VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Detect operating system (Windows or Linux/Unix)
if(WIN32)
    message(STATUS "Configuring for Windows")
    
    #### Find CasADi
    # Set the path to CasADi library and build directories
    set(CASADI_DIR "C:/Users/Administrator/source/repos/casadi/build-mingw-ucrt64")
    find_library(CASADI_LIBRARY NAMES casadi PATHS "${CASADI_DIR}")

    # Include CasADi headers
    set(CASADI_INCLUDE "C:/Users/Administrator/source/repos/casadi")
    include_directories("${CASADI_INCLUDE}")

    # Include the build directory where casadi_export.h is generated
    include_directories("${CASADI_INCLUDE}/build/")

    ### Find Pybind11
    # ~/.local/lib/python3.10/site-packages/pybind11/share/cmake/pybind11/pybind11Config.cmake
    # find_package(pybind11 REQUIRED)

    #### Collect source files
    file(GLOB SOURCES "src/*.cpp")

    # remove the bindings.cpp file
    list(FILTER SOURCES EXCLUDE REGEX "src/bindings.cpp")

    include_directories(include)


    #### Create the library
    add_library(${PROJECT_NAME} STATIC ${SOURCES})


    #### Link the CasADi library as a public dependency
    target_link_libraries(${PROJECT_NAME} PUBLIC ${CASADI_LIBRARY})


    #### Installation
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

    # Copy the casadi.dll to the output directory after building (Windows specific)
    #set(CASADI_DLL "C:/Users/u0143705/Documents/PhD/casadi-build-windows/casadi/out/build/debug/casadi.dll")
    #add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    #    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    #        "${CASADI_DLL}"
    #        $<TARGET_FILE_DIR:${PROJECT_NAME}>)
else()
    message(STATUS "Configuring for Unix/Linux")

    # Find CasADi package (Assumes CasADi is installed or its path is provided)
    find_package(CasADi REQUIRED)

    # Specify the include directories for the library
    include_directories(include ${CASADI_INCLUDE_DIRS}/casadi)

    # find Pybind11 package
    find_package(pybind11 REQUIRED PATHS ~/.local/lib/python3.10/site-packages/pybind11/share/cmake/pybind11/pybind11Config.cmake)

    # Collect source files
    file(GLOB SOURCES "src/*.cpp")

    # Create the library
    add_library(${PROJECT_NAME} STATIC ${SOURCES})
    target_include_directories(${PROJECT_NAME} PRIVATE ${pybind11_INCLUDE_DIRS})

    ### Create example
    add_executable(my_executable ${SOURCES})
    target_link_libraries(my_executable casadi ${PROJECT_NAME} pybind11::embed)
    ###

    ### Create pybind module
    pybind11_add_module(parametric_motion_planner_module MODULE ${SOURCES})
    target_link_libraries(parametric_motion_planner_module PRIVATE casadi ${PROJECT_NAME} pybind11::module)
    ###


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
endif()
