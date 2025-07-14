cmake_minimum_required(VERSION 3.20)
project(ThreeBodySimulation VERSION 1.0.0 LANGUAGES CXX)

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags for better performance and debugging
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

# Find SFML
find_package(PkgConfig REQUIRED)
pkg_check_modules(SFML REQUIRED sfml-graphics>=2.5 sfml-window>=2.5 sfml-system>=2.5)

# Create executable
set(SOURCES
    main.cpp
    src/Body.cpp
    src/PhysicsEngine.cpp
    src/Renderer.cpp
    src/Application.cpp
    src/Logger.cpp
    src/ConfigManager.cpp
)

set(HEADERS
    include/Body.h
    include/PhysicsEngine.h
    include/Renderer.h
    include/Application.h
    include/Logger.h
    include/ConfigManager.h
    include/Constants.h
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GLM_INCLUDE_DIRS}
    ${SFML_INCLUDE_DIRS}
    ${nlohmann_json_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OPENGL_LIBRARIES}
    glfw
    glm::glm
    ${SFML_LIBRARIES}
    nlohmann_json::nlohmann_json
)

# Compiler flags from pkg-config
target_compile_options(${PROJECT_NAME} PRIVATE ${SFML_CFLAGS})

# Copy config files to build directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/initial_conditions.json 
               ${CMAKE_CURRENT_BINARY_DIR}/config/initial_conditions.json COPYONLY)

# Create output directories
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/logs)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/config)

# Platform-specific settings
if(WIN32)
    target_compile_definitions(${PROJECT_NAME} PRIVATE NOMINMAX)
    # Copy SFML DLLs on Windows
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(SFML_SUFFIX "-d")
    else()
        set(SFML_SUFFIX "")
    endif()
endif()

# Installation rules
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(FILES config/initial_conditions.json DESTINATION share/${PROJECT_NAME}/config)

# Print build information
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")