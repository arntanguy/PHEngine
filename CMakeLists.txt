SET(PROJECT_NAME PHEngine)
SET(BIN_DIR bin)
SET(DOC_DIR "doc")
SET(SRC_DIR src)
SET(INCLUDE_DIR .)
SET(CMAKE_BUILD_TYPE Debug)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
set(LIB_TYPE SHARED) # STATIC or SHARED

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CSS_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

set (CMAKE_CXX_FLAGS -std=gnu++0x)

project(${PROJECT_NAME})
cmake_minimum_required(VERSION 2.6)
#set(CMAKE_CXX_FLAGS "-Wall -W -Werror -ansi -pedantic -g")

find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(SFML COMPONENTS system window network audio graphics REQUIRED)
find_package(ASSIMP REQUIRED)
find_package(CGEngine REQUIRED)


# Set all debug and info messages on
add_definitions(-DDEBUG=1 -DINFO=1)

### DEFINE NEEDED SOURCES
SET(SOURCES
${SRC_DIR}/ParticleGenerator.cpp
${SRC_DIR}/PhysicsWorld.cpp
${SRC_DIR}/RigidBody.cpp
${SRC_DIR}/BoundingBox.cpp
${SRC_DIR}/AABoundingBox.cpp
)

include_directories (${SRC_DIR} ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIR} ${ASSIMP_INCLUDE_DIRS} ${INCLUDE_DIR} ${CGENGINE_INCLUDE_DIRS})

### CREATE LIBRARY
add_library(${PROJECT_NAME} ${LIB_TYPE} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${SFML_LIBRARIES} ${ASSIMP_LIBRARIES} ${CGENGINE_LIBRARIES})



# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
     configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile ${BIN_DIR}/Doxyfile @ONLY)
     add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/${BIN_DIR}/Doxyfile
     WORKING_DIRECTORY ${DOC_DIR}
     COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif(DOXYGEN_FOUND)

