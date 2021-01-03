cmake_minimum_required(VERSION 2.8)
project(visionProject)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

find_package(realsense2 REQUIRED)
message(STATUS "Found realsense2 in ${realsense_INCLUDE_DIR}")
find_package(GLUT REQUIRED)
message(STATUS "Found GLUT in ${GLUT_INCLUDE_DIR}")
find_package(glfw3 REQUIRED)
message(STATUS "Found GLFW3 in ${GLFW3_INCLUDE_DIR}")
find_package(GLEW REQUIRED)
message(STATUS "Found GLEW in ${GLEW_INCLUDE_DIR}")
find_package(OpenGL REQUIRED)
message(STATUS "Found OpenGL in ${OPENGL_INCLUDE_DIR}")


find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
find_library( GLUT_glut_LIBRARY NAMES glut glut32 freeglut
        PATHS
        ${OPENGL_LIBRARY_DIR}
        ${GLUT_ROOT_PATH}/Release
        ${GLFW3_LIBRARY_DIR}
        ${GLEW_LIBRARY_DIR}
        )

include_directories(.)
include_directories(include ${GLFW_INCLUDE_DIRS} ${realsense_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS})

add_executable(visionProject
        ObjModel.cpp
        ObjModel.h
        realsense3.cpp
        SettingLoader.h
        SpacePrimitives.cpp
        SpacePrimitives.h
        TrackerUtils.h
        ImagePlane.cpp
        ImagePlane.h)

target_include_directories(visionProject PRIVATE ${realsense_INCLUDE_DIR} ${GLFW_LIBRARIES} ${OPENGL_INCLUDE_DIR})
target_link_libraries(visionProject ${OpenCV_LIBS} ${realsense2_LIBRARY} ${GLFW_STATIC_LIBRARIES} ${OPENGL_gl_LIBRARY} glfw GLEW)