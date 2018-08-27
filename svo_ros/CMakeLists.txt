project(svo_ros)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple()

#############################################################################
# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -Werror -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")

IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
  ADD_DEFINITIONS(-DHAVE_FAST_NEON)
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3 -mno-avx")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -ffast-math -fno-finite-math-only")

set(USE_GTSAM true)

#############################################################################
# Build library

set(HEADERS
  include/svo_ros/svo_interface.h
  include/svo_ros/svo_factory.h
  include/svo_ros/visualizer.h
  include/svo_ros/svo_nodelet.h
)

set(SOURCES
  src/svo_interface.cpp
  src/svo_factory.cpp
  src/svo_node_base.cpp
  src/visualizer.cpp
)

if(USE_GTSAM)
    ADD_DEFINITIONS(-DSVO_USE_BACKEND)
    list(APPEND HEADERS
      include/svo_ros/backend_visualizer.h
      include/svo_ros/backend_factory.h)
    list(APPEND SOURCES
      src/backend_visualizer.cpp
      src/backend_factory.cpp)
endif()

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

#############################################################################
# Build executables
cs_add_executable(svo_node src/svo_node.cpp)
target_link_libraries(svo_node ${PROJECT_NAME})

cs_add_library(svo_nodelet src/svo_nodelet.cpp)
target_link_libraries(svo_nodelet ${PROJECT_NAME})

#############################################################################
# Install
cs_install()
install(FILES svo_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
cs_export()
