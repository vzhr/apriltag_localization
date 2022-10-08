cmake_minimum_required(VERSION 3.0.2)
project(ethz_apriltag2)

add_definitions(-fPIC -O3)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
)

#add_compile_options("-g" "-funsafe-loop-optimizations" "-fsee" "-funroll-loops" "-fno-math-errno" "-funsafe-math-optimizations" "-ffinite-math-only" "-fno-signed-zeros")
add_compile_options("-O3" "-funsafe-loop-optimizations" "-fsee" "-funroll-loops" "-fno-math-errno" "-funsafe-math-optimizations" "-ffinite-math-only" "-fno-signed-zeros")

# Note: These options have been turned off to allow for binary releases -
# in local builds, they can be reactivated to achieve higher performance.
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64" OR "x86_32")
  message("enabling msse2 for x86_64 or x86_32 architecture")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm")
  message("enabling -mfpu=neon -mfloat-abi=softfp for ARM architecture")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mcpu=cortex-a9 -mfpu=neon -mtune=cortex-a9 -mvectorize-with-neon-quad -ffast-math ")
endif()

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#library
file(GLOB SOURCE_FILES "src/*.cc")
add_library(${PROJECT_NAME}  ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})

#demo
if(NOT APPLE)
#  add_executable(apriltags_demo src/example/apriltags_demo.cpp src/example/Serial.cpp)
#  target_link_libraries(apriltags_demo ${PROJECT_NAME} v4l2)
endif()

##################
## Installation ##
##################

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
