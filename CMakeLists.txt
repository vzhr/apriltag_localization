cmake_minimum_required(VERSION 3.0.2)
project(apriltag_localization)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  message_generation
  nodelet
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  tf
  apriltag_ros
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# We default to 'Release' if none was specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type" FORCE)
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Coverage" "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()


add_compile_options("-g" "-funsafe-loop-optimizations" "-fsee" "-funroll-loops" "-fno-math-errno" "-funsafe-math-optimizations" "-ffinite-math-only" "-fno-signed-zeros")
#add_compile_options("-O3" "-funsafe-loop-optimizations" "-fsee" "-funroll-loops" "-fno-math-errno" "-funsafe-math-optimizations" "-ffinite-math-only" "-fno-signed-zeros")

# Note: These options have been turned off to allow for binary releases -
# in local builds, they can be reactivated to achieve higher performance.
 if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64" OR "x86_32")
   message("enabling msse2 for x86_64 or x86_32 architecture")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -msse2 ")
 endif()
 if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm")
   message("enabling -mfpu=neon -mfloat-abi=softfp for ARM architecture")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mcpu=cortex-a9 -mfpu=neon -mtune=cortex-a9 -mvectorize-with-neon-quad -ffast-math ")
 endif()


# Extract the include directories and libraries from apriltag::apriltag as catkin_package() does not support modern cmake.
get_target_property(apriltag_INCLUDE_DIRS apriltag::apriltag INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(apriltag_LIBRARIES apriltag::apriltag INTERFACE_LINK_LIBRARIES)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_transport
    message_runtime
    nodelet
    pluginlib
    roscpp
    sensor_msgs
    std_msgs
    tf
  DEPENDS
    apriltag
    OpenCV
  LIBRARIES
#    ${PROJECT_NAME}_common
#    ${PROJECT_NAME}_continuous_detector
#    ${PROJECT_NAME}_single_image_detector
)

###########
## Build ##
###########

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
file(GLOB_RECURSE CAM_LIBRARY_SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/camera_models/src/*.cc")
message(STATUS "CAM_LIBRARY_SOURCES: ${CAM_LIBRARY_SOURCES}")
add_library(apriltag_localization
        app/apriltag_localization.cpp
        src/common_functions.cpp
        src/apriltag_localization/pose_estimator.cpp
        ${CAM_LIBRARY_SOURCES}
        )
target_link_libraries(apriltag_localization ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} apriltag::apriltag Eigen3::Eigen)
target_compile_options(apriltag_localization PRIVATE "-std=c++14")

add_library(image_nodelet
        app/image_nodelet.cpp
        )
target_link_libraries(image_nodelet ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} Eigen3::Eigen)
target_compile_options(image_nodelet PRIVATE "-std=c++14")

add_executable(apriltag_localization_node app/apriltage_localization_node.cpp)
target_link_libraries(apriltag_localization_node apriltag_localization ${catkin_LIBRARIES})

add_executable(apriltag_localization_with_camera_node app/apriltage_localization_with_camera_node.cpp)
target_link_libraries(apriltag_localization_with_camera_node apriltag_localization image_nodelet ${catkin_LIBRARIES})

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
add_executable(pose_rk4_test test/rk4_test.cpp include/apriltag_localization/pose_system.h test/matplotlibcpp.h)
target_link_libraries(pose_rk4_test Eigen3::Eigen Python3::Python Python3::Module Python3::NumPy)


#set(APRILTAG_ROS "apriltag_ros")

#add_library(${APRILTAG_ROS}_continuous_detector src/continuous_detector.cpp)
#target_link_libraries(${APRILTAG_ROS}_continuous_detector ${APRILTAG_ROS}_common ${catkin_LIBRARIES})
#
#add_library(${APRILTAG_ROS}_single_image_detector src/single_image_detector.cpp)
#target_link_libraries(${APRILTAG_ROS}_single_image_detector ${APRILTAG_ROS}_common ${catkin_LIBRARIES})
#
#add_executable(${APRILTAG_ROS}_continuous_node src/${APRILTAG_ROS}_continuous_node.cpp)
#add_dependencies(${APRILTAG_ROS}_continuous_node ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(${APRILTAG_ROS}_continuous_node ${APRILTAG_ROS}_continuous_detector ${catkin_LIBRARIES})
#
#add_executable(${APRILTAG_ROS}_single_image_server_node src/${APRILTAG_ROS}_single_image_server_node.cpp)
#add_dependencies(${APRILTAG_ROS}_single_image_server_node ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(${APRILTAG_ROS}_single_image_server_node ${APRILTAG_ROS}_single_image_detector ${catkin_LIBRARIES})
#
#add_executable(${APRILTAG_ROS}_single_image_client_node src/${APRILTAG_ROS}_single_image_client_node.cpp)
#add_dependencies(${APRILTAG_ROS}_single_image_client_node ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(${APRILTAG_ROS}_single_image_client_node ${APRILTAG_ROS}_common ${catkin_LIBRARIES})


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  apriltag_localization apriltag_localization_node image_nodelet
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(PROGRAMS scripts/analyze_image DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
