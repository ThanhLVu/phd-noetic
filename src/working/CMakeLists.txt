cmake_minimum_required(VERSION 2.8.3)
project(working)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED  
  rospy
  roscpp  
  std_msgs 
  geometry_msgs 
  sensor_msgs
  cv_bridge
  image_transport
  actionlib
  move_base_msgs
  tf2_ros
  message_generation
  visualization_msgs
  ros_tcp_endpoint
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  UnityRGBImage.msg
  ROSUnityMsg.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs visualization_msgs # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS library
# LIBRARIES library
CATKIN_DEPENDS cv_bridge image_transport rospy roscpp sensor_msgs std_msgs tf2_ros message_runtime visualization_msgs ros_tcp_endpoint
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  library
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/neo_simulation.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/neo_simulation_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_neo_simulation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(${CMAKE_SOURCE_DIR}/working/library)
add_library(library library/expand_search.cpp library/clustering.cpp library/vox-file/vox_file.cpp library/fov_check.cpp library/generate_viewpoint.cpp library/fov_check_v2.cpp)
target_link_libraries(library ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable (remove_ground_extract_cluster src/remove_ground_extract_cluster.cpp)
target_link_libraries (remove_ground_extract_cluster ${PCL_LIBRARIES} ${catkin_LIBRARIES} library)

add_executable (intensity_scan_filter src/intensity_scan_filter.cpp)
target_link_libraries (intensity_scan_filter ${catkin_LIBRARIES})

add_executable (camera_calibration_ar src/camera_calibration_ar.cpp)
target_link_libraries (camera_calibration_ar ${catkin_LIBRARIES} library)

add_executable (camera_calibration_qr_sim src/camera_calibration_qr_sim.cpp)
target_link_libraries (camera_calibration_qr_sim ${catkin_LIBRARIES} library)

add_executable (camera_calibration_qr_real src/camera_calibration_qr_real.cpp)
target_link_libraries (camera_calibration_qr_real ${catkin_LIBRARIES} library)

add_executable (multi_camera_calibration_qr_real src/multi_camera_calibration_qr_real.cpp)
target_link_libraries (multi_camera_calibration_qr_real ${catkin_LIBRARIES} library)

add_executable (multi_camera_qr_annotation src/multi_camera_qr_annotation.cpp)
target_link_libraries (multi_camera_qr_annotation ${catkin_LIBRARIES} library)

add_executable (depth_image_to_pointcloud src/depth_image_to_pointcloud.cpp)
target_link_libraries (depth_image_to_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (unity_image_publisher src/unity_image_publisher.cpp)
target_link_libraries (unity_image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (split_depth_image src/split_depth_image.cpp)
target_link_libraries (split_depth_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (combine_cameras_sim src/combine_cameras_sim.cpp)
target_link_libraries (combine_cameras_sim ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (combine_cameras_real src/combine_cameras_real.cpp)
target_link_libraries (combine_cameras_real ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (bounding_box_arm_removed src/bounding_box_arm_removed.cpp)
target_link_libraries (bounding_box_arm_removed ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (custom_control_nextore src/custom_control_nextore.cpp)
target_link_libraries (custom_control_nextore ${catkin_LIBRARIES})

add_executable (voxelize_3d_object src/voxelize_3d_object.cpp)
target_link_libraries (voxelize_3d_object ${catkin_LIBRARIES} ${PCL_LIBRARIES} library)

add_executable (voxelize_3d_object_v2 src/voxelize_3d_object_v2.cpp)
target_link_libraries (voxelize_3d_object_v2 ${catkin_LIBRARIES} ${PCL_LIBRARIES} library ${OCTOMAP_LIBRARIES})

add_executable (ex_scan_lidar_reader src/ex_scan_lidar_reader.cpp)
target_link_libraries (ex_scan_lidar_reader ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (ex_scan_combine_pointclouds src/ex_scan_combine_pointclouds.cpp)
target_link_libraries (ex_scan_combine_pointclouds ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/server_endpoint.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
