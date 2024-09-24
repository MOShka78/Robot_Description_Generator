cmake_minimum_required(VERSION 3.22)
project(BU015X_description)

option(REBUILD_XACRO "REBUILD_XACRO" OFF)

if(REBUILD_XACRO)
  execute_process(
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMAND xacro urdf/BU015X.urdf.xacro
    OUTPUT_FILE urdf/BU015X.urdf)
endif()

if($ENV{ROS_VERSION} STREQUAL "1")
  find_package(catkin REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  find_package(GTest)

  catkin_package(CATKIN_DEPENDS)

  install(DIRECTORY launch meshes urdf
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

elseif($ENV{ROS_VERSION} STREQUAL "2")
  find_package(ament_cmake REQUIRED)

  install(DIRECTORY launch meshes urdf config DESTINATION share/${PROJECT_NAME})

  ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_package()

else()
  message("Unknown ROS distro:")
  message($ENV{ROS_DISTRO})
endif()
