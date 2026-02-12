cmake_minimum_required(VERSION 3.16)
project(robot_description_generator)

set(CMAKE_CXX_STANDARD 17)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif ()

set(ROBOT_DESCRIPTION_GENERATOR_LIBS urdf urdfdom urdfdom_headers yaml-cpp)

find_package(Threads REQUIRED)

if ($ENV{ROS_VERSION} STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS urdf roscpp)
elseif ($ENV{ROS_VERSION} STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  find_package(ament_index_cpp REQUIRED)
endif ()

foreach (Dependency IN ITEMS ${ROBOT_DESCRIPTION_GENERATOR_LIBS})
  find_package(${Dependency} REQUIRED)
endforeach ()

include_directories(include)
include_directories(${URDFDOM_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})

add_executable(robot_description_generator src/description_generator_node.cpp
                                           src/robot_description_generator.cpp)

if ($ENV{ROS_VERSION} STREQUAL "1")
  add_compile_definitions(ROS ROS1)
  # CATKIN_DEPENDS only catkin packages; urdfdom/urdfdom_headers/yaml-cpp are system libs
  catkin_package(INCLUDE_DIRS CATKIN_DEPENDS urdf roscpp)
  include_directories(include ${catkin_INCLUDE_DIRS})
  target_link_libraries(robot_description_generator PUBLIC
                        ${catkin_LIBRARIES} ${URDFDOM_LIBRARIES} Threads::Threads)

elseif ($ENV{ROS_VERSION} STREQUAL "2")
  add_compile_definitions(ROS2)

  ament_target_dependencies(robot_description_generator PUBLIC
                            ${ROBOT_DESCRIPTION_GENERATOR_LIBS} ament_index_cpp)
  target_link_libraries(robot_description_generator PUBLIC Threads::Threads)

  install(TARGETS robot_description_generator DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY include/ DESTINATION include/)

  install(DIRECTORY config test DESTINATION share/${PROJECT_NAME})
else ()
  message(FATAL_ERROR "Unknown ROS distro:")
  message($ENV{ROS_DISTRO})
endif ()

target_link_libraries(robot_description_generator PUBLIC ${YAML_CPP_LIBRARIES})

target_include_directories(
  robot_description_generator
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

if (BUILD_TESTING AND ($ENV{ROS_VERSION} STREQUAL "1"))
  find_package(rostest REQUIRED)
  catkin_add_gtest(
    robot_description_generator_test src/robot_description_generator.cpp
    test/robot_description_generator_test.cpp)
  target_link_libraries(robot_description_generator_test ${catkin_LIBRARIES}
                        ${URDFDOM_LIBRARIES} ${YAML_CPP_LIBRARIES} Threads::Threads)
  target_include_directories(
    robot_description_generator_test
    PRIVATE include ${catkin_INCLUDE_DIRS} ${URDFDOM_INCLUDE_DIRS}
            ${YAML_CPP_INCLUDE_DIRS})
endif ()

if (BUILD_TESTING AND ($ENV{ROS_VERSION} STREQUAL "2"))

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    robot_description_generator_test src/robot_description_generator.cpp
    test/robot_description_generator_test.cpp)

  ament_target_dependencies(robot_description_generator_test
                            ${ROBOT_DESCRIPTION_GENERATOR_LIBS} ament_index_cpp)
  target_link_libraries(robot_description_generator_test ${YAML_CPP_LIBRARIES})

endif ()

if ($ENV{ROS_VERSION} STREQUAL "2")
  ament_package()
endif ()
