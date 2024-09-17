cmake_minimum_required(VERSION 3.22)
project(robot_description_generator)

set(CMAKE_CXX_STANDARD 17)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif ()

set(ROBOT_DESCRIPTION_GENERATOR_LIBS
    # cmake-format: sortable
    ament_index_cpp
    rclcpp
    urdf
    urdfdom
    urdfdom_headers
    yaml-cpp)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach (Dependency IN ITEMS ${ROBOT_DESCRIPTION_GENERATOR_LIBS})
  find_package(${Dependency} REQUIRED)
endforeach ()

include_directories(include)
include_directories(${URDFDOM_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})

add_executable(robot_description_generator src/description_generator_node.cpp
                                           src/robot_description_generator.cpp)

ament_target_dependencies(robot_description_generator PUBLIC
                          ${ROBOT_DESCRIPTION_GENERATOR_LIBS})
target_link_libraries(robot_description_generator PUBLIC ${YAML_CPP_LIBRARIES})

target_include_directories(
  robot_description_generator
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

install(TARGETS robot_description_generator DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
