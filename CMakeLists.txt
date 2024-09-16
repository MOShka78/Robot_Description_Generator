cmake_minimum_required(VERSION 3.8)
project(generate_robot_urdf)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  VCS_VERSION 0.8.0)

FetchContent_MakeAvailable(yaml-cpp)

set(GENERATE_ROBOT_URDF_LIBS moveit_core)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${GENERATE_ROBOT_URDF_LIBS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_executable(generate_robot_urdf src/generate_robot_urdf.cpp)

ament_target_dependencies(generate_robot_urdf PUBLIC
                          ${GENERATE_ROBOT_URDF_LIBS})
target_link_libraries(generate_robot_urdf PUBLIC yaml-cpp::yaml-cpp)

target_include_directories(
  generate_robot_urdf
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

install(TARGETS generate_robot_urdf DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
