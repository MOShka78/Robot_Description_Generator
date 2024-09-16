cmake_minimum_required(VERSION 3.22)
project(urdf_expansion)

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

set(URDF_EXPANSION_LIBS rclcpp urdf urdfdom urdfdom_headers)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${URDF_EXPANSION_LIBS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)
include_directories(${URDFDOM_INCLUDE_DIRS})

add_executable(urdf_expansion src/urdf_expansion.cpp)

ament_target_dependencies(urdf_expansion PUBLIC ${URDF_EXPANSION_LIBS})
target_link_libraries(urdf_expansion PUBLIC yaml-cpp::yaml-cpp)

target_include_directories(
  urdf_expansion PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)

install(TARGETS urdf_expansion DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
