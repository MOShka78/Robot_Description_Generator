cmake_minimum_required(VERSION 3.22)
project(urdf_expansion)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(URDF_EXPANSION_LIBS rclcpp urdf urdfdom urdfdom_headers yaml-cpp
                        ament_index_cpp)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${URDF_EXPANSION_LIBS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)
include_directories(${URDFDOM_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})

add_executable(urdf_expansion src/urdf_expansion.cpp)

ament_target_dependencies(urdf_expansion PUBLIC ${URDF_EXPANSION_LIBS})
target_link_libraries(urdf_expansion PUBLIC ${YAML_CPP_LIBRARIES})

target_include_directories(
  urdf_expansion PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)

install(TARGETS urdf_expansion DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
