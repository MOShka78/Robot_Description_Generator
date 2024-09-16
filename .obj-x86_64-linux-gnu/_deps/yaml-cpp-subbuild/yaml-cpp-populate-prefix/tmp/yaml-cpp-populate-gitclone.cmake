
if(NOT "yaml-cpp-src" IS_NEWER_THAN "/home/eliseev/workspace/urdf_expansion_ws/src/URDF_Expansion/.obj-x86_64-linux-gnu/_deps")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/home/eliseev/workspace/urdf_expansion_ws/src/URDF_Expansion/.obj-x86_64-linux-gnu/_deps'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/home/eliseev/workspace/urdf_expansion_ws/src/URDF_Expansion/.obj-x86_64-linux-gnu/_deps/yaml-cpp-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/eliseev/workspace/urdf_expansion_ws/src/URDF_Expansion/.obj-x86_64-linux-gnu/_deps/yaml-cpp-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"  clone --no-checkout --depth 1 --no-single-branch --origin "0.8.0" "https://github.com/jbeder/yaml-cpp.git" ""
    WORKING_DIRECTORY "advice.detachedHead=false"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/jbeder/yaml-cpp.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"  checkout VCS_VERSION --
  WORKING_DIRECTORY "advice.detachedHead=false/"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'VCS_VERSION'")
endif()

set(init_submodules master)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git"  submodule update origin --init TRUE
    WORKING_DIRECTORY "advice.detachedHead=false/"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: 'advice.detachedHead=false/'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "yaml-cpp-src"
    "/home/eliseev/workspace/urdf_expansion_ws/src/URDF_Expansion/.obj-x86_64-linux-gnu/_deps"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/eliseev/workspace/urdf_expansion_ws/src/URDF_Expansion/.obj-x86_64-linux-gnu/_deps'")
endif()

