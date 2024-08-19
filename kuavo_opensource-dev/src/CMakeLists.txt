cmake_minimum_required(VERSION 3.0.0)

if(Python3_FOUND)
    message(STATUS "Found Python: ${Python3_EXECUTABLE}")
    message(STATUS "Python version: ${Python3_VERSION}")
else()
    message(FATAL_ERROR "Python not found")
endif()


if(NOT OPENSOURCE)
    add_subdirectory_if_exists(common)
    add_subdirectory_if_exists(planner)
    add_subdirectory_if_exists(controller)
    add_subdirectory_if_exists(estimator)
    add_subdirectory_if_exists(hardware)
endif()
# add_subdirectory_if_exists(biped_v2)
add_subdirectory_if_exists(opensource)

add_subdirectory_if_exists(demos)
