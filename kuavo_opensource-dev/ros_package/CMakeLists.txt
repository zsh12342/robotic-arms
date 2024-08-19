cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(highlyDynamicRobot_node)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  drake
)
find_package(Python3 3.8 EXACT COMPONENTS Interpreter Development NumPy REQUIRED)
# find_package(PythonInterp REQUIRED 3.8)
if(Python3_FOUND)
    message(STATUS "Found Python: ${Python3_EXECUTABLE}")
    message(STATUS "Python version: ${Python3_VERSION}")
    
else()
    message(FATAL_ERROR "Python not found")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  )
add_executable(${PROJECT_NAME} 
./src/main.cc
)
target_link_directories(${PROJECT_NAME} PUBLIC ${biped_v2_link_dir})

# target_link_libraries(${PROJECT_NAME} HighlyDynamicRobot_lib )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  HighlyDynamicRobot_lib
)
