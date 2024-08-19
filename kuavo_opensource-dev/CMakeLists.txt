cmake_minimum_required(VERSION 3.0.0)
cmake_policy(SET CMP0048 NEW)
project(dynamic_biped VERSION 0.1.0)

option(KUAVO_INTERNAL_CONTROLLER "Compile libs for internal_controller repo" OFF)
option(OPENSOURCE "Compile opensource respository" ON)
option(ONLY_USE_HALF_UP_BODY "Use half up body mode" OFF)
option(USE_PYTHON_RUIWO_SDK "Use Python RuiwoController SDK" ON)

if(ONLY_USE_HALF_UP_BODY)
  message(STATUS "Enabling ONLY_USE_HALF_UP_BODY option is ON")
  add_compile_definitions(HALF_UP_BODY)
else()
  message(STATUS "Compiling without ONLY_USE_HALF_UP_BODY defined")
endif()

if(USE_PYTHON_RUIWO_SDK)
  message(STATUS "Enabling USE_PYTHON_RUIWO_SDK option is ON")
else()
  message(STATUS "Compiling without USE_PYTHON_RUIWO_SDK defined, OFF")
endif()

if(KUAVO_INTERNAL_CONTROLLER)
  message(STATUS "Enabling KUAVO_INTERNAL_CONTROLLER option is ON")
endif()

macro(add_subdirectory_if_exists dir)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}/CMakeLists.txt")
    add_subdirectory(${dir})
  else()
    message(STATUS "Skipping ${dir}, directory not found")
  endif()
endmacro()

find_package(drake CONFIG REQUIRED)
find_package(lcm CONFIG REQUIRED)
find_package(Python3 3.8 EXACT COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(Eigen3 REQUIRED)

# Check and Install dependencies
execute_process(
  COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tools/install_deps.sh
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

execute_process(
  COMMAND git describe
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_DESCRIBE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
message("Git describe: ${GIT_DESCRIBE}")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/opensource/common/git_describe.h.in" "${CMAKE_CURRENT_SOURCE_DIR}/src/opensource/common/git_describe.h" @ONLY)

# set ROBOT_VERSION for test
# set(ROBOT_VERSION 30)

if(NOT DEFINED ROBOT_VERSION)
    # 从环境变量中读取版本号
    if (NOT DEFINED ENV{ROBOT_VERSION})
        message(STATUS "ROBOT_VERSION environment variable not set.")
        message(STATUS "Defaulting to version 3.2")
        set(ROBOT_VERSION 32)
    else()
        set(ROBOT_VERSION $ENV{ROBOT_VERSION})
        message(STATUS "ROBOT_VERSION from system is ${ROBOT_VERSION}.")
    endif()
else()
  message(STATUS "ROBOT_VERSION is " ${ROBOT_VERSION})
endif()

add_compile_definitions(ROBOT_VERSION_INT=${ROBOT_VERSION})


# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g") # 用于性能测试

option(EXAMPLE "build example" OFF)

# 设置默认构建类型为 Release
if(NOT CMAKE_BUILD_TYPE) # 接受命令行指定CMAKE_BUILD_TYPE
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "No build type specified. Defaulting to Release.")
endif()
message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_DEBUG "-O0")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -s")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g) # 添加调试信息
else()
    add_compile_options(-O3) # 设置优化等级为 3
endif()

if (UNIX)
	add_definitions(-D_LINUX)
endif()

include_directories(./include)
include_directories(./lcm_types)


message("build example: " ${EXAMPLE})
if(EXAMPLE)
  add_subdirectory_if_exists(example)
endif(EXAMPLE)

# file(COPY ./models DESTINATION ${CMAKE_BINARY_DIR}/)
# file(COPY ./lib/Predict_Fp/ DESTINATION ${CMAKE_BINARY_DIR}/)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

# Check if we are using Catkin
if(DEFINED CATKIN_DEVEL_PREFIX)
  # This is a Catkin package.
  # Find the ROS libraries
  message(STATUS "using catkin_make")

  add_definitions(-DKUAVO_CATKIN_MAKE_OPTION)
  ## Find catkin macros and libraries
  ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  ## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    rospy
    geometry_msgs
  )
  include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
  )
  add_message_files(
    DIRECTORY ./ros_package/msg/
    FILES walkCommand.msg
    ECJointMotordata.msg
    robotQVTau.msg
    robotArmQVVD.msg
    robotTorsoState.msg
    robotPhase.msg
    robotHandPosition.msg
    robotHeadMotionData.msg
    armTargetPoses.msg
    robotArmPose.msg
    centerOfMass.msg
    robotImuGyroAcc.msg
    EndEffectorStateStamped.msg
  )
  add_service_files(
    DIRECTORY ./ros_package/srv/
    FILES srvChangePhases.srv  
    srvClearPositionCMD.srv
    srvchangeCtlMode.srv
    changeArmCtrlMode.srv
    changeAMBACCtrlMode.srv
    controlEndHand.srv
    comPosition.srv
    getComPosition.srv
    changeHandArmPosesByConfigName.srv
    controlJodellClaw.srv
  )
  generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
  )
  catkin_package(
    CATKIN_DEPENDS message_runtime
  )

  add_subdirectory_if_exists("./ros_package")
  # set_directory_properties(PROPERTIES
  #   ADDITIONAL_MAKE_CLEAN_FILES "./lib"
  # )
  # This package contains a ROS executable.
endif()



if (NOT OPENSOURCE)
  add_subdirectory_if_exists(lib/Math)
  add_subdirectory_if_exists(lib/EC_Master)
  add_subdirectory_if_exists(lib/xsens_ros_mti_driver)
  add_subdirectory_if_exists(lib/motioncapture)
  add_subdirectory_if_exists(lib/ruierman_controller)
  add_subdirectory_if_exists(lib/ruiwo_controller)
  add_subdirectory_if_exists(lib/ruiwo_controller_cxx)
  add_subdirectory_if_exists(lib/hand_sdk)
  add_subdirectory_if_exists(lib/jodell_claw_driver)
  add_subdirectory_if_exists(lib/DynamixelSDK)
endif()
add_subdirectory_if_exists(lcm)
add_subdirectory_if_exists(src)

