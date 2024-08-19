cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(biped_v2_include_dir
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/biped_v2
  ${PROJECT_SOURCE_DIR}/src/common
  ${PROJECT_SOURCE_DIR}/src/planner
  ${PROJECT_SOURCE_DIR}/src/hardware
  ${PROJECT_SOURCE_DIR}/src/estimator
  ${PROJECT_SOURCE_DIR}/src/controller
  ${PROJECT_SOURCE_DIR}/src/opensource/biped_v2
  ${PROJECT_SOURCE_DIR}/src/opensource/common
  ${PROJECT_SOURCE_DIR}/lib/xsens_ros_mti_driver/src
  ${PROJECT_SOURCE_DIR}/lib/xsens_ros_mti_driver/lib/xspublic/
  ${PROJECT_SOURCE_DIR}/lib/Predict_Fp
  ${PROJECT_SOURCE_DIR}/lib/Math
  ${PROJECT_SOURCE_DIR}/lib/EC_Master/src/Common
  ${PROJECT_SOURCE_DIR}/lib/EC_Master/src/SDK/INC 
  ${PROJECT_SOURCE_DIR}/lib/EC_Master/src/SDK/INC/Linux
  ${PROJECT_SOURCE_DIR}/lib/EC_Master/src/Sharelib/Common/ 
  ${PROJECT_SOURCE_DIR}/lib/EC_Master/src/Sharelib/EcMasterDemo/
  ${PROJECT_SOURCE_DIR}/lib/EC_Master/src/Sharelib/Linux/
  ${PROJECT_SOURCE_DIR}/lib/motioncapture/
  ${PROJECT_SOURCE_DIR}/lib/motioncapture/include/
  ${PROJECT_SOURCE_DIR}/lib/DynamixelSDK/include/dynamixel_sdk
  ${PROJECT_SOURCE_DIR}/lib/DynamixelSDK/src/
  ${PROJECT_SOURCE_DIR}/lib/ruierman_controller
  ${PROJECT_SOURCE_DIR}/lib/jodell_claw_driver/include
)

set(biped_v2_link_dir
  ${PROJECT_SOURCE_DIR}/lib/xsens_ros_mti_driver/lib/xspublic/xscontroller
  ${PROJECT_SOURCE_DIR}/lib/xsens_ros_mti_driver/lib/xspublic/xscommon
  ${PROJECT_SOURCE_DIR}/lib/xsens_ros_mti_driver/lib/xspublic/xstypes
  ${PROJECT_SOURCE_DIR}/lib/
  ${PROJECT_SOURCE_DIR}/lib/jodell_claw_driver/lib
  ${PROJECT_SOURCE_DIR}/lib/EC_Master/src/SDK/LIB/Linux/x64/
  ${PROJECT_SOURCE_DIR}/lib/motioncapture/lib/
)

set(target_objs
  ${PROJECT_SOURCE_DIR}/src/controller/pb_controller.cc.o
  ${PROJECT_SOURCE_DIR}/src/planner/plantIK.cc.o
  ${PROJECT_SOURCE_DIR}/lib/Predict_Fp/Predict_Fp.cpp.o
)

add_executable(${CMAKE_PROJECT_NAME}_demo_stand
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/StandRobot.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/StateMachineStand.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/StateMachineError.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/StandTrajectory.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/StandStateEstimation.cpp
)

target_link_directories(${CMAKE_PROJECT_NAME}_demo_stand PUBLIC ${biped_v2_link_dir})
target_include_directories(${CMAKE_PROJECT_NAME}_demo_stand PUBLIC ${biped_v2_include_dir})
target_link_libraries(${CMAKE_PROJECT_NAME}_demo_stand HighlyDynamicRobot_lib)

