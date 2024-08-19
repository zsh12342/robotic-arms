cmake_minimum_required(VERSION 3.0.0)
# project(biped_v2 VERSION 0.1.0)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(biped_v2_include_dir
  ./
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
  ${PROJECT_SOURCE_DIR}/lib/hand_sdk
  ${PROJECT_SOURCE_DIR}/lib/jodell_claw_driver/include
  ${PROJECT_SOURCE_DIR}/lib/hand_sdk
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

if(USE_PYTHON_RUIWO_SDK)
  List(APPEND biped_v2_link_dir ${PROJECT_SOURCE_DIR}/lib/ruiwo_controller/)
  List(APPEND biped_v2_include_dir ${PROJECT_SOURCE_DIR}/lib/ruiwo_controller/)
  set(ruiwo_lib ruiwo_actuatorPythonLib)
else()
  List(APPEND biped_v2_link_dir ${PROJECT_SOURCE_DIR}/lib/ruiwo_controller_cxx/3rd_party/bin/unix64/release)
  List(APPEND biped_v2_include_dir ${PROJECT_SOURCE_DIR}/lib/ruiwo_controller_cxx/include
                                   ${PROJECT_SOURCE_DIR}/lib/ruiwo_controller_cxx/3rd_party/include)
  set(ruiwo_lib ruiwo_actuatorCXXLib usb-1.0 bmapi64)
endif()

set(target_objs
  ${PROJECT_SOURCE_DIR}/src/controller/pb_controller.cc.o
  ${PROJECT_SOURCE_DIR}/src/planner/plantIK.cc.o
  ${PROJECT_SOURCE_DIR}/lib/Predict_Fp/Predict_Fp.cpp.o
)

link_directories(${biped_v2_link_dir})
# add_library(pb_controller_obj OBJECT ../common/pb_controller.cc)
# target_link_libraries(pb_controller_obj pthread gflags lcm drake::drake elmo xsens_mti_driver casadi ncurses)
# # 移除所有符号  信息
# set_target_properties(pb_controller_obj PROPERTIES LINK_FLAGS "-s")

# HighlyDynamicRobot_lib
set(OPENSOURCE_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/biped_v2/HighlyDynamicRobot.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/common/config.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/common/env_utils.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/common/robotStateStorage.cpp
  )
add_library(HighlyDynamicRobot_lib STATIC ${OPENSOURCE_SRC} )

target_include_directories(HighlyDynamicRobot_lib PUBLIC ${biped_v2_include_dir})

target_link_directories(HighlyDynamicRobot_lib PUBLIC ${biped_v2_link_dir})

set(RobotLink_LIBS 
  hardware
  planner
  controllers
  estimators
  ankle_controller
  ankle_estimator
  arm_ankle_estimator 
  arm_ankle_controller
  common
  # ${target_objs} 
  xsens_mti_driver
  pbcwalk-lcm-types 
  ec_app
  ec_master_main
  EcMaster
  AtemRasSrv
  gflags
  xscontroller 
  xscommon 
  xstypes 
  lcm 
  mathtools
  dl 
  pthread  
  drake::drake 
  stdc++fs
  motionClient
  nokov_sdk
  dxl_x64_cpp
  jodell_claw_driver
  jodellTool2
  ruierman_actuatorLib
  ${ruiwo_lib}
  hand_sdkLib
  ${Boost_LIBRARIES}
  Python3::Python
  yaml-cpp 
  )
if(OPENSOURCE)
  list(APPEND RobotLink_LIBS 
  ${target_objs} 
  casadi 
  ncurses
  )
endif()

if(DEFINED CATKIN_DEVEL_PREFIX)
  target_link_libraries(HighlyDynamicRobot_lib ${RobotLink_LIBS} ${catkin_LIBRARIES})
else()
  target_link_libraries(HighlyDynamicRobot_lib ${RobotLink_LIBS})
endif()
# if(NOT KUAVO_INTERNAL_CONTROLLER)
#   target_link_libraries(HighlyDynamicRobot_lib controllers)
# endif()

# Define the output directory for the library file
set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/lib")

# Ensure the target directory exists before the build starts
add_custom_command(TARGET HighlyDynamicRobot_lib PRE_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${LIBRARY_OUTPUT_PATH})

# Add a post-build command to copy the library file
add_custom_command(TARGET HighlyDynamicRobot_lib POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
  $<TARGET_FILE:HighlyDynamicRobot_lib>
  ${LIBRARY_OUTPUT_PATH})



file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/CURRENT_SOURCE_DIR "${CMAKE_SOURCE_DIR}")
# Add the executable with the project name and the collected source files
add_executable(kuavoController ${CMAKE_CURRENT_SOURCE_DIR}/../../src/opensource/biped_v2/main.cc)
target_link_directories(kuavoController PUBLIC ${biped_v2_link_dir})
target_include_directories(kuavoController PUBLIC ${biped_v2_include_dir})

# Link the executable with the required library
target_link_libraries(kuavoController HighlyDynamicRobot_lib)
if(KUAVO_INTERNAL_CONTROLLER)
  target_link_libraries(kuavoController controllers)
endif()


## test 
# add_executable(csv_test ../../example/biped/csvloader_test.cc  ${SRCS})
# target_link_directories(csv_test PUBLIC ${biped_v2_link_dir})
# target_include_directories(csv_test PUBLIC ${biped_v2_include_dir})  
# target_link_libraries(csv_test pthread ${OBJS} gflags lcm drake::drake xsens_mti_driver casadi ncurses ec_master_main ec_app pbcwalk-lcm-types)

# add_executable(csv_test ../../example/biped/csv_player_test.cc  ${SRCS})
# target_link_directories(csv_test PUBLIC ${biped_v2_link_dir})
# target_include_directories(csv_test PUBLIC ${biped_v2_include_dir})  
# target_link_libraries(csv_test pthread ${OBJS} gflags lcm drake::drake xsens_mti_driver casadi ncurses ec_master_main ec_app pbcwalk-lcm-types)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/src/biped_v2)

add_executable(${PROJECT_NAME}_play_back ./biped_v2/drake_play_back.cc)
# target_link_directories(csv_test PUBLIC ${biped_v2_link_dir})
# target_include_directories(csv_test PUBLIC ${biped_v2_include_dir})  
target_link_libraries(${PROJECT_NAME}_play_back HighlyDynamicRobot_lib)

if(KUAVO_INTERNAL_CONTROLLER)
  target_link_libraries(${PROJECT_NAME}_play_back controllers)
endif()

add_executable(bin2csv ./biped_v2/logBin2Csv.cc)
target_include_directories(bin2csv PUBLIC ${biped_v2_include_dir})
target_link_directories(bin2csv PUBLIC ${biped_v2_link_dir})
target_link_libraries(bin2csv common drake::drake stdc++fs)


