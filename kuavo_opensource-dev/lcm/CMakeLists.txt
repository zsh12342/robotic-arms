
find_package(lcm REQUIRED)
include(${LCM_USE_FILE})

lcm_wrap_types(
  CPP_HEADERS cpp_headers
  lcm_std_msg.lcm
  lcm_sensor_msgs.lcm
  lcm_system.lcm
  lcmt_jaco_command.lcm
  lcmt_jaco_status.lcm
)

lcm_add_library(pbcwalk-lcm-types CPP ${cpp_headers})
target_include_directories(pbcwalk-lcm-types INTERFACE ${CMAKE_CURRENT_BINARY_DIR})
