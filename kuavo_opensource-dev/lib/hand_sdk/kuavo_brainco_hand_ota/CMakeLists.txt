cmake_minimum_required(VERSION 3.10)
project(SerialPortProject)

# 设置变量
set(SCRIPT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(DIST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/dist)
set(DEP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/libuv)

# 设置编译器和选项
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall ${LIBSERIALPORT_CFLAGS_OTHER}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -std=c++11 ${LIBSERIALPORT_CFLAGS_OTHER}")
set(LIBS ${LIBSERIALPORT_LIBRARIES} -rdynamic)

# 查找所有的源文件
file(GLOB SOURCES *.c *.cpp)

# 添加头文件路径
include_directories(${DIST_DIR}/include ${DEP_DIR}/include)
link_directories(${DIST_DIR}/linux/static)

# 为每个源文件生成一个可执行文件
foreach(SOURCE ${SOURCES})
    get_filename_component(EXECUTABLE ${SOURCE} NAME_WE)
    add_executable(${EXECUTABLE} ${SOURCE})
    target_link_libraries(${EXECUTABLE} stark m pthread rt dl ${LIBS})
endforeach()

# 自定义目标：build_all
add_custom_target(build_all DEPENDS ${SOURCES})

# 自定义目标：clean_all
add_custom_target(clean_all
    COMMAND ${CMAKE_COMMAND} -E remove -f ${CMAKE_BINARY_DIR}/*
)

# 自定义目标：run_serial
add_custom_target(run_serial
    COMMAND ${CMAKE_COMMAND} -E echo "Running rs485"
    COMMAND ${CMAKE_BINARY_DIR}/rs485 ${serial_port_name}
    DEPENDS rs485
)

# 自定义目标：run_ota
add_custom_target(run_ota
    COMMAND ${CMAKE_COMMAND} -E echo "Running ota"
    COMMAND ${CMAKE_BINARY_DIR}/ota ${serial_port_name}
    DEPENDS ota
)

# 伪目标声明，确保不会与同名文件冲突
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_BINARY_DIR}/*")
