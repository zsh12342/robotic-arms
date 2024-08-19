cmake_minimum_required(VERSION 3.12)
project(ruiwoTest)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 Python 库
find_package(Python3 3.8 EXACT COMPONENTS Interpreter Development NumPy REQUIRED)
# find_package(PythonInterp REQUIRED 3.8)
if(Python3_FOUND)
    message(STATUS "Found Python: ${Python3_EXECUTABLE}")
    message(STATUS "Python version: ${Python3_VERSION}")
    
else()
    message(FATAL_ERROR "Python not found")
endif()

set(ruiwo_lib_name ruiwo_actuatorPythonLib)
add_library(${ruiwo_lib_name} SHARED ruiwo_actuator.cpp)
target_include_directories(${ruiwo_lib_name} PUBLIC ${Python3_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR})
target_link_libraries(${ruiwo_lib_name} PUBLIC Python3::Python stdc++fs -pthread)


# 添加可执行文件
add_executable(ruiwo_test ruiwo_test.cpp)
target_include_directories(ruiwo_test PRIVATE ${PROJECT_SOURCE_DIR})


# 链接 Python 库
target_link_libraries(ruiwo_test PUBLIC ${ruiwo_lib_name})
