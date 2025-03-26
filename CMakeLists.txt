cmake_minimum_required(VERSION 3.10)
project(BezierLib VERSION 1.0 LANGUAGES CXX)

# 设置C++11标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置默认编译类型为Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# 检查是否使用NLopt
# option(USE_NLOPT "Use NLopt for optimization" OFF)
# if(USE_NLOPT)
#     find_package(NLopt REQUIRED)
#     add_definitions(-DUSE_NLOPT)
# endif()

find_package(NLopt REQUIRED)
add_definitions(-DUSE_NLOPT)

# 包含头文件目录
include_directories(include)

# 定义库的源文件
set(LIB_SOURCES
    src/bezier_functions.cpp
)

# 添加共享库
add_library(bezier SHARED ${LIB_SOURCES})

# 设置库的属性
set_target_properties(bezier PROPERTIES
    # VERSION ${PROJECT_VERSION}
    # SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER "include/bezier_api.hpp"
)

# 导出符号
target_compile_definitions(bezier PRIVATE BEZIER_EXPORTS)

# 链接NLopt（如果启用）
# if(USE_NLOPT)
    # target_link_libraries(bezier PRIVATE ${NLOPT_LIBRARIES})
# endif()
target_link_libraries(bezier PRIVATE ${NLOPT_LIBRARIES})


add_executable(bezier_demo src/bezier_demo.cpp src/bezier_functions.cpp)
target_link_libraries(bezier_demo ${NLOPT_LIBRARIES})

# 添加示例程序（通过链接库）
add_executable(bezier_demo_so src/bezier_demo.cpp)
target_link_libraries(bezier_demo_so PRIVATE bezier)