cmake_minimum_required(VERSION 3.10)
project(BezierLib VERSION 3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置默认编译类型为Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# 针对MSVC使用项目本地的NLopt库
if(MSVC)
  add_compile_options(/utf-8)

  # 设置本地NLopt库的路径
  set(LOCAL_NLOPT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/include")
  set(LOCAL_NLOPT_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib")
  
  # 检查必要文件是否存在
  if(EXISTS "${LOCAL_NLOPT_INCLUDE_DIR}/nlopt.hpp" AND EXISTS "${LOCAL_NLOPT_LIB_DIR}/nlopt.lib")
    set(NLOPT_INCLUDE_DIRS "${LOCAL_NLOPT_INCLUDE_DIR}")
    set(NLOPT_LIBRARIES "${LOCAL_NLOPT_LIB_DIR}/nlopt.lib")
    set(NLOPT_FOUND TRUE)
    message(STATUS "Using local NLopt installation from ${LOCAL_NLOPT_LIB_DIR}")
  else()
    message(FATAL_ERROR "Local NLopt library not found in ${LOCAL_NLOPT_LIB_DIR}. Please place nlopt.hpp in lib/include and nlopt.lib in lib/")
  endif()
else()
  # 非MSVC平台使用系统的find_package
  find_package(NLopt REQUIRED)
endif()

add_definitions(-DUSE_NLOPT)

find_package(OpenMP)
add_definitions(-DUSE_OPENMP)

include_directories(include ${NLOPT_INCLUDE_DIRS})

# 定义库的源文件
set(LIB_SOURCES
    src/bezier_circle.cpp
    src/bezier_fixed.cpp
    src/bezier_quintic_fixed.cpp
    src/bezier_multi.cpp
    src/bezier_io.cpp
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
target_link_libraries(bezier PRIVATE ${NLOPT_LIBRARIES})
target_link_libraries(bezier PRIVATE OpenMP::OpenMP_CXX)

add_executable(test_circle test/test_circle.cpp)
target_link_libraries(test_circle bezier)

add_executable(test_quintic test/test_quintic.cpp)
target_link_libraries(test_quintic bezier)

add_executable(test_multi test/test_multi.cpp)
target_link_libraries(test_multi bezier) 


if(MSVC)
  # 检查DLL文件位置
  set(NLOPT_DLL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/nlopt.dll")
  if(NOT EXISTS "${NLOPT_DLL_PATH}")
    set(NLOPT_DLL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/x64/nlopt.dll")
    if(NOT EXISTS "${NLOPT_DLL_PATH}")
      message(WARNING "nlopt.dll not found in lib/ or lib/x64/. Make sure to copy it to the output directory.")
    endif()
  endif()
  
  # 如果找到了DLL，添加复制命令
  if(EXISTS "${NLOPT_DLL_PATH}")
    add_custom_command(TARGET bezier POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${NLOPT_DLL_PATH}"
      $<TARGET_FILE_DIR:bezier>)
  endif()
endif()