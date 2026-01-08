# Standard ESP-IDF boilerplate
cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "2.1.0")

# 1. Include project.cmake
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# 2. Disable all strict compiler checks that cause errors in legacy code
# These flags tell GCC to treat these issues as warnings, not errors
add_compile_options(-Wno-error)
add_compile_options(-Wno-format)
add_compile_options(-Wno-error=format)
add_compile_options(-Wno-error=unused-macros)
add_compile_options(-Wno-missing-field-initializers)
add_compile_options(-Wno-error=missing-field-initializers)

# 3. Specifically for C++ (lvgl_display.cc is a C++ file)
# Forces g++ to continue even if macros are redefined
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error -Wno-error=unused-macros -Wno-missing-field-initializers")

# 4. Global include fix for memset/memcpy
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -include string.h")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include string.h")

project(xiaozhi)