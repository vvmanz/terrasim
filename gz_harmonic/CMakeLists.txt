cmake_minimum_required(VERSION 3.16)
project(terramech_gz_harmonic CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
find_package(gz-common5 REQUIRED)
add_library(terramech_gz_system SHARED src/terramech_system.cpp)
target_include_directories(terramech_gz_system PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../core/include)
target_link_libraries(terramech_gz_system PRIVATE
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-common5::gz-common5)
