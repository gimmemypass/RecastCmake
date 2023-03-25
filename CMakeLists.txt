# CMakeList.txt : CMake project for RecastCMake, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.14)

set(CMAKE_CXX_STANDARD 14)

# add this options before PROJECT keyword
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

project ("RecastCMake")

#set macro-directory
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/macros")

# Set RPATH-handing (CMake parameters)
set(CMAKE_SKIP_BUILD_RPATH 0)
set(CMAKE_BUILD_WITH_INSTALL_RPATH 0)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_MACOSX_RPATH 0)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH 1)

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Lib")

message(STATUS "OS: ${CMAKE_SYSTEM_NAME}")

# build in Release-mode by default if not explicitly set
if( NOT CMAKE_BUILD_TYPE )
  set(CMAKE_BUILD_TYPE "Release")
endif()

option(CMAKE_INSTALL_PREFIX "Install location" "./Install")

if( UNIX )
    # set (CMAKE_CXX_FLAGS "-std=c++11 -pthread")
    # set (DEPENDENCIES pthread)
elseif ( WIN32 )
    # set (DEPENDENCIES ws2_32)
endif()

# Add dependencies
add_subdirectory(dep)

# Add core sources
add_subdirectory(src)

#Add tests
# add_subdirectory(tests)
