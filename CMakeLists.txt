#Cmake version
cmake_minimum_required (VERSION 3.9)

#Project name
project (skelton)

## ###################################################################
## Version setup
## ###################################################################

set(${PROJECT_NAME}_VERSION_MAJOR 2)
set(${PROJECT_NAME}_VERSION_MINOR 0)
set(${PROJECT_NAME}_VERSION_PATCH 0)
set(${PROJECT_NAME}_VERSION
${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH})

## ###################################################################
## Path setup
## ###################################################################

include(GNUInstallDirs)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

## #################################################################
## Generate compilation database
## #################################################################

set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")

## ###################################################################
## Build setup
## ###################################################################

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include(GenerateExportHeader)

if(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unknown-pragmas")
endif(UNIX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WIN32)
  add_definitions("/wd4068 /wd4244 /wd4996 /wd4005 /wd4305")
endif(WIN32)


## #################################################################
## Dependencies - external
## #################################################################

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(GSL REQUIRED)
include_directories(${GSL_INCLUDE_DIRS})

# find_package(Qhull REQUIRED)
# include_directories(${QHULL_INCLUDE_DIRS})
add_library(qhullcpp STATIC IMPORTED)
set_target_properties(qhullcpp PROPERTIES
  IMPORTED_LOCATION "/usr/lib64/libqhullcpp.a"
  INTERFACE_INCLUDE_DIRECTORIES "/usr/include/libqhullcpp"
)
add_library(qhull_r SHARED IMPORTED)
set_target_properties(qhull_r PROPERTIES
  IMPORTED_LOCATION "/usr/lib64/libqhull_r.so"
  INTERFACE_INCLUDE_DIRECTORIES "/usr/include/libqhull_r"
)

# Prepare "Catch" library for other executables
set(CATCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test/catch)
add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR})


## #################################################################
## Dependencies - internal
## #################################################################

include_directories(${CMAKE_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)
# include_directories(${CMAKE_SOURCE_DIR}/src/cli)
# include_directories(${CMAKE_SOURCE_DIR}/src/field)
# include_directories(${CMAKE_SOURCE_DIR}/src/graph)
# include_directories(${CMAKE_SOURCE_DIR}/src/kernel)
# include_directories(${CMAKE_SOURCE_DIR}/src/math)
# include_directories(${CMAKE_SOURCE_DIR}/src/mesh)
# include_directories(${CMAKE_SOURCE_DIR}/src/metric)
# include_directories(${CMAKE_SOURCE_DIR}/src/reader)
# include_directories(${CMAKE_SOURCE_DIR}/src/skeleton)
# include_directories(${CMAKE_SOURCE_DIR}/src/visualization)


## #################################################################
## Input
## #################################################################

add_subdirectory (src)
# add_subdirectory (app)
add_subdirectory (test)
