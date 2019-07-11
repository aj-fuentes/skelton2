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

# find_package(Eigen3 REQUIRED)
add_library(eigen3 INTERFACE IMPORTED)
set_target_properties(eigen3 PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/external/include/Eigen"
)
# include_directories()

# find_package(GSL REQUIRED)
# include_directories(${GSL_INCLUDE_DIRS})

add_library(gsl STATIC IMPORTED)
set_target_properties(gsl PROPERTIES
  IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/external/lib/libgsl.a"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/external/include/gsl"
)
add_library(gslcblas STATIC IMPORTED)
set_target_properties(gslcblas PROPERTIES
  IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/external/lib/libgslcblas.a"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/external/include/gsl"
)

add_library(qhullcpp STATIC IMPORTED)
set_target_properties(qhullcpp PROPERTIES
  IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/external/lib/libqhullcpp.a"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/external/include/libqhullcpp"
)
add_library(qhull_r STATIC IMPORTED)
set_target_properties(qhull_r PROPERTIES
  IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/external/lib/libqhullstatic_r.a"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/external/include/libqhull_r"
)

add_library(glpk STATIC IMPORTED)
set_target_properties(glpk PROPERTIES
  IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/external/lib/libglpk.a"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/external/include"
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

## #################################################################
## Input
## #################################################################

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(icesl_plugin)
