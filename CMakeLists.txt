# Title     :   The testbed CMakeLists.txt
# Author    :   Jaewoong Jang
# Date      :   Jul 15th 2021
# Copyright(c) 2021 Doosan Robotics Inc. All rights reserved


cmake_minimum_required (VERSION 3.11.0)

project(testbed)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## target architecture
 if (NOT ARCH)
   set (ARCH ${CMAKE_SYSTEM_NAME})
endif (NOT ARCH)

if (NOT PROC)
  #set (PROC ${CMAKE_SYSTEM_PROCESSOR})
  set (PROC "x86")
endif (NOT PROC)

if (${PROC} MATCHES "x86_64$")
  message (FATAL_ERROR "Please check tartget processor.")
endif (${PROC} MATCHES "x86_64$")

if (${PROC} MATCHES "x86$")
  set (CMAKE_CXX_FLAGS 
    "${CMAKE_CXX_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=0 -m32 -std=c++11 -g -O2 -no-pie -fno-strict-aliasing -Wno-deprecated"
    )
elseif (${PROC} MATCHES "x86_64$")
  set (CMAKE_CXX_FLAGS 
    "${CMAKE_CXX_FLAGS} -std=c++11 -g -O2 -no-pie -fno-strict-aliasing -Wno-deprecated"
    )
endif (${PROC} MATCHES "x86$")

add_definitions(-D__XENO__)

message(STATUS "Build machine: ${CMAKE_SYSTEM_NAME}, processor: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Target machine: ${ARCH}, processor: ${PROC}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

## drfl
set(DRFL_PREFIX "${CMAKE_SOURCE_DIR}/stage/drfl")
set(DRFL_INCLUDE_DIR "${DRFL_PREFIX}/include")
set(DRFL_LIB_DIR "${DRFL_PREFIX}/lib")

add_library(drfl STATIC IMPORTED)

set_target_properties (drfl PROPERTIES
  IMPORTED_LOCATION 
  "${DRFL_LIB_DIR}/libDRFL.a")

set(DRFL_LIBS 
  drfl
  )

message(STATUS "DRFL include path: ${DRFL_INCLUDE_DIR}")
message(STATUS "DRFL library path: ${DRFL_LIB_DIR}")
message(STATUS "DRFL library     : ${DRFL_LIBS}")

## xenomai library
set(XENO_PREFIX "${CMAKE_SOURCE_DIR}/stage/xenomai")
set(XENO_INCLUDE_DIR "${XENO_PREFIX}/include")
set(XENO_LIB_DIR "${XENO_PREFIX}/lib")
set(XENO_LIBS 
  "${XENO_LIB_DIR}/libnative.so.3.0.0"
  "${XENO_LIB_DIR}/libpthread_rt.so.1.0.0"
  "${XENO_LIB_DIR}/librtdm.so.1.0.0"
  "${XENO_LIB_DIR}/libxenomai.so.0.0.0"
  )
message(STATUS "Xenomai include path: ${XENO_INCLUDE_DIR}")
message(STATUS "Xenomai library path: ${XENO_LIB_DIR}")
message(STATUS "Xenomai library     : ${XENO_LIBS}")

## poco library
set(POCO_PREFIX "${CMAKE_SOURCE_DIR}/stage/poco")
set(POCO_INCLUDE_DIR "${POCO_PREFIX}/include")
set(POCO_LIB_DIR "${POCO_PREFIX}/lib/i686")
set(POCO_LIBS 
  "${POCO_LIB_DIR}/libPocoFoundation.so"
  "${POCO_LIB_DIR}/libPocoNet.so"
  )

message(STATUS "poco include path: ${POCO_INCLUDE_DIR}")
message(STATUS "poco library path: ${POCO_LIB_DIR}")
message(STATUS "poco library     : ${POCO_LIBS}")


## boost library
set(BOOST_PREFIX "${CMAKE_SOURCE_DIR}/stage/boost")
set(BOOST_INCLUDE_DIR "${BOOST_PREFIX}/include")
set(BOOST_LIB_DIR "${BOOST_PREFIX}/lib")

add_library(boost_system STATIC IMPORTED)
add_library(boost_date_time STATIC IMPORTED)
add_library(boost_iostreams STATIC IMPORTED)

set_target_properties (boost_system PROPERTIES
  IMPORTED_LOCATION 
  "${BOOST_LIB_DIR}/libboost_system.a")
set_target_properties (boost_date_time PROPERTIES
  IMPORTED_LOCATION 
  "${BOOST_LIB_DIR}/libboost_date_time.a")
set_target_properties (boost_iostreams PROPERTIES
  IMPORTED_LOCATION 
  "${BOOST_LIB_DIR}/libboost_iostreams.a")
set(BOOST_LIBS 
  boost_system
  boost_date_time
  boost_iostreams
  )

message(STATUS "boost include path: ${POCO_INCLUDE_DIR}")
message(STATUS "boost library path: ${POCO_LIB_DIR}")
message(STATUS "boost library     : ${POCO_LIBS}")


## log4cxx library
set(LOG4CXX_PREFIX "${CMAKE_SOURCE_DIR}/stage/log4cxx")
set(LOG4CXX_INCLUDE_DIR "${LOG4CXX_PREFIX}/include")
set(LOG4CXX_LIB_DIR "${LOG4CXX_PREFIX}/lib")
add_library(log4cxx STATIC IMPORTED)
set_target_properties (log4cxx PROPERTIES
  IMPORTED_LOCATION 
  "${LOG4CXX_LIB_DIR}/liblog4cxx.a")
set(LOG4CXX_LIBS 
  log4cxx
  )
message(STATUS "log4cxx include path: ${LOG4CXX_INCLUDE_DIR}")
message(STATUS "log4cxx library path: ${LOG4CXX_LIB_DIR}")
message(STATUS "log4cxx library     : ${LOG4CXX_LIBS}")


## framwork library
set(FRAMEWORK_PREFIX "${CMAKE_SOURCE_DIR}/stage/framework")
set(FRAMEWORK_INCLUDE_DIR "${FRAMEWORK_PREFIX}/include")
set(FRAMEWORK_LIB_DIR "${FRAMEWORK_PREFIX}/lib")
set(FRAMEWORK_LIBS ${FRAMEWORK_LIBS} pthread)
message(STATUS "framwork include path: ${FRAMEWORK_INCLUDE_DIR}")
message(STATUS "framwork library path: ${FRAMEWORK_LIB_DIR}")
message(STATUS "framwork library     : ${FRAMEWORK_LIBS}")

## sub directory
add_subdirectory(src)
