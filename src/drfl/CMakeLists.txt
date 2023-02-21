cmake_minimum_required (VERSION 3.11.0)

project(drfl_test)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
)

add_executable(drfl_test
  "${SRC_FILES}"
  )

target_include_directories(drfl_test
  PRIVATE "${DRFL_INCLUDE_DIR}"
  PRIVATE "${POCO_INCLUDE_DIR}"
  PRIVATE "${XENO_INCLUDE_DIR}"
  "${FRAMEWORK_INCLUDE_DIR}"
  )

target_link_libraries(drfl_test
  "${DRFL_LIBS}"
  "${POCO_LIBS}"
  "${XENO_LIBS}"
  "${FRAMEWORK_LIBS}"
  )

set_target_properties (drfl_test
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY
  "${CMAKE_BINARY_DIR}"
  COMPILE_FLAGS
  "${CMAKE_CXX_FLAGS}"
  )
