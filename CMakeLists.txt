cmake_minimum_required(VERSION 3.23)

project(FlightControl VERSION 1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(lqr_test)

file(GLOB_RECURSE LQR_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/lqr/*"
)

target_sources(lqr_test
  PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/tests/lqr_test.c"
  ${LQR_SOURCES}
)
