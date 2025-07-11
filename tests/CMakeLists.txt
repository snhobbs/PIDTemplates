cmake_minimum_required(VERSION 3.10.0)
SET(CMAKE_VERBOSE_MAKEFILE ON)
project(tests)
set(TargetName ${CMAKE_PROJECT_NAME})

set(ProjectDirectory "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(LibraryDirectory ${ProjectDirectory}/libs)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON )
add_executable(${TargetName})

# Add test sources
set(TESTSRC ${CMAKE_CURRENT_SOURCE_DIR}/source)
get_filename_component(TESTSRC source ABSOLUTE)
#aux_source_directory(${TESTSRC} DIR_SRCS)
#target_sources(${TargetName} PUBLIC ${DIR_SRCS})
# Sources
target_sources(${TargetName} PUBLIC
    ${TESTSRC}/main.cpp
    ${TESTSRC}/test_gsl_linear_fit_evenly_spaced.cpp
    ${TESTSRC}/test_gsl_linear_fit.cpp
    ${TESTSRC}/test_IIR_PIFilter.cpp
    # Add more test sources as needed
)

target_include_directories(${TargetName} PUBLIC ${ProjectDirectory}/source)
target_include_directories(${TargetName} PUBLIC ${ProjectDirectory}/include)
target_include_directories(${TargetName} PUBLIC ${TESTSRC} )

# Target Options
set(CMAKE_CXX_STANDARD 20)
target_compile_features(${TargetName} PUBLIC cxx_std_20)
set_property(TARGET ${TargetName} PROPERTY CXX_STANDARD 20)
#SET(CMAKE_CXX_COMPILER clang++-8)
#SET(CMAKE_CXX_COMPILER /usr/bin/g++-8)
target_compile_definitions(${TargetName} PRIVATE TESTING)
target_compile_definitions(${TargetName} PRIVATE UNITTESTING)
target_compile_definitions(${TargetName} PRIVATE LINUX)
target_compile_definitions(${TargetName} PRIVATE DEBUG)

#fixme this should have all the sources

if(FALSE)
  target_link_libraries(${TargetName} asan)
target_link_libraries(${TargetName} tsan)
target_link_libraries(${TargetName} ubsan)
target_link_libraries(${TargetName} msan)
endif()

# System Includes to keep eclipse happy
include_directories("/usr/src")
#include_directories("/usr/include")
include_directories(SYSTEM /usr/lib)
include_directories(SYSTEM /home/simon/local/include)
include_directories(SYSTEM /usr/include)

# GTEST
#find_library(googletest REQUIRED PATHS "/usr/src")
#find_package(gtest REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/extern/googletest/googletest/generated")
set(gtest_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extern/googletest/googletest")
target_include_directories(${TargetName} PUBLIC "${gtest_DIR}/include")
target_link_directories(${TargetName} PUBLIC ${gtest_DIR})
target_link_directories(${TargetName} PUBLIC ${gtest_DIR}/../lib)
target_link_libraries(${TargetName} gtest)
target_link_libraries(${TargetName} pthread)

# Dependencies
## cnl
set(cnl_DIR ${ProjectDirectory}/libs/cnl)
target_include_directories(${TargetName} PUBLIC ${cnl_DIR}/include)
include_directories(${cnl_DIR}/include)

## PIDTemplates
set(PIDTemplates_DIR ${LibraryDirectory}/PIDTemplates)
include_directories(${PIDTemplates_DIR}/include)
target_include_directories(${TargetName} PUBLIC ${PIDTemplates_DIR}/include)
#target_include_directories(${TargetName} PUBLIC ${PIDTemplates_DIR}/models)
target_compile_options(
  ${TargetName}
  PRIVATE
  -fconcepts
  -ggdb
  )

message(STATUS "Sources: ${DIR_SRCS}")

add_test(NAME ${CMAKE_SOURCE_DIR} COMMAND tests)
