cmake_minimum_required(VERSION 3.10.0)
project(tests)



add_executable(tests)

set(TargetName tests)

set(CMAKE_CXX_STANDARD 20)
target_compile_features(${TargetName} PUBLIC cxx_std_20)
set_property(TARGET ${TargetName} PROPERTY CXX_STANDARD 20)
#SET(CMAKE_CXX_COMPILER clang++-8)
#SET(CMAKE_CXX_COMPILER /usr/bin/g++-8)
get_filename_component(TESTSRC source ABSOLUTE)

#fixme this should have all the sources
aux_source_directory(${TESTSRC} DIR_SRCS)

target_sources(tests PUBLIC ${DIR_SRCS})
if(FALSE)
  target_link_libraries(${TargetName} asan)
target_link_libraries(${TargetName} tsan)
target_link_libraries(${TargetName} ubsan)
target_link_libraries(${TargetName} msan)
endif()

include_directories("/usr/src")
include_directories("/usr/include")
#find_library(googletest REQUIRED PATHS "/usr/src")
#find_package(gtest REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/extern/googletest/googletest/generated")
set(gtest_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extern/googletest/googletest")
include_directories(${gtest_DIR})
include_directories(${gtest_DIR}/include)
include_directories(${gtest_DIR}/include)

set(cnl_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs/cnl)
target_include_directories(${TargetName} PUBLIC ${cnl_DIR}/include)
include_directories(${cnl_DIR}/include)

target_include_directories(${TargetName} PUBLIC "${gtest_DIR}/include")
target_include_directories(tests PUBLIC ${TESTSRC} )
target_include_directories(tests PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(tests PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/PIDTemplates)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include/PIDTemplates/models)
target_link_directories(${TargetName} PUBLIC ${gtest_DIR}/../lib)
target_link_libraries(${TargetName} gtest)
target_link_libraries(${TargetName} pthread)
target_compile_options(
  ${TargetName}
  PRIVATE
  -fconcepts
  -ggdb
  )

message(STATUS "Sources: ${DIR_SRCS}")

add_test(NAME ${CMAKE_SOURCE_DIR} COMMAND tests)
