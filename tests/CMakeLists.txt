cmake_minimum_required(VERSION 3.15)
project(PIDTemplates-tests CXX)

# bundled googletest — present as a submodule in extern/googletest
add_subdirectory(extern/googletest EXCLUDE_FROM_ALL)

set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(TESTSRC  ${CMAKE_CURRENT_SOURCE_DIR}/source)

add_executable(tests
    ${TESTSRC}/main.cpp
    ${TESTSRC}/test_linear_fit.cpp
    ${TESTSRC}/test_linear_fit_evenly_spaced.cpp
    ${TESTSRC}/test_IIR_PIFilter.cpp
    ${TESTSRC}/test_PIDelayIntegratorControl.cpp
    ${TESTSRC}/test_bump_test.cpp
    ${TESTSRC}/test_bump_test_fixed_point.cpp
    ${TESTSRC}/test_fit_delay_integrator.cpp
    ${TESTSRC}/test_models_DelayIntegrator.cpp
)

set_property(TARGET tests PROPERTY CXX_STANDARD 20)
target_compile_definitions(tests PRIVATE TESTING UNITTESTING LINUX DEBUG)
target_compile_options(tests PRIVATE
    -O2
    -fsanitize=address
    -fsanitize=undefined
    -fno-sanitize=signed-integer-overflow
    -fno-omit-frame-pointer
    -Wall -Wextra -Wshadow -Wconversion
    $<$<CXX_COMPILER_ID:GNU>:-Wdouble-promotion>
    $<$<CXX_COMPILER_ID:GNU>:-Wnull-dereference>
    $<$<COMPILE_LANGUAGE:CXX>:-Wold-style-cast>
)
target_link_options(tests PRIVATE -fsanitize=address -fsanitize=undefined)
target_link_libraries(tests PRIVATE gtest pthread)
target_include_directories(tests PRIVATE
    ${LIB_DIR}/source
    ${LIB_DIR}/include
    ${LIB_DIR}/libs/cnl/include
    ${TESTSRC}
)
