cmake_minimum_required(VERSION 3.14)
project(paku_core_tests VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch Unity test framework
include(FetchContent)
FetchContent_Declare(
    unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG v2.5.2
)
FetchContent_MakeAvailable(unity)

# Include paths for paku_lib
set(PAKU_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../paku_core/lib/paku_lib/src)

# Arduino compatibility layer for native testing (not used in ESP32 build)
set(NATIVE_COMPAT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/native_compat)

# Source files to test
set(PAKU_LIB_SOURCES
    ${PAKU_LIB_DIR}/payload.cpp
    ${PAKU_LIB_DIR}/intervals.cpp
    ${PAKU_LIB_DIR}/flow.cpp
    ${PAKU_LIB_DIR}/ruuvi.cpp
)

# Unit test source files
set(UNIT_TEST_SOURCES
    test_payload.cpp
    test_intervals.cpp
    test_flow.cpp
)

# Create a test executable for each unit test file
foreach(test_source ${UNIT_TEST_SOURCES})
    get_filename_component(test_name ${test_source} NAME_WE)
    
    add_executable(${test_name}
        ${test_source}
        ${PAKU_LIB_SOURCES}
    )
    
    target_include_directories(${test_name} PRIVATE
        ${NATIVE_COMPAT_DIR}
        ${PAKU_LIB_DIR}
        ${unity_SOURCE_DIR}/src
    )
    
    target_link_libraries(${test_name} PRIVATE unity)
    
    # Add as CTest test
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

# Integration tests
add_executable(test_integration
    test_integration.cpp
    ${PAKU_LIB_SOURCES}
)

target_include_directories(test_integration PRIVATE
    ${NATIVE_COMPAT_DIR}
    ${PAKU_LIB_DIR}
    ${unity_SOURCE_DIR}/src
)

target_link_libraries(test_integration PRIVATE unity)

add_test(NAME test_integration COMMAND test_integration)

# Enable CTest
enable_testing()
