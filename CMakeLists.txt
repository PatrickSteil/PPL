cmake_minimum_required(VERSION 3.16)
project(DAGHL)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "-pipe -march=native -Wfatal-errors -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

set(CMAKE_CXX_FLAGS_RELEASE "-ffast-math -DNDEBUG -O3 -g")
set(CMAKE_CXX_FLAGS_DEBUG "-Werror -Wpedantic -pedantic-errors -Wall -Wextra -g -fno-omit-frame-pointer -O0")
set(CMAKE_CXX_FLAGS_SANITIZE "-fsanitize=address,undefined -fno-omit-frame-pointer -g -O1")
set(CMAKE_LINKER_FLAGS_SANITIZE "-fsanitize=address,undefined")

if(CMAKE_BUILD_TYPE STREQUAL "Sanitize")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_SANITIZE}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS_SANITIZE}")
    message(STATUS "Sanitizer build enabled")
endif()

add_executable(PPL main.cpp)
target_include_directories(PPL PUBLIC datastructures)

add_executable(TT_PPL timetable_main.cpp)
target_include_directories(TT_PPL PUBLIC datastructures)

enable_testing()
find_package(GTest)

if(GTest_FOUND)
    add_executable(unit_test 
        tests/bit_vector_test.cpp 
        tests/bfs_tools_test.cpp 
        tests/graph_test.cpp 
        tests/path_labels_test.cpp 
        tests/chain_test.cpp 
    )
    target_include_directories(unit_test PUBLIC datastructures)
    target_link_libraries(unit_test PRIVATE GTest::gtest_main)
    # target_compile_options(unit_test PRIVATE -fopenmp)
    add_test(NAME unit_test COMMAND unit_test)
else()
    message(STATUS "GoogleTest not found, skipping unit tests")
endif()