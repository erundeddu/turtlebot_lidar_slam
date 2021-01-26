cmake_minimum_required(VERSION 3.9)
project(rigid2d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# create the rigid2d library and add its include directories
add_library(rigid2d src/rigid2d.cpp)
target_include_directories(rigid2d PUBLIC include/)
target_compile_features(rigid2d PUBLIC cxx_std_17) # enable C++ 17
target_compile_options(rigid2d PUBLIC -Wall -Wextra)

# create the executable  and link it with the rigid2d library
add_executable(rigid2d_main src/main.cpp)
target_link_libraries(rigid2d_main rigid2d)

enable_testing()
add_executable(rigid_test tests/tests.cpp)
target_link_libraries(rigid_test rigid2d)
add_test(NAME Test_of_Rigid2d COMMAND rigid_test)

target_compile_options(rigid2d PUBLIC -Wall -Wextra -pipe)

find_package(Doxygen)
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) # Use the readme in your doxygen docs
doxygen_add_docs(doxygen src/ README.md ALL)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR
    "No in source builds allowed. Create a separate build directory.
    SOURCE_DIR=${CMAKE_SOURCE_DIR}  BINARY_DIR=${CMAKE_BINARY_DIR} ")
endif()
