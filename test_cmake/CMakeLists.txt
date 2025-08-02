cmake_minimum_required(VERSION 3.8)
project(quill_test)

# Add cmake modules path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

# Find Quill library
find_package(Quill REQUIRED)

# Create test executable
add_executable(quill_simple_test
  ../examples/simple_test.cpp
)

# Link libraries
target_link_libraries(quill_simple_test
  Quill::Quill
)

# Set C++ standard
target_compile_features(quill_simple_test PRIVATE cxx_std_17) 