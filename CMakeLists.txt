cmake_minimum_required(VERSION 3.8)
project(quill_logger)

# Author: Ruslan Mambetov
# High-performance ROS2 logging system using Quill library

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_runtime_c REQUIRED)

# Add cmake modules path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Find Quill library
find_package(Quill REQUIRED)

# Enable automatic dependency management
ament_auto_find_build_dependencies()

# Include directories
include_directories(include)

# Create the library
add_library(${PROJECT_NAME}
  src/quill_logger.cpp
  src/quill_logger_interface.cpp
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  Quill::Quill
  ${rclcpp_LIBRARIES}
  ${rcl_LIBRARIES}
  ${rcutils_LIBRARIES}
  ${rosidl_runtime_c_LIBRARIES}
)

# Include directories for the library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rcl
  rcutils
  rosidl_runtime_c
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install configuration files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install pkg-config file
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/quill_logger.pc
  DESTINATION lib/pkgconfig
)

# Export the library
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rcl)
ament_export_dependencies(rcutils)
ament_export_dependencies(rosidl_runtime_c)

# Create a simple test node
add_executable(quill_logger_test_node
  src/test_node.cpp
)

target_link_libraries(quill_logger_test_node
  ${PROJECT_NAME}
)

ament_target_dependencies(quill_logger_test_node
  rclcpp
)

install(TARGETS quill_logger_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# Create simple node example
add_executable(simple_node_example
  examples/auto_replace_example.cpp
)

target_link_libraries(simple_node_example
  ${PROJECT_NAME}
)

ament_target_dependencies(simple_node_example
  rclcpp
)

install(TARGETS simple_node_example
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Check for gmock (optional)
  find_package(ament_cmake_gmock QUIET)
  
  # Unit tests
  ament_add_gtest(test_quill_logger
    test/test_quill_logger.cpp
  )
  
  target_link_libraries(test_quill_logger
    ${PROJECT_NAME}
  )
  
  ament_target_dependencies(test_quill_logger
    rclcpp
    rcl
    rcutils
  )
  
  # Simple tests (no ROS2 dependencies)
  ament_add_gtest(test_quill_logger_simple
    test/test_quill_logger_simple.cpp
  )
  
  target_link_libraries(test_quill_logger_simple
    ${PROJECT_NAME}
  )
  
  # ROS2 interface tests
  ament_add_gtest(test_quill_logger_interface
    test/test_quill_logger_interface.cpp
  )
  
  target_link_libraries(test_quill_logger_interface
    ${PROJECT_NAME}
  )
  
  ament_target_dependencies(test_quill_logger_interface
    rclcpp
    rcl
    rcutils
  )
  
  # Performance tests (optional - requires benchmark library)
  find_package(benchmark QUIET)
  if(benchmark_FOUND)
    ament_add_gtest(test_quill_logger_performance
      test/test_quill_logger_performance.cpp
    )
    
    target_link_libraries(test_quill_logger_performance
      ${PROJECT_NAME}
      benchmark::benchmark
    )
    
    ament_target_dependencies(test_quill_logger_performance
      rclcpp
      rcl
      rcutils
    )
    
    target_compile_definitions(test_quill_logger_performance PRIVATE BENCHMARK_FOUND)
  else()
    message(STATUS "benchmark library not found - skipping performance tests")
  endif()
endif()

ament_package() 