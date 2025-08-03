/**
 * @file test_quill_logger_interface.cpp
 * @brief Unit tests for quill_logger ROS2 interface
 * @author Ruslan Mambetov
 * @date 2024
 * @version 1.0.0
 * @license MIT License
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <chrono>
#include <thread>
#include "quill_logger/quill_logger.hpp"
#include "quill_logger/quill_logger_interface.hpp"

using namespace quill_logger;
using namespace testing;

class QuillLoggerInterfaceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Clean up any existing logger instance
    if (QuillLogger::getInstance()->isInitialized()) {
      QuillLogger::getInstance()->shutdown();
    }
  }

  void TearDown() override {
    // Clean up after each test
    if (QuillLogger::getInstance()->isInitialized()) {
      QuillLogger::getInstance()->shutdown();
    }
  }
};

// Test ROS2 interface initialization
TEST_F(QuillLoggerInterfaceTest, ROS2InterfaceInitialization) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.enable_console_output = true;
  config.enable_file_output = false;
  config.log_level = "INFO";
  
  bool result = logger->initialize(config);
  EXPECT_TRUE(result);
  EXPECT_TRUE(logger->isInitialized());
  
  // Check if ROS2 interface is working
  EXPECT_TRUE(QuillLoggerInterface::isInitialized());
}

// Test ROS2 logging macros integration
TEST_F(QuillLoggerInterfaceTest, ROS2LoggingMacros) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  // Create a test node
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_node");
  
  // Test different log levels
  RCLCPP_INFO(node->get_logger(), "Test info message");
  RCLCPP_WARN(node->get_logger(), "Test warning message");
  RCLCPP_ERROR(node->get_logger(), "Test error message");
  RCLCPP_DEBUG(node->get_logger(), "Test debug message");
  
  rclcpp::shutdown();
}

// Test ROS2 logging with formatting
TEST_F(QuillLoggerInterfaceTest, ROS2LoggingWithFormatting) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_node");
  
  // Test formatted messages
  RCLCPP_INFO(node->get_logger(), "Formatted message: %d", 42);
  RCLCPP_WARN(node->get_logger(), "String: %s, Number: %f", "test", 3.14);
  RCLCPP_ERROR(node->get_logger(), "Multiple args: %d, %s, %f", 123, "hello", 2.718);
  
  rclcpp::shutdown();
}

// Test ROS2 logging level filtering
TEST_F(QuillLoggerInterfaceTest, ROS2LoggingLevelFiltering) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.log_level = "WARN";  // Only WARN and above
  config.enable_console_output = true;
  config.enable_file_output = false;
  
  logger->initialize(config);
  
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_node");
  
  // These should be filtered out (below WARN level)
  RCLCPP_DEBUG(node->get_logger(), "This debug message should be filtered");
  RCLCPP_INFO(node->get_logger(), "This info message should be filtered");
  
  // These should be logged (WARN and above)
  RCLCPP_WARN(node->get_logger(), "This warning message should be logged");
  RCLCPP_ERROR(node->get_logger(), "This error message should be logged");
  
  rclcpp::shutdown();
}

// Test ROS2 interface shutdown
TEST_F(QuillLoggerInterfaceTest, ROS2InterfaceShutdown) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  EXPECT_TRUE(QuillLoggerInterface::isInitialized());
  
  logger->shutdown();
  
  EXPECT_FALSE(QuillLoggerInterface::isInitialized());
}

// Test ROS2 logging performance
TEST_F(QuillLoggerInterfaceTest, ROS2LoggingPerformance) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  // Initialize ROS2
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  auto node = std::make_shared<rclcpp::Node>("test_node");
  
  // Measure logging performance
  const int num_messages = 1000;
  auto start = std::chrono::high_resolution_clock::now();
  
  for (int i = 0; i < num_messages; ++i) {
    RCLCPP_INFO(node->get_logger(), "Performance test message %d", i);
  }
  
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  
  // Logging should be reasonably fast (< 10ms for 1000 messages)
  EXPECT_LT(duration.count(), 10000);
  
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

// Test ROS2 logging from multiple threads
TEST_F(QuillLoggerInterfaceTest, ROS2MultiThreadedLogging) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  // Initialize ROS2
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  auto node = std::make_shared<rclcpp::Node>("test_node");
  
  std::vector<std::thread> threads;
  std::atomic<int> counter{0};
  
  // Create multiple threads that log messages
  for (int i = 0; i < 5; ++i) {
    threads.emplace_back([node, &counter, i]() {
      for (int j = 0; j < 100; ++j) {
        RCLCPP_INFO(node->get_logger(), "Thread %d, message %d", i, j);
        counter++;
      }
    });
  }
  
  // Wait for all threads to complete
  for (auto& thread : threads) {
    thread.join();
  }
  
  EXPECT_EQ(counter.load(), 500);
  
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

// Test ROS2 logging with different node names
TEST_F(QuillLoggerInterfaceTest, ROS2DifferentNodeNames) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  // Initialize ROS2
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  auto node1 = std::make_shared<rclcpp::Node>("node1");
  auto node2 = std::make_shared<rclcpp::Node>("node2");
  auto node3 = std::make_shared<rclcpp::Node>("node3");
  
  // Test logging from different nodes
  RCLCPP_INFO(node1->get_logger(), "Message from node1");
  RCLCPP_WARN(node2->get_logger(), "Message from node2");
  RCLCPP_ERROR(node3->get_logger(), "Message from node3");
  
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

// Test ROS2 logging with conditional statements
TEST_F(QuillLoggerInterfaceTest, ROS2ConditionalLogging) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  // Initialize ROS2
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  auto node = std::make_shared<rclcpp::Node>("test_node");
  
  bool condition = true;
  int value = 42;
  
  // Test conditional logging
  RCLCPP_INFO_EXPRESSION(node->get_logger(), condition, "This should be logged");
  RCLCPP_WARN_EXPRESSION(node->get_logger(), value > 40, "Value is greater than 40");
  RCLCPP_ERROR_EXPRESSION(node->get_logger(), false, "This should not be logged");
  
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

// Test ROS2 logging with once macros
TEST_F(QuillLoggerInterfaceTest, ROS2OnceLogging) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  // Initialize ROS2
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  auto node = std::make_shared<rclcpp::Node>("test_node");
  
  // Test once macros
  RCLCPP_INFO_ONCE(node->get_logger(), "This should appear only once");
  RCLCPP_WARN_ONCE(node->get_logger(), "This warning should appear only once");
  
  // Call again - should not appear
  RCLCPP_INFO_ONCE(node->get_logger(), "This should appear only once");
  RCLCPP_WARN_ONCE(node->get_logger(), "This warning should appear only once");
  
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

// Test ROS2 logging with throttled macros
TEST_F(QuillLoggerInterfaceTest, ROS2ThrottledLogging) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  // Initialize ROS2
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  auto node = std::make_shared<rclcpp::Node>("test_node");
  
  // Test throttled logging (should only log once per second)
  for (int i = 0; i < 10; ++i) {
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Throttled message %d", i);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

// Test ROS2 logging with skip first macro
TEST_F(QuillLoggerInterfaceTest, ROS2SkipFirstLogging) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  // Initialize ROS2
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  auto node = std::make_shared<rclcpp::Node>("test_node");
  
  // Test skip first logging
  RCLCPP_INFO_SKIPFIRST(node->get_logger(), "This should be skipped on first call");
  RCLCPP_INFO_SKIPFIRST(node->get_logger(), "This should appear on second call");
  
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
} 