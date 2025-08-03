/**
 * @file test_node.cpp
 * @brief Test node demonstrating the Quill logger functionality
 * @author Ruslan Mambetov
 * @date 2024
 * @version 1.0.0
 * @license MIT License
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "quill_logger/quill_logger.hpp"

using namespace std::chrono_literals;

/**
 * @brief Test node class that demonstrates Quill logger usage
 *
 * This node shows how to use the Quill logger as a drop-in replacement
 * for the default ROS2 logger. It demonstrates various logging levels
 * and message types.
 */
class QuillLoggerTestNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the test node
   *
   * @param name Node name
   */
  QuillLoggerTestNode(const std::string& name = "quill_logger_test_node") : Node(name) {
    // Initialize the Quill logger
    auto logger = quill_logger::QuillLogger::getInstance();

    // Configure the logger with realtime optimizations
    quill_logger::LoggerConfig config;
    config.log_file_path = "";  // Use ROS2 standard log directory
    config.log_level = "DEBUG";
    config.enable_console_output = true;
    config.enable_file_output = true;
    config.log_format = "{time} [{level}] {logger} - {message}";
    
    // Realtime performance settings - DISABLED for testing
    config.enable_backend_performance_mode = true;
    config.backend_thread_sleep_duration_ns = 50000;  // 50 microseconds
    config.backend_thread_cpu_affinity = 1;           // Use CPU core 1
    config.backend_thread_priority = 80;              // High priority

    if (!logger->initialize(config)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize Quill logger");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Quill logger test node started");
    RCLCPP_INFO(this->get_logger(), "Node name: %s", this->get_name());

    // Create a timer to demonstrate logging
    timer_ = this->create_wall_timer(2s, std::bind(&QuillLoggerTestNode::timer_callback, this));

    // Log different levels
    demonstrateLogLevels();
  }

 private:
  /**
   * @brief Timer callback that demonstrates periodic logging
   */
  void timer_callback() {
    static int counter = 0;
    counter++;

    RCLCPP_INFO(this->get_logger(), "Timer callback #%d", counter);

    if (counter % 5 == 0) {
      RCLCPP_WARN(this->get_logger(), "This is a warning message (every 5th call)");
    }

    if (counter % 10 == 0) {
      RCLCPP_ERROR(this->get_logger(), "This is an error message (every 10th call)");
    }
  }

  /**
   * @brief Demonstrate different log levels and message types
   */
  void demonstrateLogLevels() {
    RCLCPP_DEBUG(this->get_logger(), "This is a debug message");
    RCLCPP_INFO(this->get_logger(), "This is an info message");
    RCLCPP_WARN(this->get_logger(), "This is a warning message");
    RCLCPP_ERROR(this->get_logger(), "This is an error message");

    // Demonstrate formatted messages
    RCLCPP_INFO(this->get_logger(), "Formatted message with number: %d", 42);
    RCLCPP_INFO(this->get_logger(), "Formatted message with string: %s", "test");
    RCLCPP_INFO(this->get_logger(), "Formatted message with multiple args: %d, %s, %.2f", 123,
                "hello", 3.14159);

    // Demonstrate different data types
    RCLCPP_INFO(this->get_logger(), "Integer: %d, Float: %.3f, String: %s", 100, 2.718, "example");

    // Demonstrate conditional logging
    bool condition = true;
    RCLCPP_INFO_EXPRESSION(this->get_logger(), condition,
                           "This message is logged because condition is true");

    RCLCPP_INFO_EXPRESSION(this->get_logger(), !condition,
                           "This message is NOT logged because condition is false");

    // Demonstrate function entry/exit logging
    RCLCPP_DEBUG(this->get_logger(), "Entering demonstrateLogLevels function");
    RCLCPP_DEBUG(this->get_logger(), "Exiting demonstrateLogLevels function");
  }

  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for periodic logging
};

/**
 * @brief Main function for the test node
 *
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return int Exit code
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<QuillLoggerTestNode>();

  RCLCPP_INFO(node->get_logger(), "Starting Quill logger test node");
  RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to stop");

  rclcpp::spin(node);

  RCLCPP_INFO(node->get_logger(), "Shutting down Quill logger test node");

  // Shutdown the Quill logger
  auto logger = quill_logger::QuillLogger::getInstance();
  logger->shutdown();

  rclcpp::shutdown();
  return 0;
}