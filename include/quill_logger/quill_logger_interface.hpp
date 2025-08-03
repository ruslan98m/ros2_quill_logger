/**
 * @file quill_logger_interface.hpp
 * @brief Interface for integrating Quill logger with ROS2 logging system
 * @author Ruslan Mambetov
 * @date 2025
 * @version 1.0.0
 * @license MIT License
 */

#ifndef QUILL_LOGGER_INTERFACE_HPP_
#define QUILL_LOGGER_INTERFACE_HPP_

#include <rcutils/logging.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "quill_logger.hpp"

namespace quill_logger {

/**
 * @brief Interface class for ROS2 logging integration
 *
 * This class provides the necessary interface functions to integrate the Quill logger
 * with the ROS2 logging system. It implements the required callback functions that
 * ROS2 uses for logging and redirects them to the Quill logger.
 *
 * @example
 * ```cpp
 * #include <quill_logger/quill_logger_interface.hpp>
 *
 * // The interface is automatically set up when the logger is initialized
 * auto logger = quill_logger::QuillLogger::getInstance();
 * logger->initialize();
 *
 * // Now all ROS2 logging calls will use Quill
 * RCLCPP_INFO(rclcpp::get_logger("my_node"), "This goes through Quill");
 * ```
 */
class QuillLoggerInterface {
 public:
  /**
   * @brief Initialize the ROS2 logging interface
   *
   * This method sets up the custom logging handlers that will intercept
   * all ROS2 logging calls and redirect them to the Quill logger.
   *
   * @param logger Pointer to the Quill logger instance
   * @return true if initialization was successful, false otherwise
   */
  static bool initialize(std::shared_ptr<QuillLogger> logger);

  /**
   * @brief Shutdown the ROS2 logging interface
   *
   * This method restores the original ROS2 logging handlers and
   * cleans up the interface resources.
   */
  static void shutdown();

  /**
   * @brief Check if the interface is initialized
   *
   * @return true if the interface is initialized, false otherwise
   */
  static bool isInitialized();

 private:
  /**
   * @brief Custom logging output handler for ROS2
   *
   * This function is called by ROS2 when a log message needs to be output.
   * It redirects the message to the Quill logger.
   *
   * @param location Source location information
   * @param severity Log severity level
   * @param name Logger name
   * @param timestamp Timestamp of the log message
   * @param format Format string
   * @param args Variable arguments for formatting
   */
  static void ros2LogOutputHandler(const rcutils_log_location_t* location, int severity,
                                   const char* name, rcutils_time_point_value_t timestamp,
                                   const char* format, va_list* args);

  static std::shared_ptr<QuillLogger> logger_instance_;       ///< Reference to the logger instance
  static bool initialized_;                                   ///< Initialization flag
  static rcutils_logging_output_handler_t original_handler_;  ///< Original ROS2 logging handler
};

/**
 * @brief Setup ROS2 logging integration
 *
 * This function provides a convenient way to setup the ROS2 logging integration
 * without having to call the interface directly.
 *
 * @param logger Pointer to the Quill logger instance
 * @return true if setup was successful, false otherwise
 */
bool setupROS2LoggingIntegration(std::shared_ptr<QuillLogger> logger);

/**
 * @brief Teardown ROS2 logging integration
 *
 * This function provides a convenient way to teardown the ROS2 logging integration
 * without having to call the interface directly.
 */
void teardownROS2LoggingIntegration();

}  // namespace quill_logger

#endif  // QUILL_LOGGER_INTERFACE_HPP_