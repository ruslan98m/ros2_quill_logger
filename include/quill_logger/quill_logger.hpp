/**
 * @file quill_logger.hpp
 * @brief Main header file for the Quill-based ROS2 logger implementation
 * @author Ruslan Mambetov
 * @date 2024
 * @version 1.0.0
 * @license MIT License
 */

#ifndef QUILL_LOGGER_HPP_
#define QUILL_LOGGER_HPP_

#include <quill/Backend.h>
#include <quill/Frontend.h>
#include <quill/LogMacros.h>
#include <quill/Logger.h>
#include <quill/sinks/ConsoleSink.h>
#include <quill/sinks/FileSink.h>
#include <rcutils/logging.h>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace quill_logger {

/**
 * @brief Log levels enumeration matching ROS2 log levels
 *
 * This enumeration provides log levels that correspond to the standard ROS2 logging levels,
 * ensuring compatibility with existing ROS2 logging macros and functions.
 */
enum class LogLevel {
  DEBUG = RCUTILS_LOG_SEVERITY_DEBUG,  ///< Debug level for detailed diagnostic information
  INFO = RCUTILS_LOG_SEVERITY_INFO,    ///< Info level for general information messages
  WARN = RCUTILS_LOG_SEVERITY_WARN,    ///< Warning level for potential issues
  ERROR = RCUTILS_LOG_SEVERITY_ERROR,  ///< Error level for error conditions
  FATAL = RCUTILS_LOG_SEVERITY_FATAL   ///< Fatal level for critical errors
};

/**
 * @brief Configuration structure for the Quill logger
 *
 * This structure holds all configuration parameters for the Quill logger,
 * including file paths, log levels, and formatting options.
 */
struct LoggerConfig {
  std::string log_file_path = "";     ///< Path to the log file (empty = use ROS2 standard location)
  std::string log_level = "INFO";     ///< Default log level
  bool enable_console_output = true;  ///< Enable console output
  bool enable_file_output = true;     ///< Enable file output
  std::string log_format = "{time} [{level}] {logger} - {message}";  ///< Log message format
  size_t max_file_size = 10 * 1024 * 1024;  ///< Maximum log file size in bytes
  size_t max_files = 5;                     ///< Maximum number of log files to keep
  
  // Realtime performance settings for Backend thread
  uint64_t backend_thread_sleep_duration_ns = 100000;  ///< Backend thread sleep duration in nanoseconds (100 microseconds default)
  int backend_thread_cpu_affinity = -1;                ///< CPU affinity for backend thread (-1 = no affinity)
  int backend_thread_priority = -1;                    ///< Thread priority for backend (-1 = default priority)
  bool enable_backend_performance_mode = false;        ///< Enable high-performance mode for realtime systems
};

/**
 * @brief Main Quill logger class for ROS2 integration
 *
 * This class provides a high-performance logging implementation based on the Quill library
 * that can be used as a drop-in replacement for the default ROS2 logger. It supports
 * both console and file output, configurable log levels, and automatic log rotation.
 *
 * @example
 * ```cpp
 * #include <quill_logger/quill_logger.hpp>
 *
 * // Initialize the logger
 * auto logger = quill_logger::QuillLogger::getInstance();
 * logger->initialize();
 *
 * // Use in a ROS2 node
 * RCLCPP_INFO(logger->getLogger(), "This is an info message");
 * RCLCPP_ERROR(logger->getLogger(), "This is an error message");
 * ```
 */
class QuillLogger : public std::enable_shared_from_this<QuillLogger> {
 public:
  /**
   * @brief Get the singleton instance of the Quill logger
   *
   * @return std::shared_ptr<QuillLogger> Shared pointer to the logger instance
   */
  static std::shared_ptr<QuillLogger> getInstance();

  /**
   * @brief Initialize the logger with default configuration
   *
   * This method initializes the Quill logger with default settings and
   * sets up the necessary ROS2 logging infrastructure.
   *
   * @return true if initialization was successful, false otherwise
   */
  bool initialize();

  /**
   * @brief Initialize the logger with custom configuration
   *
   * @param config Custom configuration for the logger
   * @return true if initialization was successful, false otherwise
   */
  bool initialize(const LoggerConfig& config);

  /**
   * @brief Get the Quill logger instance for use with ROS2 logging macros
   *
   * @return quill::Logger* Pointer to the Quill logger instance
   */
  quill::Logger* getLogger();

  /**
   * @brief Set the log level for the logger
   *
   * @param level The log level to set
   */
  void setLogLevel(LogLevel level);

  /**
   * @brief Set the log level by string
   *
   * @param level_str String representation of the log level
   */
  void setLogLevel(const std::string& level_str);

  /**
   * @brief Get the current log level
   *
   * @return LogLevel Current log level
   */
  LogLevel getLogLevel() const;

  /**
   * @brief Flush all pending log messages
   *
   * This method ensures that all buffered log messages are written to their destinations.
   */
  void flush();

  /**
   * @brief Shutdown the logger
   *
   * This method properly shuts down the logger and releases all resources.
   */
  void shutdown();

  /**
   * @brief Check if the logger is initialized
   *
   * @return true if the logger is initialized, false otherwise
   */
  bool isInitialized() const;

  /**
   * @brief Public destructor
   */
  ~QuillLogger() = default;

 private:
  /**
   * @brief Private constructor for singleton pattern
   */
  QuillLogger() = default;

  /**
   * @brief Copy constructor (deleted for singleton)
   */
  QuillLogger(const QuillLogger&) = delete;

  /**
   * @brief Assignment operator (deleted for singleton)
   */
  QuillLogger& operator=(const QuillLogger&) = delete;

  /**
   * @brief Setup ROS2 logging handlers
   *
   * This method sets up the custom logging handlers that will intercept
   * ROS2 logging calls and redirect them to Quill.
   */
  void setupROS2LoggingHandlers();

  /**
   * @brief Convert ROS2 log level to Quill log level
   *
   * @param ros_level ROS2 log level
   * @return quill::LogLevel Corresponding Quill log level
   */
  static quill::LogLevel convertLogLevel(LogLevel ros_level);

  /**
   * @brief Convert string log level to LogLevel enum
   *
   * @param level_str String representation of log level
   * @return LogLevel Corresponding LogLevel enum value
   */
  static LogLevel stringToLogLevel(const std::string& level_str);

  quill::Logger* logger_ = nullptr;          ///< Quill logger instance
  LoggerConfig config_;                      ///< Logger configuration
  bool initialized_ = false;                 ///< Initialization flag
  LogLevel current_level_ = LogLevel::INFO;  ///< Current log level

  static std::shared_ptr<QuillLogger> instance_;  ///< Singleton instance
  static std::mutex instance_mutex_;              ///< Mutex for thread safety
};

/**
 * @brief Get the default logger instance
 *
 * This function provides a convenient way to get the default logger instance
 * without having to call getInstance() directly.
 *
 * @return std::shared_ptr<QuillLogger> Shared pointer to the default logger
 */
std::shared_ptr<QuillLogger> getDefaultLogger();

/**
 * @brief Initialize the global logger with default configuration
 *
 * This function initializes the global logger instance with default settings.
 * It's a convenience function that calls getInstance()->initialize().
 *
 * @return true if initialization was successful, false otherwise
 */
bool initializeGlobalLogger();

/**
 * @brief Initialize the global logger with custom configuration
 *
 * This function initializes the global logger instance with custom settings.
 * It's a convenience function that calls getInstance()->initialize(config).
 *
 * @param config Custom configuration for the logger
 * @return true if initialization was successful, false otherwise
 */
bool initializeGlobalLogger(const LoggerConfig& config);

}  // namespace quill_logger

#endif  // QUILL_LOGGER_HPP_