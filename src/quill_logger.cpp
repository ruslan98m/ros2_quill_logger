/**
 * @file quill_logger.cpp
 * @brief Implementation of the Quill-based ROS2 logger
 * @author Ruslan Mambetov
 * @date 2025
 * @version 1.0.0
 * @license MIT License
 */

#include "quill_logger/quill_logger.hpp"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <pthread.h>
#include <sched.h>
#include "quill_logger/quill_logger_interface.hpp"

namespace quill_logger {

// Helper function to get ROS2 standard log directory
std::string getROS2LogDirectory() {
  // Check if ROS_LOG_DIR environment variable is set
  const char* ros_log_dir = std::getenv("ROS_LOG_DIR");
  if (ros_log_dir != nullptr) {
    return std::string(ros_log_dir);
  }

  // Use default ROS2 log directory
  const char* home_dir = std::getenv("HOME");
  if (home_dir != nullptr) {
    return std::string(home_dir) + "/.ros/log";
  }

  // Fallback to /tmp if HOME is not set
  return "/tmp";
}

// Helper function to generate timestamp for log filename
std::string generateLogTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto tm = *std::localtime(&time_t);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S");
  return oss.str();
}

// Helper function to configure Backend thread for realtime performance
void configureBackendThread(const LoggerConfig& config) {
  if (config.enable_backend_performance_mode) {
    // Set CPU affinity if specified
    if (config.backend_thread_cpu_affinity >= 0) {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(config.backend_thread_cpu_affinity, &cpuset);
      
      pthread_t current_thread = pthread_self();
      if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "Warning: Failed to set CPU affinity for backend thread" << std::endl;
      }
    }
    
    // Set thread priority if specified
    if (config.backend_thread_priority >= 0) {
      struct sched_param param;
      param.sched_priority = config.backend_thread_priority;
      
      if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        std::cerr << "Warning: Failed to set thread priority for backend thread" << std::endl;
      }
    }
  }
}

// Static member initialization
std::shared_ptr<QuillLogger> QuillLogger::instance_ = nullptr;
std::mutex QuillLogger::instance_mutex_;

std::shared_ptr<QuillLogger> QuillLogger::getInstance() {
  std::lock_guard<std::mutex> lock(instance_mutex_);
  if (!instance_) {
    instance_ = std::shared_ptr<QuillLogger>(new QuillLogger());
  }
  return instance_;
}

bool QuillLogger::initialize() {
  return initialize(LoggerConfig{});
}

bool QuillLogger::initialize(const LoggerConfig& config) {
  if (initialized_) {
    return true;
  }

  config_ = config;

  // Set default log file path to ROS2 standard location if not specified
  if (config_.log_file_path.empty()) {
    std::string log_dir = getROS2LogDirectory();
    std::string timestamp = generateLogTimestamp();
    config_.log_file_path = log_dir + "/quill_logger_" + timestamp + ".log";
  }

  try {
    // Configure backend thread for realtime performance if enabled
    configureBackendThread(config_);
    
    // Start the Quill backend with custom configuration
    if (config_.enable_backend_performance_mode) {
      // Use custom sleep duration for realtime systems
      quill::BackendOptions backend_options;
      backend_options.sleep_duration = std::chrono::nanoseconds(config_.backend_thread_sleep_duration_ns);
      quill::Backend::start(backend_options);
    } else {
      // Use default backend configuration
      quill::Backend::start();
    }

    // Create sinks
    std::vector<std::shared_ptr<quill::Sink>> sinks;

    // Add console sink if enabled
    if (config_.enable_console_output) {
      auto console_sink = std::make_shared<quill::ConsoleSink>();
      sinks.push_back(console_sink);
    }

    // Add file sink if enabled
    if (config_.enable_file_output) {
      try {
        auto file_sink = std::make_shared<quill::FileSink>(config_.log_file_path);
        sinks.push_back(file_sink);
      } catch (const std::exception& e) {
        // If file sink creation fails, fallback to console
        std::cerr << "Failed to create file sink: " << e.what() << std::endl;
        std::cerr << "Falling back to console output" << std::endl;
        config_.enable_file_output = false;
        config_.enable_console_output = true;
      }
    }

    // Create the logger
    if (sinks.empty()) {
      // If no sinks specified, use console as default
      auto console_sink = std::make_shared<quill::ConsoleSink>();
      logger_ = quill::Frontend::create_or_get_logger("ros2_quill_logger", {console_sink});
    } else {
      logger_ = quill::Frontend::create_or_get_logger("ros2_quill_logger", sinks);
    }

    // Set log level
    quill::LogLevel quill_level = convertLogLevel(stringToLogLevel(config_.log_level));
    logger_->set_log_level(quill_level);

    // Setup ROS2 logging integration
    if (!QuillLoggerInterface::initialize(shared_from_this())) {
      std::cerr << "Failed to setup ROS2 logging integration" << std::endl;
      return false;
    }

    initialized_ = true;
    current_level_ = stringToLogLevel(config_.log_level);

    // Log initialization message
    LOG_INFO(logger_, "Quill logger initialized successfully");
    LOG_INFO(logger_, "Log file: {}", config_.log_file_path);
    LOG_INFO(logger_, "Log level: {}", config_.log_level);
    
    // Log backend configuration if performance mode is enabled
    if (config_.enable_backend_performance_mode) {
      LOG_INFO(logger_, "Backend performance mode enabled");
      LOG_INFO(logger_, "Backend sleep duration: {} ns", config_.backend_thread_sleep_duration_ns);
      if (config_.backend_thread_cpu_affinity >= 0) {
        LOG_INFO(logger_, "Backend CPU affinity: {}", config_.backend_thread_cpu_affinity);
      }
      if (config_.backend_thread_priority >= 0) {
        LOG_INFO(logger_, "Backend thread priority: {}", config_.backend_thread_priority);
      }
    }

    return true;

  } catch (const std::exception& e) {
    std::cerr << "Failed to initialize Quill logger: " << e.what() << std::endl;
    return false;
  }
}

quill::Logger* QuillLogger::getLogger() {
  return logger_;
}

void QuillLogger::setLogLevel(LogLevel level) {
  if (!initialized_ || !logger_) {
    return;
  }

  quill::LogLevel quill_level = convertLogLevel(level);
  logger_->set_log_level(quill_level);
  current_level_ = level;

  LOG_INFO(logger_, "Log level changed to: {}", static_cast<int>(level));
}

void QuillLogger::setLogLevel(const std::string& level_str) {
  LogLevel level = stringToLogLevel(level_str);
  setLogLevel(level);
}

LogLevel QuillLogger::getLogLevel() const {
  return current_level_;
}

void QuillLogger::flush() {
  if (initialized_ && logger_) {
    // Quill automatically flushes, but we can force a flush if needed
    // For now, we'll just return as Quill handles this automatically
  }
}

void QuillLogger::shutdown() {
  if (!initialized_) {
    return;
  }

  LOG_INFO(logger_, "Shutting down Quill logger");

  // Shutdown ROS2 logging integration
  QuillLoggerInterface::shutdown();

  // Reset logger pointer
  logger_ = nullptr;
  initialized_ = false;
}

bool QuillLogger::isInitialized() const {
  return initialized_;
}

void QuillLogger::setupROS2LoggingHandlers() {
  // This is handled by QuillLoggerInterface
}

quill::LogLevel QuillLogger::convertLogLevel(LogLevel ros_level) {
  switch (ros_level) {
    case LogLevel::DEBUG:
      return quill::LogLevel::Debug;
    case LogLevel::INFO:
      return quill::LogLevel::Info;
    case LogLevel::WARN:
      return quill::LogLevel::Warning;
    case LogLevel::ERROR:
      return quill::LogLevel::Error;
    case LogLevel::FATAL:
      return quill::LogLevel::Critical;
    default:
      return quill::LogLevel::Info;
  }
}

LogLevel QuillLogger::stringToLogLevel(const std::string& level_str) {
  std::string upper_level = level_str;
  std::transform(upper_level.begin(), upper_level.end(), upper_level.begin(), ::toupper);

  if (upper_level == "DEBUG")
    return LogLevel::DEBUG;
  if (upper_level == "INFO")
    return LogLevel::INFO;
  if (upper_level == "WARN" || upper_level == "WARNING")
    return LogLevel::WARN;
  if (upper_level == "ERROR")
    return LogLevel::ERROR;
  if (upper_level == "FATAL" || upper_level == "CRITICAL")
    return LogLevel::FATAL;

  // Default to INFO if unknown
  return LogLevel::INFO;
}

std::shared_ptr<QuillLogger> getDefaultLogger() {
  return QuillLogger::getInstance();
}

bool initializeGlobalLogger() {
  return getDefaultLogger()->initialize();
}

bool initializeGlobalLogger(const LoggerConfig& config) {
  return getDefaultLogger()->initialize(config);
}

}  // namespace quill_logger