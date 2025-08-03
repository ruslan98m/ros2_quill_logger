/**
 * @file quill_logger_interface.cpp
 * @brief Implementation of ROS2 logging interface integration
 * @author ROS2 Control Development Team
 * @date 2024
 * @version 1.0.0
 * @license MIT License
 */

#include "quill_logger/quill_logger_interface.hpp"
#include <cstdarg>
#include <cstring>
#include <iostream>
#include <sstream>

namespace quill_logger {

// Static member initialization
std::shared_ptr<QuillLogger> QuillLoggerInterface::logger_instance_ = nullptr;
bool QuillLoggerInterface::initialized_ = false;
rcutils_logging_output_handler_t QuillLoggerInterface::original_handler_ = nullptr;

bool QuillLoggerInterface::initialize(std::shared_ptr<QuillLogger> logger) {
  if (initialized_) {
    return true;
  }

  logger_instance_ = logger;

  try {
    // Store original handler
    original_handler_ = rcutils_logging_get_output_handler();

    // Set custom handler
    rcutils_logging_set_output_handler(ros2LogOutputHandler);

    initialized_ = true;
    return true;

  } catch (const std::exception& e) {
    std::cerr << "Failed to initialize ROS2 logging interface: " << e.what() << std::endl;
    return false;
  }
}

void QuillLoggerInterface::shutdown() {
  if (!initialized_) {
    return;
  }

  // Restore original handler
  if (original_handler_) {
    rcutils_logging_set_output_handler(original_handler_);
  }

  initialized_ = false;
  logger_instance_ = nullptr;
}

bool QuillLoggerInterface::isInitialized() {
  return initialized_;
}

void QuillLoggerInterface::ros2LogOutputHandler(const rcutils_log_location_t* location,
                                                int severity, const char* name,
                                                rcutils_time_point_value_t timestamp,
                                                const char* format, va_list* args) {
  if (!logger_instance_ || !logger_instance_->isInitialized()) {
    // Fallback to original handler if available
    if (original_handler_) {
      original_handler_(location, severity, name, timestamp, format, args);
    }
    return;
  }

  try {
    auto quill_logger = logger_instance_->getLogger();
    if (!quill_logger) {
      return;
    }

    // Convert ROS2 severity to Quill log level
    quill::LogLevel quill_level;
    switch (severity) {
      case RCUTILS_LOG_SEVERITY_DEBUG:
        quill_level = quill::LogLevel::Debug;
        break;
      case RCUTILS_LOG_SEVERITY_INFO:
        quill_level = quill::LogLevel::Info;
        break;
      case RCUTILS_LOG_SEVERITY_WARN:
        quill_level = quill::LogLevel::Warning;
        break;
      case RCUTILS_LOG_SEVERITY_ERROR:
        quill_level = quill::LogLevel::Error;
        break;
      case RCUTILS_LOG_SEVERITY_FATAL:
        quill_level = quill::LogLevel::Critical;
        break;
      default:
        quill_level = quill::LogLevel::Info;
        break;
    }

    // Format the message
    std::string formatted_msg;
    if (args && format) {
      // Use vsnprintf to format the message
      va_list args_copy;
      va_copy(args_copy, *args);

      // First, determine the required buffer size
      int size = vsnprintf(nullptr, 0, format, args_copy);
      va_end(args_copy);

      if (size > 0) {
        std::vector<char> buffer(size + 1);
        va_copy(args_copy, *args);
        vsnprintf(buffer.data(), buffer.size(), format, args_copy);
        va_end(args_copy);
        formatted_msg = std::string(buffer.data());
      }
    } else if (format) {
      formatted_msg = std::string(format);
    }

    // Create a complete message with location info
    std::stringstream full_msg;
    if (location && location->function_name) {
      full_msg << "[" << location->function_name;
      if (location->line_number > 0) {
        full_msg << ":" << location->line_number;
      }
      full_msg << "] ";
    }
    full_msg << formatted_msg;

    // Log using Quill
    switch (quill_level) {
      case quill::LogLevel::Debug:
        LOG_DEBUG(quill_logger, "{}", full_msg.str());
        break;
      case quill::LogLevel::Info:
        LOG_INFO(quill_logger, "{}", full_msg.str());
        break;
      case quill::LogLevel::Warning:
        LOG_WARNING(quill_logger, "{}", full_msg.str());
        break;
      case quill::LogLevel::Error:
        LOG_ERROR(quill_logger, "{}", full_msg.str());
        break;
      case quill::LogLevel::Critical:
        LOG_CRITICAL(quill_logger, "{}", full_msg.str());
        break;
      default:
        LOG_INFO(quill_logger, "{}", full_msg.str());
        break;
    }

  } catch (const std::exception& e) {
    // Fallback to original handler on error
    if (original_handler_) {
      original_handler_(location, severity, name, timestamp, format, args);
    }
  }
}

bool setupROS2LoggingIntegration(std::shared_ptr<QuillLogger> logger) {
  return QuillLoggerInterface::initialize(logger);
}

void teardownROS2LoggingIntegration() {
  QuillLoggerInterface::shutdown();
}

}  // namespace quill_logger
