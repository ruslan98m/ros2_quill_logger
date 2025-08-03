/**
 * @file test_quill_logger.cpp
 * @brief Unit tests for quill_logger
 * @author Ruslan Mambetov
 * @date 2024
 * @version 1.0.0
 * @license MIT License
 */

#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include "quill_logger/quill_logger.hpp"

using namespace quill_logger;
using namespace testing;

class QuillLoggerTest : public ::testing::Test {
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

  std::string getLogFileContent(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
      return "";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
  }

  bool fileExists(const std::string& filepath) {
    std::ifstream file(filepath);
    return file.good();
  }
};

// Test singleton pattern
TEST_F(QuillLoggerTest, SingletonPattern) {
  auto instance1 = QuillLogger::getInstance();
  auto instance2 = QuillLogger::getInstance();
  
  EXPECT_EQ(instance1, instance2);
  EXPECT_NE(instance1, nullptr);
  EXPECT_NE(instance2, nullptr);
}

// Test default initialization
TEST_F(QuillLoggerTest, DefaultInitialization) {
  auto logger = QuillLogger::getInstance();
  
  EXPECT_FALSE(logger->isInitialized());
  
  bool result = logger->initialize();
  EXPECT_TRUE(result);
  EXPECT_TRUE(logger->isInitialized());
}

// Test custom configuration initialization
TEST_F(QuillLoggerTest, CustomConfigurationInitialization) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.log_level = "WARN";
  config.enable_console_output = true;
  config.enable_file_output = false;
  config.log_file_path = "/tmp/test_quill_logger.log";
  
  bool result = logger->initialize(config);
  EXPECT_TRUE(result);
  EXPECT_TRUE(logger->isInitialized());
  EXPECT_EQ(logger->getLogLevel(), LogLevel::WARN);
}

// Test log level setting
TEST_F(QuillLoggerTest, LogLevelSetting) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  // Test setting by enum
  logger->setLogLevel(LogLevel::ERROR);
  EXPECT_EQ(logger->getLogLevel(), LogLevel::ERROR);
  
  // Test setting by string
  logger->setLogLevel("DEBUG");
  EXPECT_EQ(logger->getLogLevel(), LogLevel::DEBUG);
  
  logger->setLogLevel("INFO");
  EXPECT_EQ(logger->getLogLevel(), LogLevel::INFO);
  
  logger->setLogLevel("WARN");
  EXPECT_EQ(logger->getLogLevel(), LogLevel::WARN);
  
  logger->setLogLevel("ERROR");
  EXPECT_EQ(logger->getLogLevel(), LogLevel::ERROR);
  
  logger->setLogLevel("FATAL");
  EXPECT_EQ(logger->getLogLevel(), LogLevel::FATAL);
}

// Test invalid log level string
TEST_F(QuillLoggerTest, InvalidLogLevelString) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  // Should default to INFO for invalid strings
  logger->setLogLevel("INVALID_LEVEL");
  EXPECT_EQ(logger->getLogLevel(), LogLevel::INFO);
}

// Test file output
TEST_F(QuillLoggerTest, FileOutput) {
  std::string test_log_file = "/tmp/test_file_output.log";
  
  // Remove existing file
  std::remove(test_log_file.c_str());
  
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.log_file_path = test_log_file;
  config.enable_console_output = false;
  config.enable_file_output = true;
  config.log_level = "INFO";
  
  bool result = logger->initialize(config);
  EXPECT_TRUE(result);
  
  // Give some time for backend to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Check if file was created
  EXPECT_TRUE(fileExists(test_log_file));
  
  // Clean up
  std::remove(test_log_file.c_str());
}

// Test console output only
TEST_F(QuillLoggerTest, ConsoleOutputOnly) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.enable_console_output = true;
  config.enable_file_output = false;
  config.log_level = "INFO";
  
  bool result = logger->initialize(config);
  EXPECT_TRUE(result);
  
  // Should not create a log file
  EXPECT_FALSE(fileExists(config.log_file_path));
}

// Test realtime performance mode
TEST_F(QuillLoggerTest, RealtimePerformanceMode) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.enable_backend_performance_mode = true;
  config.backend_thread_sleep_duration_ns = 50000;
  config.backend_thread_cpu_affinity = 1;
  config.backend_thread_priority = 80;
  config.enable_file_output = false;
  config.log_level = "WARN";
  
  bool result = logger->initialize(config);
  EXPECT_TRUE(result);
  EXPECT_TRUE(logger->isInitialized());
}

// Test multiple initializations
TEST_F(QuillLoggerTest, MultipleInitializations) {
  auto logger = QuillLogger::getInstance();
  
  // First initialization
  bool result1 = logger->initialize();
  EXPECT_TRUE(result1);
  EXPECT_TRUE(logger->isInitialized());
  
  // Second initialization should return true (already initialized)
  bool result2 = logger->initialize();
  EXPECT_TRUE(result2);
  EXPECT_TRUE(logger->isInitialized());
}

// Test shutdown
TEST_F(QuillLoggerTest, Shutdown) {
  auto logger = QuillLogger::getInstance();
  
  logger->initialize();
  EXPECT_TRUE(logger->isInitialized());
  
  logger->shutdown();
  EXPECT_FALSE(logger->isInitialized());
}

// Test getLogger method
TEST_F(QuillLoggerTest, GetLogger) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  auto quill_logger = logger->getLogger();
  EXPECT_NE(quill_logger, nullptr);
}

// Test flush method
TEST_F(QuillLoggerTest, Flush) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  // Flush should not throw
  EXPECT_NO_THROW(logger->flush());
}

// Test log level conversion
TEST_F(QuillLoggerTest, LogLevelConversion) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  // Test all log level conversions
  EXPECT_EQ(logger->getLogLevel(), LogLevel::INFO);
  
  logger->setLogLevel(LogLevel::DEBUG);
  EXPECT_EQ(logger->getLogLevel(), LogLevel::DEBUG);
  
  logger->setLogLevel(LogLevel::WARN);
  EXPECT_EQ(logger->getLogLevel(), LogLevel::WARN);
  
  logger->setLogLevel(LogLevel::ERROR);
  EXPECT_EQ(logger->getLogLevel(), LogLevel::ERROR);
  
  logger->setLogLevel(LogLevel::FATAL);
  EXPECT_EQ(logger->getLogLevel(), LogLevel::FATAL);
}

// Test configuration persistence
TEST_F(QuillLoggerTest, ConfigurationPersistence) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.log_level = "ERROR";
  config.enable_console_output = true;
  config.enable_file_output = false;
  config.enable_backend_performance_mode = true;
  config.backend_thread_sleep_duration_ns = 25000;
  
  logger->initialize(config);
  
  // Configuration should be preserved
  EXPECT_EQ(logger->getLogLevel(), LogLevel::ERROR);
}

// Test thread safety
TEST_F(QuillLoggerTest, ThreadSafety) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  std::vector<std::thread> threads;
  std::atomic<int> counter{0};
  
  // Create multiple threads that access the logger
  for (int i = 0; i < 10; ++i) {
    threads.emplace_back([logger, &counter]() {
      for (int j = 0; j < 100; ++j) {
        logger->setLogLevel(LogLevel::INFO);
        logger->getLogLevel();
        counter++;
      }
    });
  }
  
  // Wait for all threads to complete
  for (auto& thread : threads) {
    thread.join();
  }
  
  EXPECT_EQ(counter.load(), 1000);
}

// Test performance - measure initialization time
TEST_F(QuillLoggerTest, InitializationPerformance) {
  auto logger = QuillLogger::getInstance();
  
  auto start = std::chrono::high_resolution_clock::now();
  bool result = logger->initialize();
  auto end = std::chrono::high_resolution_clock::now();
  
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  
  EXPECT_TRUE(result);
  // Initialization should be fast (< 10ms)
  EXPECT_LT(duration.count(), 10000);
}

// Test error handling - invalid file path
TEST_F(QuillLoggerTest, InvalidFilePath) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.log_file_path = "/invalid/path/test.log";
  config.enable_file_output = true;
  config.enable_console_output = false;
  
  // Should still initialize successfully (fallback to console)
  bool result = logger->initialize(config);
  EXPECT_TRUE(result);
  EXPECT_TRUE(logger->isInitialized());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
} 