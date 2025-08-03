/**
 * @file test_quill_logger_simple.cpp
 * @brief Simple unit tests for quill_logger without external dependencies
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

class QuillLoggerSimpleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (QuillLogger::getInstance()->isInitialized()) {
      QuillLogger::getInstance()->shutdown();
    }
  }

  void TearDown() override {
    if (QuillLogger::getInstance()->isInitialized()) {
      QuillLogger::getInstance()->shutdown();
    }
  }

  bool fileExists(const std::string& filepath) {
    std::ifstream file(filepath);
    return file.good();
  }
};

// Test basic functionality
TEST_F(QuillLoggerSimpleTest, BasicFunctionality) {
  auto logger = QuillLogger::getInstance();
  
  EXPECT_FALSE(logger->isInitialized());
  
  bool result = logger->initialize();
  EXPECT_TRUE(result);
  EXPECT_TRUE(logger->isInitialized());
  
  logger->shutdown();
  EXPECT_FALSE(logger->isInitialized());
}

// Test configuration
TEST_F(QuillLoggerSimpleTest, Configuration) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.log_level = "WARN";
  config.enable_console_output = true;
  config.enable_file_output = false;
  
  bool result = logger->initialize(config);
  EXPECT_TRUE(result);
  EXPECT_EQ(logger->getLogLevel(), LogLevel::WARN);
}

// Test log level changes
TEST_F(QuillLoggerSimpleTest, LogLevelChanges) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  logger->setLogLevel(LogLevel::ERROR);
  EXPECT_EQ(logger->getLogLevel(), LogLevel::ERROR);
  
  logger->setLogLevel("DEBUG");
  EXPECT_EQ(logger->getLogLevel(), LogLevel::DEBUG);
}

// Test realtime configuration
TEST_F(QuillLoggerSimpleTest, RealtimeConfiguration) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.enable_backend_performance_mode = true;
  config.backend_thread_sleep_duration_ns = 50000;
  config.backend_thread_cpu_affinity = 1;
  config.backend_thread_priority = 80;
  config.enable_file_output = false;
  
  bool result = logger->initialize(config);
  EXPECT_TRUE(result);
  EXPECT_TRUE(logger->isInitialized());
}

// Test performance - simple timing
TEST_F(QuillLoggerSimpleTest, SimplePerformance) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  auto start = std::chrono::high_resolution_clock::now();
  
  // Perform some operations
  logger->setLogLevel(LogLevel::INFO);
  logger->getLogLevel();
  logger->setLogLevel(LogLevel::WARN);
  logger->getLogLevel();
  
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  
  // Should be very fast
  EXPECT_LT(duration.count(), 1000);  // < 1ms
}

// Test thread safety
TEST_F(QuillLoggerSimpleTest, ThreadSafety) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  std::vector<std::thread> threads;
  std::atomic<int> counter{0};
  
  for (int i = 0; i < 5; ++i) {
    threads.emplace_back([logger, &counter]() {
      for (int j = 0; j < 100; ++j) {
        logger->setLogLevel(LogLevel::INFO);
        logger->getLogLevel();
        counter++;
      }
    });
  }
  
  for (auto& thread : threads) {
    thread.join();
  }
  
  EXPECT_EQ(counter.load(), 500);
}

// Test multiple initializations
TEST_F(QuillLoggerSimpleTest, MultipleInitializations) {
  auto logger = QuillLogger::getInstance();
  
  bool result1 = logger->initialize();
  EXPECT_TRUE(result1);
  
  bool result2 = logger->initialize();
  EXPECT_TRUE(result2);  // Should return true for already initialized
}

// Test invalid log level
TEST_F(QuillLoggerSimpleTest, InvalidLogLevel) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  logger->setLogLevel("INVALID_LEVEL");
  EXPECT_EQ(logger->getLogLevel(), LogLevel::INFO);  // Should default to INFO
}

// Test getLogger method
TEST_F(QuillLoggerSimpleTest, GetLogger) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  auto quill_logger = logger->getLogger();
  EXPECT_NE(quill_logger, nullptr);
}

// Test flush method
TEST_F(QuillLoggerSimpleTest, Flush) {
  auto logger = QuillLogger::getInstance();
  logger->initialize();
  
  EXPECT_NO_THROW(logger->flush());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
} 