/**
 * @file test_quill_logger_performance.cpp
 * @brief Performance tests for quill_logger
 * @author Ruslan Mambetov
 * @date 2025
 * @version 1.0.0
 * @license MIT License
 */

#include <gtest/gtest.h>
#ifdef BENCHMARK_FOUND
#include <benchmark/benchmark.h>
#endif
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <vector>
#include <atomic>
#include "quill_logger/quill_logger.hpp"

using namespace quill_logger;

class QuillLoggerPerformanceTest : public ::testing::Test {
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
};

#ifdef BENCHMARK_FOUND
// Benchmark: Basic logging performance
static void BM_BasicLogging(benchmark::State& state) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.enable_console_output = false;
  config.enable_file_output = false;
  config.log_level = "INFO";
  
  logger->initialize(config);
  
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("benchmark_node");
  
  for (auto _ : state) {
    RCLCPP_INFO(node->get_logger(), "Benchmark message");
  }
  
  rclcpp::shutdown();
}

// Benchmark: Formatted logging performance
static void BM_FormattedLogging(benchmark::State& state) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.enable_console_output = false;
  config.enable_file_output = false;
  config.log_level = "INFO";
  
  logger->initialize(config);
  
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("benchmark_node");
  
  int counter = 0;
  for (auto _ : state) {
    RCLCPP_INFO(node->get_logger(), "Formatted message: %d, %s, %f", counter, "test", 3.14);
    counter++;
  }
  
  rclcpp::shutdown();
}

// Benchmark: Realtime mode logging performance
static void BM_RealtimeLogging(benchmark::State& state) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.enable_backend_performance_mode = true;
  config.backend_thread_sleep_duration_ns = 25000;
  config.enable_console_output = false;
  config.enable_file_output = false;
  config.log_level = "WARN";
  
  logger->initialize(config);
  
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("benchmark_node");
  
  for (auto _ : state) {
    RCLCPP_WARN(node->get_logger(), "Realtime benchmark message");
  }
  
  rclcpp::shutdown();
}

// Benchmark: Multi-threaded logging performance
static void BM_MultiThreadedLogging(benchmark::State& state) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.enable_console_output = false;
  config.enable_file_output = false;
  config.log_level = "INFO";
  
  logger->initialize(config);
  
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("benchmark_node");
  
  std::atomic<int> counter{0};
  
  for (auto _ : state) {
    std::vector<std::thread> threads;
    
    for (int i = 0; i < 4; ++i) {
      threads.emplace_back([node, &counter, i]() {
        for (int j = 0; j < 100; ++j) {
          RCLCPP_INFO(node->get_logger(), "Thread %d, message %d", i, j);
          counter++;
        }
      });
    }
    
    for (auto& thread : threads) {
      thread.join();
    }
  }
  
  rclcpp::shutdown();
}

// Benchmark: Initialization performance
static void BM_Initialization(benchmark::State& state) {
  for (auto _ : state) {
    auto logger = QuillLogger::getInstance();
    
    LoggerConfig config;
    config.enable_console_output = false;
    config.enable_file_output = false;
    config.log_level = "INFO";
    
    logger->initialize(config);
    logger->shutdown();
  }
}

// Benchmark: Log level changes performance
static void BM_LogLevelChanges(benchmark::State& state) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.enable_console_output = false;
  config.enable_file_output = false;
  config.log_level = "INFO";
  
  logger->initialize(config);
  
  for (auto _ : state) {
    logger->setLogLevel(LogLevel::DEBUG);
    logger->setLogLevel(LogLevel::INFO);
    logger->setLogLevel(LogLevel::WARN);
    logger->setLogLevel(LogLevel::ERROR);
    logger->setLogLevel(LogLevel::FATAL);
  }
}

// Register benchmarks
BENCHMARK(BM_BasicLogging);
BENCHMARK(BM_FormattedLogging);
BENCHMARK(BM_RealtimeLogging);
BENCHMARK(BM_MultiThreadedLogging);
BENCHMARK(BM_Initialization);
BENCHMARK(BM_LogLevelChanges);
#endif

// Performance test: Latency measurement
TEST_F(QuillLoggerPerformanceTest, LoggingLatency) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.enable_console_output = false;
  config.enable_file_output = false;
  config.log_level = "INFO";
  
  logger->initialize(config);
  
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("latency_test_node");
  
  const int num_tests = 10000;
  std::vector<double> latencies;
  latencies.reserve(num_tests);
  
  for (int i = 0; i < num_tests; ++i) {
    auto start = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(node->get_logger(), "Latency test message %d", i);
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    latencies.push_back(duration.count());
  }
  
  // Calculate statistics
  double avg_latency = 0;
  double min_latency = std::numeric_limits<double>::max();
  double max_latency = 0;
  
  for (double latency : latencies) {
    avg_latency += latency;
    min_latency = std::min(min_latency, latency);
    max_latency = std::max(max_latency, latency);
  }
  avg_latency /= latencies.size();
  
  // Sort for percentile calculation
  std::sort(latencies.begin(), latencies.end());
  double p95_latency = latencies[static_cast<size_t>(num_tests * 0.95)];
  double p99_latency = latencies[static_cast<size_t>(num_tests * 0.99)];
  
  // Performance assertions
  EXPECT_LT(avg_latency, 1000);    // Average latency < 1 microsecond
  EXPECT_LT(p95_latency, 2000);    // 95th percentile < 2 microseconds
  EXPECT_LT(p99_latency, 5000);    // 99th percentile < 5 microseconds
  
  rclcpp::shutdown();
}

// Performance test: Throughput measurement
TEST_F(QuillLoggerPerformanceTest, LoggingThroughput) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.enable_console_output = false;
  config.enable_file_output = false;
  config.log_level = "INFO";
  
  logger->initialize(config);
  
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("throughput_test_node");
  
  const int num_messages = 100000;
  auto start = std::chrono::high_resolution_clock::now();
  
  for (int i = 0; i < num_messages; ++i) {
    RCLCPP_INFO(node->get_logger(), "Throughput test message %d", i);
  }
  
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  
  double throughput = (double)num_messages / (duration.count() / 1000.0);
  
  // Performance assertions
  EXPECT_GT(throughput, 100000);   // > 100k messages per second
  EXPECT_LT(duration.count(), 1000); // < 1 second for 100k messages
  
  rclcpp::shutdown();
}

// Performance test: Memory usage
TEST_F(QuillLoggerPerformanceTest, MemoryUsage) {
  auto logger = QuillLogger::getInstance();
  
  LoggerConfig config;
  config.enable_console_output = false;
  config.enable_file_output = false;
  config.log_level = "INFO";
  
  logger->initialize(config);
  
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("memory_test_node");
  
  // Log many messages to test memory usage
  const int num_messages = 50000;
  
  for (int i = 0; i < num_messages; ++i) {
    RCLCPP_INFO(node->get_logger(), "Memory test message %d with some additional data", i);
  }
  
  // Give some time for backend to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Memory usage should be reasonable (no memory leaks)
  // This is a basic test - in real scenarios you'd use memory profiling tools
  
  rclcpp::shutdown();
}

// Performance test: Realtime mode vs normal mode
TEST_F(QuillLoggerPerformanceTest, RealtimeVsNormalMode) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("realtime_test_node");
  
  // Test normal mode
  auto normal_logger = QuillLogger::getInstance();
  LoggerConfig normal_config;
  normal_config.enable_console_output = false;
  normal_config.enable_file_output = false;
  normal_config.log_level = "INFO";
  normal_config.enable_backend_performance_mode = false;
  
  normal_logger->initialize(normal_config);
  
  const int num_tests = 1000;
  auto normal_start = std::chrono::high_resolution_clock::now();
  
  for (int i = 0; i < num_tests; ++i) {
    RCLCPP_INFO(node->get_logger(), "Normal mode test %d", i);
  }
  
  auto normal_end = std::chrono::high_resolution_clock::now();
  auto normal_duration = std::chrono::duration_cast<std::chrono::microseconds>(normal_end - normal_start);
  
  normal_logger->shutdown();
  
  // Test realtime mode
  auto realtime_logger = QuillLogger::getInstance();
  LoggerConfig realtime_config;
  realtime_config.enable_console_output = false;
  realtime_config.enable_file_output = false;
  realtime_config.log_level = "INFO";
  realtime_config.enable_backend_performance_mode = true;
  realtime_config.backend_thread_sleep_duration_ns = 25000;
  
  realtime_logger->initialize(realtime_config);
  
  auto realtime_start = std::chrono::high_resolution_clock::now();
  
  for (int i = 0; i < num_tests; ++i) {
    RCLCPP_INFO(node->get_logger(), "Realtime mode test %d", i);
  }
  
  auto realtime_end = std::chrono::high_resolution_clock::now();
  auto realtime_duration = std::chrono::duration_cast<std::chrono::microseconds>(realtime_end - realtime_start);
  
  realtime_logger->shutdown();
  
  // Both modes should be fast
  EXPECT_LT(normal_duration.count(), 1000);    // Normal mode < 1ms
  EXPECT_LT(realtime_duration.count(), 1000);  // Realtime mode < 1ms
  
  rclcpp::shutdown();
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  
  // Run benchmarks if benchmark library is available
  if (argc > 1 && std::string(argv[1]) == "--benchmark") {
    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    return 0;
  }
  
  return RUN_ALL_TESTS();
} 