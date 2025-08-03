#include <rclcpp/rclcpp.hpp>
#include <quill_logger/quill_logger.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Initialize Quill logger with realtime optimizations
    auto logger = quill_logger::QuillLogger::getInstance();
    
    // Configure for realtime performance
    quill_logger::LoggerConfig config;
    config.enable_backend_performance_mode = true;     // Enable realtime mode
    config.backend_thread_sleep_duration_ns = 50000;  // 50 microseconds
    config.backend_thread_cpu_affinity = 1;           // Use CPU core 1
    config.backend_thread_priority = 80;              // High priority
    config.enable_file_output = false;                // Disable file I/O for max performance
    config.log_level = "WARN";                        // Reduce log volume
    
    logger->initialize(config);
    
    // Now all ROS2 logs will go through optimized Quill!
    auto node = std::make_shared<rclcpp::Node>("example_node");
    
    // These messages will be logged through optimized Quill
    RCLCPP_INFO(node->get_logger(), "Realtime-optimized Quill logger initialized!");
    RCLCPP_WARN(node->get_logger(), "Warning through optimized Quill");
    RCLCPP_ERROR(node->get_logger(), "Error through optimized Quill");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
} 