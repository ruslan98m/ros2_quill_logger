#include <rclcpp/rclcpp.hpp>
#include <quill_logger/quill_logger.hpp>

int main(int argc, char** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Initialize Quill logger - this automatically replaces the default logger
    auto logger = quill_logger::QuillLogger::getInstance();
    logger->initialize();
    
    // Now all ROS2 logs will go through Quill!
    auto node = std::make_shared<rclcpp::Node>("example_node");
    
    // These messages will be logged through Quill
    RCLCPP_INFO(node->get_logger(), "This message goes through Quill logger!");
    RCLCPP_WARN(node->get_logger(), "Warning through Quill");
    RCLCPP_ERROR(node->get_logger(), "Error through Quill");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
} 