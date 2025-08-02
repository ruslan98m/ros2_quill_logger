# Quill Logger for ROS2

**Author: Ruslan Mambetov**

A high-performance ROS2 logging system using the Quill library that provides configurable output formats and can be used as a drop-in replacement for the default ROS2 logger.

## Features

- ⚡ **High Performance** - Minimal overhead logging
- 🔄 **Asynchronous Logging** - Non-blocking main thread
- 🎨 **Colored Output** - Beautiful console formatting
- 📁 **File Rotation** - Automatic log file management
- 🔧 **Flexible Configuration** - Multiple configuration options
- 🔄 **Full Compatibility** - Complete ROS2 logging API compatibility

## Quick Start

### Method 1: Automatic Replacement in Code (Recommended)

Add to the beginning of your `main()`:

```cpp
#include <quill_logger/quill_logger.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Initialize Quill logger - this automatically replaces the default logger
    auto logger = quill_logger::QuillLogger::getInstance();
    logger->initialize();
    
    // Now all ROS2 logs will go through Quill!
    auto node = std::make_shared<rclcpp::Node>("my_node");
    
    RCLCPP_INFO(node->get_logger(), "This message goes through Quill logger!");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Method 2: Manual Initialization

```bash
# Run the test node to see Quill logger in action
ros2 run quill_logger quill_logger_test_node

# Or run the simple example
ros2 run quill_logger simple_node_example
```

## Installation

### 1. Add Dependency

In `package.xml`:
```xml
<depend>quill_logger</depend>
```

In `CMakeLists.txt`:
```cmake
find_package(quill_logger REQUIRED)
```

### 2. Add Initialization to Code

```cpp
#include <quill_logger/quill_logger.hpp>

// In the beginning of main()
auto logger = quill_logger::QuillLogger::getInstance();
logger->initialize();
```

### 3. Rebuild Project

```bash
colcon build --packages-select your_package
source install/setup.bash
```

### 4. Test

```bash
ros2 run your_package your_node
```

## Configuration

### Basic Settings

```cpp
quill_logger::LoggerConfig config;
config.log_level = "DEBUG";                    // DEBUG, INFO, WARN, ERROR, FATAL
config.log_file_path = "/tmp/my_app.log";      // Log file path
config.enable_console_output = true;           // Console output
config.enable_file_output = true;              // File output

auto logger = quill_logger::QuillLogger::getInstance();
logger->initialize(config);
```

### Runtime Configuration

```cpp
// Change log level during runtime
logger->setLogLevel(quill_logger::LogLevel::DEBUG);

// Or via string
logger->setLogLevel("DEBUG");
```

## Log Files

After Quill logger initialization, logs will be written to:

- **Console**: Colored output with timestamps
- **File**: `~/.ros/log/quill_logger_YYYY-MM-DD-HH-MM-SS.log` (ROS2 standard location)

Example output:
```
01:29:38.529009053 [327241] quill_logger.cpp:130         LOG_INFO      ros2_quill_logger Quill logger initialized successfully
01:29:38.529128510 [327241] quill_logger.cpp:131         LOG_INFO      ros2_quill_logger Log file: /home/pc/.ros/log/quill_logger_2025-08-03-01-29-38.log
01:29:38.529128833 [327241] quill_logger.cpp:132         LOG_INFO      ros2_quill_logger Log level: DEBUG
```

## Available Components

- **`quill_logger_test_node`** - Test node for functionality verification
- **`simple_node_example`** - Example of automatic replacement

## Troubleshooting

### Problem: Logs don't appear

**Solution**: Ensure Quill logger is initialized before creating nodes:

```cpp
// ✅ Correct
auto logger = quill_logger::QuillLogger::getInstance();
logger->initialize();
auto node = std::make_shared<rclcpp::Node>("my_node");

// ❌ Incorrect
auto node = std::make_shared<rclcpp::Node>("my_node");
auto logger = quill_logger::QuillLogger::getInstance();
logger->initialize();
```

### Problem: Log file not created

**Solution**: Check permissions and path:

```cpp
config.log_file_path = "/tmp/my_app.log";  // Ensure directory exists
```

### Problem: Low performance

**Solution**: Configure log level:

```cpp
config.log_level = "INFO";  // Instead of DEBUG for production
```

## Quick Test

```bash
# 1. Build the package
colcon build --packages-select quill_logger

# 2. Source the workspace
source install/setup.bash

# 3. Run the test
ros2 run quill_logger quill_logger_test_node

# 4. Check the log file
tail -f ~/.ros/log/quill_logger_*.log
```

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Author

**Ruslan Mambetov** - High-performance ROS2 logging system using Quill library 