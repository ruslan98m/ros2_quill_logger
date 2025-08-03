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
- 🚀 **Realtime Optimized** - Configurable backend thread for realtime systems

## Realtime Performance

This library is optimized for realtime systems with configurable backend thread settings:

- **CPU Affinity**: Bind backend thread to specific CPU core
- **Thread Priority**: Set SCHED_FIFO priority for backend thread  
- **Sleep Duration**: Configure backend thread response time
- **Performance Mode**: Enable/disable realtime optimizations

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
```

## Installation

### From Source

#### Prerequisites

- ROS2 Humble
- Quill library
- CMake 3.8+
- Build tools (gcc, make, etc.)

#### Build Instructions

```bash
# Clone the repository
git clone https://github.com/ruslan98m/ros2_quill_logger.git
cd ros2_quill_logger

# Build
source /opt/ros/humble/setup.bash
colcon build --packages-select quill_logger

# Install
source install/setup.bash
```

### Using as ROS2 Package Dependency

#### 1. Add Dependency

In `package.xml`:
```xml
<depend>quill_logger</depend>
```

In `CMakeLists.txt`:
```cmake
find_package(quill_logger REQUIRED)
```

#### 2. Add Initialization to Code

```cpp
#include <quill_logger/quill_logger.hpp>

// In the beginning of main()
auto logger = quill_logger::QuillLogger::getInstance();
logger->initialize();
```

#### 3. Rebuild Project

```bash
colcon build --packages-select your_package
source install/setup.bash
```

#### 4. Test

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

### Realtime Performance Settings

For realtime systems, you can optimize the backend thread performance:

```cpp
quill_logger::LoggerConfig config;
config.enable_backend_performance_mode = true;     // Enable realtime optimizations
config.backend_thread_sleep_duration_ns = 50000;  // 50 microseconds (faster response)
config.backend_thread_cpu_affinity = 1;           // Use CPU core 1
config.backend_thread_priority = 80;              // High priority (SCHED_FIFO)

auto logger = quill_logger::QuillLogger::getInstance();
logger->initialize(config);
```

**Realtime Configuration Options:**
- `enable_backend_performance_mode`: Enable realtime optimizations
- `backend_thread_sleep_duration_ns`: Backend thread sleep duration (lower = faster response)
- `backend_thread_cpu_affinity`: CPU core to bind backend thread (-1 = no affinity)
- `backend_thread_priority`: Thread priority 1-99 for SCHED_FIFO (-1 = default)

**Recommended settings for realtime systems:**
```yaml
enable_backend_performance_mode: true
backend_thread_sleep_duration_ns: 50000  # 50 microseconds
backend_thread_cpu_affinity: 1           # Dedicated CPU core
backend_thread_priority: 80              # High priority
enable_file_output: false                # Disable file I/O for maximum performance
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

## Testing

The library includes comprehensive unit tests to ensure reliability and performance:

### Running Tests

```bash
# Run all tests
colcon test --packages-select quill_logger

# Run specific test
colcon test --packages-select quill_logger --ctest-args -R test_quill_logger

# View test results
colcon test-result --all
```

### Test Coverage

- **Unit Tests**: Core functionality testing
- **Integration Tests**: ROS2 interface testing  
- **Performance Tests**: Benchmarking and performance validation
- **Simple Tests**: Basic functionality verification

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [Quill](https://github.com/odygrd/quill) - High-performance logging library
- [ROS2](https://ros.org/) - Robot Operating System 2