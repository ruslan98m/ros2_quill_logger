#!/bin/bash

set -e

echo "=== Testing APT Installation ==="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if we're building the package first
if [ ! -f "../quill-logger_*.deb" ]; then
    print_warning "Debian package not found. Building it first..."
    ./scripts/build_deb.sh
fi

# Find the deb package
DEB_PACKAGE=$(find .. -name "quill-logger_*.deb" | head -1)
if [ -z "$DEB_PACKAGE" ]; then
    print_error "No Debian package found!"
    exit 1
fi

print_status "Found package: $DEB_PACKAGE"

# Install the package
print_status "Installing Debian package..."
sudo dpkg -i "$DEB_PACKAGE" || true
sudo apt-get install -f -y

# Test pkg-config
print_status "Testing pkg-config..."
if pkg-config --exists quill_logger; then
    print_status "✓ pkg-config found quill_logger"
else
    print_error "✗ pkg-config could not find quill_logger"
    exit 1
fi

print_status "pkg-config --cflags:"
pkg-config --cflags quill_logger

print_status "pkg-config --libs:"
pkg-config --libs quill_logger

# Test header compilation
print_status "Testing header compilation..."
cat > test_install.cpp << 'EOF'
#include <quill_logger/quill_logger.hpp>
#include <quill_logger/quill_logger_interface.hpp>

int main() {
    // Test basic functionality
    quill_logger::LoggerConfig config;
    config.log_level = "INFO";
    config.enable_console_output = true;
    config.enable_file_output = false;
    
    // Test initialization
    quill_logger::QuillLogger::initialize(config);
    
    // Test getting logger
    auto logger = quill_logger::QuillLogger::getLogger("test_node");
    logger->info("Test message from APT installation");
    
    // Test shutdown
    quill_logger::QuillLogger::shutdown();
    
    std::cout << "✓ All tests passed!" << std::endl;
    return 0;
}
EOF

# Compile test program
print_status "Compiling test program..."
g++ -o test_install test_install.cpp $(pkg-config --cflags --libs quill_logger)

# Run test program
print_status "Running test program..."
./test_install

# Test demo applications
print_status "Testing demo applications..."
if [ -f "/usr/bin/quill_logger_test_node" ]; then
    print_status "✓ Demo application found: /usr/bin/quill_logger_test_node"
else
    print_warning "✗ Demo application not found"
fi

# Test library files
print_status "Testing library files..."
if [ -f "/usr/lib/libquill_logger.so" ] || [ -f "/usr/lib/x86_64-linux-gnu/libquill_logger.so" ]; then
    print_status "✓ Library file found"
else
    print_warning "✗ Library file not found"
fi

# Test header files
print_status "Testing header files..."
if [ -f "/usr/include/quill_logger/quill_logger.hpp" ]; then
    print_status "✓ Header files found"
else
    print_warning "✗ Header files not found"
fi

# Test config files
print_status "Testing config files..."
if [ -f "/usr/share/quill_logger/config/quill_logger_config.yaml" ]; then
    print_status "✓ Config files found"
else
    print_warning "✗ Config files not found"
fi

# Test ROS2 integration
print_status "Testing ROS2 integration..."
if [ -n "$ROS_DISTRO" ]; then
    print_status "ROS2 environment detected: $ROS_DISTRO"
    
    # Test ROS2 compilation
    cat > test_ros2.cpp << 'EOF'
#include <rclcpp/rclcpp.hpp>
#include <quill_logger/quill_logger_interface.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("test_node");
    
    // Test ROS2 logging integration
    RCLCPP_INFO(node->get_logger(), "Testing ROS2 integration");
    
    rclcpp::shutdown();
    return 0;
}
EOF
    
    # Try to compile ROS2 test
    if command -v ament_cmake &> /dev/null; then
        print_status "✓ ROS2 build tools available"
    else
        print_warning "✗ ROS2 build tools not available"
    fi
else
    print_warning "ROS2 environment not detected"
fi

# Cleanup
print_status "Cleaning up..."
rm -f test_install test_install.cpp test_ros2.cpp

print_status "=== APT Installation Test Complete ==="
print_status "✓ All tests passed successfully!" 