#!/bin/bash

# Build script for quill-logger debian package
set -e

echo "🔧 Building quill-logger debian package..."

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ] || [ ! -d "debian" ]; then
    echo "❌ Error: Must be run from the quill_logger root directory"
    exit 1
fi

# Clean previous builds
echo "🧹 Cleaning previous builds..."
rm -rf build/
rm -f ../quill-logger_*.deb
rm -f ../quill-logger_*.dsc
rm -f ../quill-logger_*.tar.gz
rm -f ../quill-logger_*.changes

# Install build dependencies
echo "📦 Installing build dependencies..."
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    debhelper \
    devscripts \
    dh-make \
    cmake \
    pkg-config \
    libgtest-dev

# Install ROS2 Humble if not already installed
if ! pkg-config --exists rclcpp; then
    echo "📦 Installing ROS2 Humble..."
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt-get update && sudo apt-get install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y ros-humble-desktop
    sudo apt-get install -y ros-humble-rclcpp ros-humble-rcl ros-humble-rcutils ros-humble-rosidl-runtime-c
fi

# Install Quill library if not already installed
if ! pkg-config --exists quill; then
    echo "📦 Installing Quill library..."
    # Try to install from package manager first
    if ! sudo apt-get install -y libquill-dev; then
        echo "⚠️  Quill library not available in package manager"
        echo "   You may need to build it from source:"
        echo "   https://github.com/odygrd/quill"
        echo "   For now, we'll try to build without it..."
    fi
fi

# Check dependencies
echo "🔍 Checking dependencies..."
if [ -d "/opt/ros/humble" ]; then
    echo "✅ ROS2 Humble found"
else
    echo "❌ ROS2 Humble not found"
    exit 1
fi

if pkg-config --exists quill; then
    echo "✅ Quill library found"
else
    echo "⚠️  Quill library not found - will need to be installed separately"
fi

# Build the package
echo "🔨 Building debian package..."
debuild -us -uc -b

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "✅ Package built successfully!"
    echo "📦 Generated packages:"
    ls -la ../quill-logger_*
    
    echo ""
    echo "🎯 To install the package:"
    echo "   sudo dpkg -i ../quill-logger_*.deb"
    echo "   sudo apt-get install -f  # Install missing dependencies"
    
    echo ""
    echo "🎯 To install all packages:"
    echo "   sudo dpkg -i ../libquill-logger-dev_*.deb"
    echo "   sudo dpkg -i ../quill-logger-demo_*.deb"
    echo "   sudo dpkg -i ../quill-logger-dbg_*.deb"
    
else
    echo "❌ Package build failed!"
    exit 1
fi 