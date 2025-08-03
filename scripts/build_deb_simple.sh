#!/bin/bash

# Simple build script for quill-logger debian package (without external dependencies)
set -e

echo "🔧 Building quill-logger debian package (simple version)..."

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

# Create a simple test build
echo "🔨 Creating test build..."
mkdir -p build
cd build
cmake .. -DBUILD_TESTING=OFF
make -j$(nproc)
cd ..

# Build the package with -d flag to ignore dependencies
echo "🔨 Building debian package (ignoring dependencies)..."
debuild -us -uc -b -d

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "✅ Package built successfully!"
    echo "📦 Generated packages:"
    ls -la ../quill-logger_*
    
    echo ""
    echo "🎯 Note: This is a test build without external dependencies"
    echo "   For full functionality, install ROS2 Humble and Quill library first"
    
    echo ""
    echo "🎯 To install the package:"
    echo "   sudo dpkg -i ../quill-logger_*.deb"
    echo "   sudo apt-get install -f  # Install missing dependencies"
    
else
    echo "❌ Package build failed!"
    exit 1
fi 