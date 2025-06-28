#!/bin/bash

# FPV Drone Configurator - Cross-Platform Build Script
# This script builds the application for Windows, macOS, and Linux

set -e

echo "🚀 FPV Drone Configurator - Cross-Platform Build Script"
echo "========================================================"

# Check if we're in the correct directory
if [ ! -f "package.json" ]; then
    echo "❌ Error: package.json not found. Please run this script from the desktop_app directory."
    exit 1
fi

# Check if node_modules exists
if [ ! -d "node_modules" ]; then
    echo "📦 Installing dependencies..."
    npm install
fi

# Build the React app first
echo "⚛️ Building React application..."
npm run build

if [ $? -ne 0 ]; then
    echo "❌ React build failed!"
    exit 1
fi

echo "✅ React build completed successfully"

# Function to build for specific platform
build_platform() {
    local platform=$1
    local script_name=$2
    
    echo ""
    echo "🔨 Building for $platform..."
    echo "────────────────────────────────"
    
    if npm run "$script_name"; then
        echo "✅ $platform build completed successfully"
        
        # Show output files
        echo "📁 Output files:"
        case $platform in
            "Windows")
                find dist/ -name "*.exe" -o -name "*.msi" 2>/dev/null | head -5
                ;;
            "macOS")
                find dist/ -name "*.dmg" -o -name "*.zip" 2>/dev/null | head -5
                ;;
            "Linux")
                find dist/ -name "*.AppImage" -o -name "*.deb" -o -name "*.rpm" 2>/dev/null | head -5
                ;;
        esac
    else
        echo "❌ $platform build failed!"
        return 1
    fi
}

# Main build process
echo ""
echo "🏗️ Starting cross-platform builds..."

# Check what to build
if [ "$1" = "all" ] || [ "$1" = "" ]; then
    echo "Building for all platforms..."
    
    build_platform "Linux" "dist-linux"
    
    # Only build for other platforms if we're on appropriate OS or in CI
    if command -v wine &> /dev/null || [ "$CI" = "true" ]; then
        build_platform "Windows" "dist-win"
    else
        echo "⚠️ Wine not detected - skipping Windows build"
        echo "💡 To build for Windows from Linux, install Wine or use GitHub Actions"
    fi
    
    if [ "$(uname)" = "Darwin" ] || [ "$CI" = "true" ]; then
        build_platform "macOS" "dist-mac"
    else
        echo "⚠️ Not on macOS - skipping macOS build"
        echo "💡 macOS builds can only be created on macOS or via GitHub Actions"
    fi
    
elif [ "$1" = "linux" ]; then
    build_platform "Linux" "dist-linux"
elif [ "$1" = "windows" ] || [ "$1" = "win" ]; then
    build_platform "Windows" "dist-win"
elif [ "$1" = "mac" ] || [ "$1" = "macos" ]; then
    build_platform "macOS" "dist-mac"
else
    echo "Usage: $0 [all|linux|windows|mac]"
    echo "  all     - Build for all platforms (default)"
    echo "  linux   - Build for Linux only"
    echo "  windows - Build for Windows only"
    echo "  mac     - Build for macOS only"
    exit 1
fi

echo ""
echo "🎉 Build process completed!"
echo "📁 All outputs are in the 'dist/' directory"

# Show disk usage
if command -v du &> /dev/null; then
    echo ""
    echo "📊 Build output sizes:"
    du -sh dist/* 2>/dev/null || echo "No build outputs found"
fi

echo ""
echo "✨ Ready for distribution!"
echo "📋 Next steps:"
echo "   1. Test the applications on target platforms"
echo "   2. Sign the executables for distribution"
echo "   3. Upload to your distribution channels" 