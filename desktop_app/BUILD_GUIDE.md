# FPV Drone Configurator - Cross-Platform Build Guide

Complete guide for building the application for Windows, macOS, and Linux.

## Quick Start

### 1. Install Dependencies
```bash
cd desktop_app
npm install
```

### 2. Create Icons (Required)
```bash
# Install icon tools (Linux/macOS)
sudo apt install inkscape  # Ubuntu/Debian
brew install inkscape      # macOS

# Create PNG icon
inkscape public/icon.svg --export-type=png --export-filename=public/icon.png --export-width=256
```

### 3. Build Applications
```bash
# Automated build script
./build-all-platforms.sh

# Or manual commands
npm run dist-linux    # Linux builds
npm run dist-win      # Windows builds  
npm run dist-mac      # macOS builds
npm run dist-all      # All platforms
```

## Build Commands

| Command | Description | Output |
|---------|-------------|--------|
| `npm run dist-all` | Build for all platforms | Multiple formats |
| `npm run dist-linux` | Linux only | AppImage, DEB, RPM |
| `npm run dist-win` | Windows only | NSIS, Portable |
| `npm run dist-mac` | macOS only | DMG, ZIP |
| `npm run pack` | Development build | Unpacked apps |

## Platform Requirements

### Linux (Current Platform)
- ✅ Can build: Linux packages
- ⚠️ Can build: Windows (with Wine)
- ❌ Cannot build: macOS

### Additional Tools
```bash
# For Windows builds from Linux
sudo apt install wine

# For icon conversion
sudo apt install inkscape imagemagick
```

## Output Files

All builds go to `dist/` directory:

**Linux:**
- `*.AppImage` - Universal Linux app
- `*.deb` - Debian/Ubuntu package
- `*.rpm` - RedHat/Fedora package

**Windows:**
- `*.exe` - NSIS installer
- `*-portable.exe` - Portable app

**macOS:**
- `*.dmg` - Disk image
- `*.zip` - App bundle

## GitHub Actions (Recommended)

For building on all platforms automatically, use GitHub Actions.

Create `.github/workflows/build.yml`:

```yaml
name: Build All Platforms
on: [push, pull_request]

jobs:
  build:
    strategy:
      matrix:
        os: [ubuntu-latest, windows-latest, macos-latest]
    runs-on: ${{ matrix.os }}
    
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: '18'
      
      - name: Install dependencies
        working-directory: desktop_app
        run: npm ci
      
      - name: Build
        working-directory: desktop_app
        run: |
          if [ "$RUNNER_OS" == "Linux" ]; then
            npm run dist-linux
          elif [ "$RUNNER_OS" == "Windows" ]; then
            npm run dist-win
          elif [ "$RUNNER_OS" == "macOS" ]; then
            npm run dist-mac
          fi
        shell: bash
```

This automatically builds for all platforms on every commit! 