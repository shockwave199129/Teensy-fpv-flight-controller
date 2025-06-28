# Public Assets Folder

This folder contains static assets for the FPV Drone Desktop Application.

## ✅ Available Assets

### Application Icons
- ✅ `icon.svg` - Main application icon (vector source)
- 🔲 `icon.png` (256x256) - PNG version needed for general use
- 🔲 `icon.ico` - Windows executable icon (to be generated)
- 🔲 `icon.icns` - macOS application bundle icon (to be generated)

### UI Images
- ✅ `drone-diagram.svg` - Quadcopter X-configuration layout diagram
- ✅ `compass-rose.svg` - GPS/magnetometer compass visualization
- 🔲 `drone-diagram.png` - PNG version for broader compatibility
- 🔲 `compass-rose.png` - PNG version for broader compatibility

### Calibration Position Images
All accelerometer calibration position diagrams (SVG sources available):
- ✅ `calibration-positions/position-1-level.svg` - Normal flight orientation
- ✅ `calibration-positions/position-2-upside-down.svg` - 180° roll flip
- ✅ `calibration-positions/position-3-right-side.svg` - 90° roll right
- ✅ `calibration-positions/position-4-left-side.svg` - 90° roll left
- ✅ `calibration-positions/position-5-nose-down.svg` - 90° pitch forward
- ✅ `calibration-positions/position-6-nose-up.svg` - 90° pitch backward

PNG versions needed:
- 🔲 `calibration-positions/position-1-level.png`
- 🔲 `calibration-positions/position-2-upside-down.png`
- 🔲 `calibration-positions/position-3-right-side.png`
- 🔲 `calibration-positions/position-4-left-side.png`
- 🔲 `calibration-positions/position-5-nose-down.png`
- 🔲 `calibration-positions/position-6-nose-up.png`

### Documentation
- ✅ `user-manual.md` - Comprehensive user manual (Markdown source)
- ✅ `quick-start-guide.md` - Quick start guide (Markdown source)
- ✅ `cli-guide.md` - Complete CLI command reference and usage guide
- 🔲 `user-manual.pdf` - PDF version needed for distribution
- 🔲 `quick-start-guide.pdf` - PDF version needed for distribution
- 🔲 `cli-guide.pdf` - PDF version needed for distribution

### Development Assets
- ✅ `BINARY_ASSETS_TODO.md` - Instructions for creating binary assets from sources

## 🛠️ Asset Creation Guide

### SVG to PNG Conversion
All SVG files can be converted to PNG using:
- **Inkscape**: `inkscape file.svg --export-type=png --export-filename=file.png --export-width=256`
- **Online converters**: Various web-based SVG to PNG tools
- **ImageMagick**: `convert file.svg -resize 256x256 file.png`

### Markdown to PDF Conversion
Convert documentation using:
- **Pandoc**: `pandoc user-manual.md -o user-manual.pdf --pdf-engine=xelatex --toc`
- **Online converters**: Markdown to PDF web tools
- **LibreOffice**: Import markdown, export as PDF

### Icon Generation
Create platform-specific icons from `icon.svg`:
- **Windows ICO**: Use online converters or IconForge
- **macOS ICNS**: Use iconutil command-line tool
- **Multi-resolution**: Include 16x16, 32x32, 48x48, 64x64, 128x128, 256x256

## 📁 File Structure

```
public/
├── README.md                                   ✅ This file
├── BINARY_ASSETS_TODO.md                      ✅ Asset creation guide
├── icon.svg                                   ✅ Main app icon (source)
├── drone-diagram.svg                          ✅ Motor configuration diagram
├── compass-rose.svg                           ✅ GPS/magnetometer compass
├── user-manual.md                             ✅ User manual (source)
├── quick-start-guide.md                       ✅ Quick start guide (source)
└── calibration-positions/                     ✅ Calibration position images
    ├── position-1-level.svg                   ✅ Level position
    ├── position-2-upside-down.svg             ✅ Upside down position
    ├── position-3-right-side.svg              ✅ Right side position
    ├── position-4-left-side.svg               ✅ Left side position
    ├── position-5-nose-down.svg               ✅ Nose down position
    └── position-6-nose-up.svg                 ✅ Nose up position
```

## 🎨 Design Guidelines

### Visual Consistency
- **Color Scheme**: Blue (#1e40af), Green (#10b981), Red (#ef4444), Purple (#8b5cf6)
- **Typography**: Arial/Helvetica for SVG text
- **Icon Style**: Modern, clean, professional
- **Background**: Light gray (#f8fafc) for diagrams

### Technical Specifications
- **SVG Viewboxes**: Properly defined for scalability
- **PNG Resolution**: 96 DPI for web use
- **Transparency**: Maintained where appropriate
- **File Naming**: Kebab-case for consistency

## 📊 File Size Guidelines

- **Icons**: < 100KB each (SVG < 10KB, PNG < 100KB)
- **Diagrams**: < 500KB each (SVG < 20KB, PNG < 500KB)
- **Documentation**: < 5MB each (Markdown < 100KB, PDF < 5MB)
- **Total Folder**: Target < 20MB for distribution

## ⚡ Quick Build Commands

### Convert All SVGs to PNG
```bash
# Using Inkscape
find . -name "*.svg" -exec inkscape {} --export-type=png --export-filename={}.png \;

# Using ImageMagick
find . -name "*.svg" -exec convert {} {}.png \;
```

### Build Documentation
```bash
# Create PDFs from Markdown
pandoc user-manual.md -o user-manual.pdf --pdf-engine=xelatex --toc
pandoc quick-start-guide.md -o quick-start-guide.pdf --pdf-engine=xelatex
```

### Create Icons
```bash
# Generate PNG icon
inkscape icon.svg --export-type=png --export-filename=icon.png --export-width=256

# For ICO and ICNS, use platform-specific tools or online converters
```

## 🎯 Usage in Application

### SVG Files
- Directly usable in modern browsers and Electron
- Scalable for different screen resolutions
- Smaller file sizes for vector graphics

### PNG Files
- Fallback for older systems
- Required for system icons and some UI libraries
- Better compatibility across platforms

### Documentation
- Markdown files for online viewing
- PDF files for offline distribution and printing
- Embedded help system integration

## 📝 Notes

- All SVG files include proper accessibility attributes
- Color choices ensure good contrast ratios
- Text remains readable at small sizes
- Graphics work well on both light and dark backgrounds
- Professional appearance suitable for commercial use

---

**Status**: ✅ All source assets created | 🔲 Binary conversion needed  
**Last Updated**: 2024  
**Asset Count**: 11 SVG files, 2 Markdown documents, 1 guide  
**Ready for**: Binary asset generation and distribution 