# Binary Assets Creation Guide

This document lists the binary assets that need to be created from the SVG sources provided.

## Application Icons

### Required Formats
Create these icons from `icon.svg`:

1. **icon.png** (256x256)
   - Main application icon for general use
   - High-quality PNG with transparency
   - Used in application UI and taskbar

2. **icon.ico** (Windows)
   - Multi-resolution ICO file with sizes: 16x16, 32x32, 48x48, 64x64, 128x128, 256x256
   - For Windows executables and shortcuts
   - Include transparency

3. **icon.icns** (macOS)
   - Apple icon format with multiple resolutions
   - For macOS application bundles
   - Include retina (@2x) variants

### Creation Instructions
1. Open `icon.svg` in a vector graphics editor (Inkscape, Illustrator, etc.)
2. Export as PNG at 256x256 resolution with transparent background
3. Use icon conversion tools to create ICO and ICNS formats:
   - Windows: Use online converters or IconForge
   - macOS: Use iconutil command-line tool
   - Cross-platform: Use ImageMagick or online converters

## UI Images

### From SVG Sources
Create PNG versions of these SVG files for broader compatibility:

1. **drone-diagram.png** (400x400)
   - Convert from `drone-diagram.svg`
   - Used in motor configuration interface

2. **compass-rose.png** (300x300)
   - Convert from `compass-rose.svg`
   - Used in GPS and magnetometer displays

### Calibration Position Images
Convert all calibration position SVGs to PNG format (250x200):

1. **calibration-positions/position-1-level.png**
2. **calibration-positions/position-2-upside-down.png**
3. **calibration-positions/position-3-right-side.png**
4. **calibration-positions/position-4-left-side.png**
5. **calibration-positions/position-5-nose-down.png**
6. **calibration-positions/position-6-nose-up.png**

### Export Settings
- Format: PNG with transparency
- Quality: Maximum/Lossless
- Background: Transparent
- DPI: 96 (web standard)

## Documentation

### PDF Creation
Convert markdown documents to PDF format:

1. **user-manual.pdf**
   - Source: `user-manual.md`
   - Include table of contents
   - Professional formatting with headers/footers
   - Page numbering

2. **quick-start-guide.pdf**
   - Source: `quick-start-guide.md`
   - Compact, easy-to-print format
   - Include diagrams where helpful

### PDF Creation Tools
- **Pandoc**: Command-line markdown to PDF conversion
- **Markdown to PDF**: Online converters
- **GitBook/Sphinx**: Documentation generators
- **Adobe Acrobat**: Professional PDF creation
- **LibreOffice Writer**: Import markdown, export PDF

### Recommended Pandoc Command
```bash
pandoc user-manual.md -o user-manual.pdf --pdf-engine=xelatex --toc
pandoc quick-start-guide.md -o quick-start-guide.pdf --pdf-engine=xelatex
```

## Fonts (Optional)

If custom fonts are needed:

1. **Modern Sans-Serif**: For UI elements (e.g., Inter, Roboto)
2. **Monospace**: For code/terminal displays (e.g., Fira Code, Source Code Pro)

Ensure fonts have appropriate licenses for distribution.

## File Size Guidelines

Target file sizes:
- Icons: < 100KB each
- UI Images: < 500KB each
- Fonts: < 1MB each
- PDFs: < 5MB each
- Total public folder: < 20MB

## Automation Script

Consider creating a build script to automate asset generation:

```bash
#!/bin/bash
# Convert SVG to PNG
inkscape icon.svg --export-type=png --export-filename=icon.png --export-width=256

# Convert markdown to PDF
pandoc user-manual.md -o user-manual.pdf --pdf-engine=xelatex --toc

# Add to build process
npm run build:assets
```

## Quality Checklist

Before finalizing assets:
- [ ] All images have transparent backgrounds where appropriate
- [ ] Icons work well at small sizes (16x16, 32x32)
- [ ] Colors are consistent with application theme
- [ ] Text is readable in all images
- [ ] PDF documents are properly formatted
- [ ] File sizes are within guidelines
- [ ] All formats are cross-platform compatible

---

*This guide ensures all visual assets maintain consistent quality and branding throughout the application.* 