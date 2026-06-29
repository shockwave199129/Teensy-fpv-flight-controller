#!/usr/bin/env node
/**
 * Generates PNG/ICO/ICNS icons from the SVG source.
 * Run: node scripts/generate-icons.js
 * Requires: npm install --save-dev sharp png-to-ico
 */
const sharp = require('sharp');
const path = require('path');
const fs = require('fs');

const SVG_PATH = path.join(__dirname, '..', 'public', 'icon.svg');
const PUBLIC_DIR = path.join(__dirname, '..', 'public');
const BUILD_DIR = path.join(__dirname, '..', 'build');

const SIZES = [16, 32, 48, 64, 128, 256, 512];

async function main() {
  if (!fs.existsSync(BUILD_DIR)) fs.mkdirSync(BUILD_DIR, { recursive: true });

  const svgBuffer = fs.readFileSync(SVG_PATH);

  console.log('Generating PNG icons...');
  const pngPaths = {};
  for (const size of SIZES) {
    const outPath = path.join(PUBLIC_DIR, `icon-${size}.png`);
    await sharp(svgBuffer).resize(size, size).png().toFile(outPath);
    pngPaths[size] = outPath;
    console.log(`  icon-${size}.png`);
  }

  // Main icon.png (256x256)
  const mainPng = path.join(PUBLIC_DIR, 'icon.png');
  await sharp(svgBuffer).resize(256, 256).png().toFile(mainPng);
  console.log('  icon.png (256x256)');

  // ICO (Windows) - build from PNG data using ICO format spec
  try {
    const icoSizes = [16, 32, 48, 64, 128, 256];
    const pngBuffers = icoSizes.map(s => fs.readFileSync(pngPaths[s]));
    const numImages = pngBuffers.length;
    const headerSize = 6 + numImages * 16;
    let dataOffset = headerSize;

    const header = Buffer.alloc(headerSize);
    header.writeUInt16LE(0, 0);      // reserved
    header.writeUInt16LE(1, 2);      // ICO type
    header.writeUInt16LE(numImages, 4);

    const offsets = [];
    for (let i = 0; i < numImages; i++) {
      const size = icoSizes[i] >= 256 ? 0 : icoSizes[i];
      const entryOffset = 6 + i * 16;
      header.writeUInt8(size, entryOffset);          // width
      header.writeUInt8(size, entryOffset + 1);      // height
      header.writeUInt8(0, entryOffset + 2);         // color palette
      header.writeUInt8(0, entryOffset + 3);         // reserved
      header.writeUInt16LE(1, entryOffset + 4);      // color planes
      header.writeUInt16LE(32, entryOffset + 6);     // bits per pixel
      header.writeUInt32LE(pngBuffers[i].length, entryOffset + 8);  // data size
      header.writeUInt32LE(dataOffset, entryOffset + 12);           // data offset
      dataOffset += pngBuffers[i].length;
    }

    const icoBuffer = Buffer.concat([header, ...pngBuffers]);
    fs.writeFileSync(path.join(PUBLIC_DIR, 'icon.ico'), icoBuffer);
    console.log('  icon.ico (multi-resolution)');
  } catch (e) {
    console.log('  [SKIP] icon.ico -', e.message);
  }

  // ICNS (macOS) - generate iconset then use iconutil if on macOS
  if (process.platform === 'darwin') {
    const iconsetDir = path.join(BUILD_DIR, 'icon.iconset');
    if (!fs.existsSync(iconsetDir)) fs.mkdirSync(iconsetDir, { recursive: true });

    const icnsSizes = [
      [16, 'icon_16x16.png'], [32, 'icon_16x16@2x.png'],
      [32, 'icon_32x32.png'], [64, 'icon_32x32@2x.png'],
      [128, 'icon_128x128.png'], [256, 'icon_128x128@2x.png'],
      [256, 'icon_256x256.png'], [512, 'icon_256x256@2x.png'],
      [512, 'icon_512x512.png'],
    ];

    for (const [size, name] of icnsSizes) {
      await sharp(svgBuffer).resize(size, size).png().toFile(path.join(iconsetDir, name));
    }

    const { execSync } = require('child_process');
    try {
      execSync(`iconutil -c icns "${iconsetDir}" -o "${path.join(PUBLIC_DIR, 'icon.icns')}"`);
      console.log('  icon.icns (macOS)');
    } catch (e) {
      console.log('  [SKIP] icon.icns - iconutil failed');
    }
  } else {
    console.log('  [SKIP] icon.icns - only generated on macOS (use iconutil)');
  }

  // DMG background (540x380 with gradient and text)
  const dmgWidth = 540;
  const dmgHeight = 380;
  const dmgSvg = `<svg width="${dmgWidth}" height="${dmgHeight}" xmlns="http://www.w3.org/2000/svg">
    <defs>
      <linearGradient id="bg" x1="0%" y1="0%" x2="100%" y2="100%">
        <stop offset="0%" style="stop-color:#1e3a5f;stop-opacity:1" />
        <stop offset="100%" style="stop-color:#0f172a;stop-opacity:1" />
      </linearGradient>
    </defs>
    <rect width="${dmgWidth}" height="${dmgHeight}" fill="url(#bg)"/>
    <text x="${dmgWidth / 2}" y="80" text-anchor="middle" fill="#ffffff" font-family="Arial,sans-serif" font-size="28" font-weight="bold">FPV Drone Configurator</text>
    <text x="${dmgWidth / 2}" y="110" text-anchor="middle" fill="#94a3b8" font-family="Arial,sans-serif" font-size="14">Drag to Applications to install</text>
    <line x1="170" y1="230" x2="370" y2="230" stroke="#334155" stroke-width="2" stroke-dasharray="8,4"/>
    <polygon points="350,225 370,230 350,235" fill="#334155"/>
  </svg>`;

  await sharp(Buffer.from(dmgSvg)).png().toFile(path.join(BUILD_DIR, 'dmg-background.png'));
  console.log('  build/dmg-background.png');

  console.log('\nDone! Run "npm run build-electron" to package the app.');
}

main().catch(err => {
  console.error('Error generating icons:', err);
  process.exit(1);
});
