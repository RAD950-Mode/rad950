#!/usr/bin/env bash
# Build a Universal (or single-arch) macOS RAD-950.app bundle and drag-to-Applications .dmg.
# Usage:
#   make_dmg.sh <arm64_bin> <x64_bin_or_empty> <version> <out_dmg>
set -euo pipefail

ARM64_BIN="${1:?arm64 binary path required}"
X64_BIN="${2:-}"
RAW_VERSION="${3:-0.1.0}"
OUT_DMG="${4:-rad950-macos-universal.dmg}"
VERSION="${RAW_VERSION#v}"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

APP="$WORK/stage/RAD-950.app"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources"

if [[ -n "$X64_BIN" && -f "$X64_BIN" ]]; then
  lipo -create -output "$APP/Contents/MacOS/rad950" "$ARM64_BIN" "$X64_BIN"
else
  cp "$ARM64_BIN" "$APP/Contents/MacOS/rad950"
fi
chmod +x "$APP/Contents/MacOS/rad950"

# Generate the RAD-950 4-FSK ascending-bars AppIcon.icns using only Python stdlib + sips + iconutil
ICONSET="$WORK/AppIcon.iconset"
mkdir -p "$ICONSET"
python3 - "$WORK/icon_1024.png" << 'PY'
import struct, sys, zlib

W = H = 1024
buf = bytearray(W * H * 4)

def fill_rect(x0, y0, x1, y1, r, g, b, a=255):
    x0, y0 = max(0, int(x0)), max(0, int(y0))
    x1, y1 = min(W, int(x1)), min(H, int(y1))
    for y in range(y0, y1):
        row = y * W * 4
        for x in range(x0, x1):
            i = row + x * 4
            buf[i] = r; buf[i+1] = g; buf[i+2] = b; buf[i+3] = a

# Rounded dark-navy macOS squircle background (#0f1726) with subtle border (#263a5c)
rad = 220
pad = 48
for y in range(pad, H - pad):
    row = y * W * 4
    for x in range(pad, W - pad):
        dx = max(pad + rad - x, 0, x - (W - pad - rad - 1))
        dy = max(pad + rad - y, 0, y - (H - pad - rad - 1))
        dist = (dx * dx + dy * dy) ** 0.5
        if dist <= rad:
            i = row + x * 4
            if dist >= rad - 10:
                buf[i], buf[i+1], buf[i+2], buf[i+3] = 44, 68, 108, 255
            else:
                # Subtle vertical gradient
                t = (y - pad) / (H - 2 * pad)
                buf[i]   = int(14 + 8 * (1 - t))
                buf[i+1] = int(20 + 12 * (1 - t))
                buf[i+2] = int(34 + 18 * (1 - t))
                buf[i+3] = 255

# 4 ascending 4-FSK bars matching rad950.org & desktop brand mark
bars = [
    (210, 560, 330, 800,  93, 167, 232),
    (365, 440, 485, 800, 126, 194, 248),
    (520, 320, 640, 800, 168, 216, 255),
    (675, 200, 795, 800, 255, 255, 255),
]
for x0, y0, x1, y1, r, g, b in bars:
    fill_rect(x0, y0, x1, y1, r, g, b, 255)

def png_chunk(tag, data):
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

raw = bytearray()
for y in range(H):
    raw.append(0)
    raw.extend(buf[y*W*4:(y+1)*W*4])

png = (
    b"\x89PNG\r\n\x1a\n"
    + png_chunk(b"IHDR", struct.pack(">IIBBBBB", W, H, 8, 6, 0, 0, 0))
    + png_chunk(b"IDAT", zlib.compress(bytes(raw), 9))
    + png_chunk(b"IEND", b"")
)
with open(sys.argv[1], "wb") as f:
    f.write(png)
PY

for sz in 16 32 128 256 512; do
  sips -z "$sz" "$sz" "$WORK/icon_1024.png" --out "$ICONSET/icon_${sz}x${sz}.png" >/dev/null
  dbl=$((sz * 2))
  sips -z "$dbl" "$dbl" "$WORK/icon_1024.png" --out "$ICONSET/icon_${sz}x${sz}@2x.png" >/dev/null
done
iconutil -c icns "$ICONSET" -o "$APP/Contents/Resources/AppIcon.icns"

cat > "$APP/Contents/Info.plist" << PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleName</key>
  <string>RAD-950</string>
  <key>CFBundleDisplayName</key>
  <string>RAD-950</string>
  <key>CFBundleIdentifier</key>
  <string>org.rad950.desktop</string>
  <key>CFBundleVersion</key>
  <string>${VERSION}</string>
  <key>CFBundleShortVersionString</key>
  <string>${VERSION}</string>
  <key>CFBundlePackageType</key>
  <string>APPL</string>
  <key>CFBundleExecutable</key>
  <string>rad950</string>
  <key>CFBundleIconFile</key>
  <string>AppIcon</string>
  <key>LSMinimumSystemVersion</key>
  <string>12.0</string>
  <key>NSHighResolutionCapable</key>
  <true/>
  <key>NSMicrophoneUsageDescription</key>
  <string>RAD-950 uses audio input to receive and decode weak-signal HF audio from your transceiver or USB sound card.</string>
  <key>NSLocalNetworkUsageDescription</key>
  <string>RAD-950 connects to local SDRs (TCI) and Hamlib CAT servers (rigctld) on your network.</string>
</dict>
</plist>
PLIST

# Sign bundle (Developer ID + Hardened Runtime when SIGN_IDENTITY is set; ad-hoc otherwise)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENTITLEMENTS="$SCRIPT_DIR/entitlements.plist"
if [[ -n "${SIGN_IDENTITY:-}" ]]; then
  codesign --force --deep --options runtime --timestamp --entitlements "$ENTITLEMENTS" --sign "$SIGN_IDENTITY" "$APP"
  codesign --verify --deep --strict --verbose=2 "$APP"
else
  codesign --force --deep --sign - "$APP"
fi

# Classic drag-to-Applications symlink
ln -s /Applications "$WORK/stage/Applications"

rm -f "$OUT_DMG"
hdiutil create -volname "RAD-950" -srcfolder "$WORK/stage" -ov -format UDZO "$OUT_DMG" >/dev/null
if [[ -n "${SIGN_IDENTITY:-}" ]]; then
  codesign --force --timestamp --sign "$SIGN_IDENTITY" "$OUT_DMG"
fi
ls -lh "$OUT_DMG"
