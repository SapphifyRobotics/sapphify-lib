#!/usr/bin/env bash
# Builds the offline installation zip.
#
# Why this exists: a vendordep installed in "online" mode caches its artifacts locally and WPILib
# clears that cache after about 30 days. A team that installed in October and arrives at a
# competition in March with no internet has a robot that will not build. Every established vendor
# ships an offline zip for exactly this reason, and every SystemcoreTesting vendor page documents
# one.
#
# The zip extracts into the root of the user's wpilib year directory, so the paths inside it are
# relative to that root:
#
#   Linux and macOS   ~/wpilib/2027_alpha5
#   Windows           C:\Users\Public\wpilib\2027_alpha5
#
#   ./tools/build-offline-zip.sh [output-dir]     default: build
set -euo pipefail

VERSION="2027.0.0-alpha-1"
WPILIB_YEAR="2027_alpha5"
NAME="SapphifyLib"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/build}"
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

echo "==> building maven tree"
"$ROOT/tools/build-maven.sh" "$STAGE/maven" >/dev/null

# WPILib looks for offline artifacts under <year>/maven, and for the vendordep JSON under
# <year>/vendordeps, so the archive mirrors that layout exactly.
mkdir -p "$STAGE/pkg/maven" "$STAGE/pkg/vendordeps"
cp -r "$STAGE/maven/." "$STAGE/pkg/maven/"
cp "$ROOT/vendordep/$NAME-$VERSION.json" "$STAGE/pkg/vendordeps/$NAME.json"

cat > "$STAGE/pkg/vendordeps/README.txt" <<TXT
$NAME $VERSION — offline installation

Extract the contents of this archive into the root of your WPILib $WPILIB_YEAR directory:

  Linux and macOS   ~/wpilib/$WPILIB_YEAR
  Windows           C:\\Users\\Public\\wpilib\\$WPILIB_YEAR

Then, in your robot project, add vendordeps/$NAME.json — or copy it there directly.

Installing this way needs no internet connection and does not expire. A vendordep installed in
online mode clears its cache after roughly 30 days and will fail to build without a network.

Documentation: https://frc.sapphify.com
TXT

mkdir -p "$OUT"
ZIP="$OUT/$NAME-offline-$VERSION.zip"
rm -f "$ZIP"
( cd "$STAGE/pkg" && zip -qr "$ZIP" . )

echo "==> done: $ZIP ($(du -h "$ZIP" | cut -f1), $(unzip -l "$ZIP" | tail -1 | awk '{print $2}') files)"
