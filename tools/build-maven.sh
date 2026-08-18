#!/usr/bin/env bash
# Builds the complete static Maven repository for SapphifyLib.
#
# A Maven repository is nothing but files in directories: a jar, a sources jar, a javadoc jar,
# a pom, maven-metadata.xml, and a checksum set beside each. No artifact server is involved and
# no Gradle is required — javac and jar from any JDK 17+ are enough. AndyMark serve their FRC
# vendordep repository the same way, straight off static hosting.
#
# check.py from wpilibsuite/vendor-json-repo downloads and opens all three jars, so all three
# must exist. Missing the sources or javadoc jar fails validation.
#
#   ./tools/build-maven.sh [output-dir]      default: build/maven
set -euo pipefail

VERSION="2027.0.0-alpha-1"
GROUP_PATH="com/sapphify/frc"
ARTIFACT="SapphifyLib-java"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/build/maven}"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

command -v javac >/dev/null || { echo "javac not found. Install a JDK 17+ and put it on PATH." >&2; exit 1; }

# The self-check is a verification tool, not shipped API.
mapfile -t SRC < <(find "$ROOT/src/main/java" -name '*.java' ! -name 'SapphifyLibSelfCheck.java')

echo "==> compiling ${#SRC[@]} sources"
javac -d "$WORK/classes" "${SRC[@]}"

echo "==> generating javadoc"
javadoc -quiet -Xdoclint:none -d "$WORK/javadoc" "${SRC[@]}" 2>/dev/null

DEST="$OUT/$GROUP_PATH/$ARTIFACT/$VERSION"
rm -rf "$OUT"; mkdir -p "$DEST"

echo "==> packaging"
( cd "$WORK/classes" && jar --create --file "$DEST/$ARTIFACT-$VERSION.jar" . )
( cd "$ROOT/src/main/java" && jar --create --file "$DEST/$ARTIFACT-$VERSION-sources.jar" \
    $(find . -name '*.java' ! -name 'SapphifyLibSelfCheck.java') )
( cd "$WORK/javadoc" && jar --create --file "$DEST/$ARTIFACT-$VERSION-javadoc.jar" . )

cat > "$DEST/$ARTIFACT-$VERSION.pom" <<POM
<?xml version="1.0" encoding="UTF-8"?>
<project xmlns="http://maven.apache.org/POM/4.0.0"
         xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
         xsi:schemaLocation="http://maven.apache.org/POM/4.0.0 http://maven.apache.org/xsd/maven-4.0.0.xsd">
  <modelVersion>4.0.0</modelVersion>
  <groupId>com.sapphify.frc</groupId>
  <artifactId>$ARTIFACT</artifactId>
  <version>$VERSION</version>
  <packaging>jar</packaging>
  <name>SapphifyLib</name>
  <description>WPILib vendor library for SAPPHIFY FRC CAN devices. ROTEM is the first.</description>
  <url>https://rotem.sapphify.com</url>
  <licenses>
    <license><name>MIT License</name><url>https://opensource.org/licenses/MIT</url><distribution>repo</distribution></license>
  </licenses>
  <organization><name>Sapphify LLC</name><url>https://sapphify.com</url></organization>
  <scm><url>https://github.com/SapphifyRobotics/sapphify-lib</url></scm>
</project>
POM

cat > "$OUT/$GROUP_PATH/$ARTIFACT/maven-metadata.xml" <<META
<?xml version="1.0" encoding="UTF-8"?>
<metadata>
  <groupId>com.sapphify.frc</groupId>
  <artifactId>$ARTIFACT</artifactId>
  <versioning>
    <latest>$VERSION</latest>
    <release>$VERSION</release>
    <versions><version>$VERSION</version></versions>
  </versioning>
</metadata>
META

echo "==> checksums"
( cd "$OUT" && find . -type f ! -name '*.md5' ! -name '*.sha*' | while read -r f; do
    md5sum    "$f" | cut -d' ' -f1 > "$f.md5"
    sha1sum   "$f" | cut -d' ' -f1 > "$f.sha1"
    sha256sum "$f" | cut -d' ' -f1 > "$f.sha256"
    sha512sum "$f" | cut -d' ' -f1 > "$f.sha512"
  done )

echo "==> done: $OUT ($(find "$OUT" -type f | wc -l) files)"
echo
echo "Validate without publishing anything:"
echo "  curl -O https://raw.githubusercontent.com/wpilibsuite/vendor-json-repo/main/check.py"
echo "  mkdir -p 2027_alpha5 && cp vendordep/SapphifyLib-*.json 2027_alpha5/"
echo "  python3 check.py -v --local-maven $OUT 2027_alpha5/SapphifyLib-$VERSION.json"
