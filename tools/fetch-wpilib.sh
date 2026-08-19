#!/usr/bin/env bash
# Fetches the WPILib 2027 Java artifacts we compile the WPILib layer against.
#
# These are not on frcmaven's generic release/development paths — those carry only
# edu.wpi.first.* up to 2026. The 2027 org.wpilib.* coordinates live in year-scoped
# repositories, which is what wpilibRepositories.use2027Repos() resolves to internally.
set -euo pipefail

VERSION="${WPILIB_VERSION:-2027.0.0-alpha-6}"
MAVEN="https://frcmaven.wpi.edu/artifactory/wpilib-mvn-release-2027-local/org/wpilib"
DEST="${1:-.wpilib}"

ARTIFACTS="
wpilibj/wpilibj-java
hal/hal-java
wpiutil/wpiutil-java
wpimath/wpimath-java
wpiunits/wpiunits-java
datalog/datalog-java
epilogue/epilogue-runtime-java
ntcore/ntcore-java
"

mkdir -p "$DEST"
for a in $ARTIFACTS; do
  n="$(basename "$a")"
  [ -s "$DEST/$n.jar" ] && continue
  echo "  fetching $n $VERSION"
  curl -fsSL -o "$DEST/$n.jar" "$MAVEN/$a/$VERSION/$n-$VERSION.jar"
done
echo "==> WPILib $VERSION in $DEST ($(find "$DEST" -name '*.jar' | wc -l) jars)"
