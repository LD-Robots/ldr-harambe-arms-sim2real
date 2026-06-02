#!/usr/bin/env bash
#
# Fetch the prebuilt ONNX Runtime into third_party/onnxruntime.
# ONNX Runtime ships as a precompiled release tarball (not buildable as a
# git submodule in any practical sense), so it is downloaded on demand.
#
# Usage:
#   ./scripts/init_onnxruntime.sh
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

ONNXRUNTIME_VERSION="1.17.0"
ONNXRUNTIME_DIR="$WS_ROOT/third_party/onnxruntime"

if [[ -f "$ONNXRUNTIME_DIR/lib/libonnxruntime.so" ]]; then
  echo "ONNX Runtime already present at third_party/onnxruntime — skipping."
  exit 0
fi

TARBALL="onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz"
URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/${TARBALL}"

mkdir -p "$WS_ROOT/third_party"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "Downloading ONNX Runtime ${ONNXRUNTIME_VERSION}..."
echo "  $URL"
curl -fL "$URL" -o "$TMP/$TARBALL"

tar xzf "$TMP/$TARBALL" -C "$TMP"
rm -rf "$ONNXRUNTIME_DIR"
mv "$TMP/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}" "$ONNXRUNTIME_DIR"

echo "ONNX Runtime ${ONNXRUNTIME_VERSION} extracted to third_party/onnxruntime"
