#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
git submodule sync --recursive
git submodule update --init --remote --merge --recursive
