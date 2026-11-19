#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
g++ -O2 -std=c++17 qpmap_encoder.cpp -o qpmap_encoder \
  $(pkg-config --cflags --libs libavcodec libavutil)
echo "built: $(pwd)/qpmap_encoder"
