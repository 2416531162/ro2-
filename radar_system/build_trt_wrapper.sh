#!/bin/bash
set -e
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

echo "Building libtrt_engine_wrapper.so for NVIDIA Jetson..."
g++ -O3 -shared -fPIC \
    -I/usr/local/cuda-11.4/include \
    -I/usr/include/aarch64-linux-gnu \
    trt_engine_wrapper.cpp \
    -L/usr/local/cuda-11.4/lib64 \
    -L/usr/lib/aarch64-linux-gnu \
    -lcudart -lnvinfer \
    -o libtrt_engine_wrapper.so

echo "Built successfully: $DIR/libtrt_engine_wrapper.so"
