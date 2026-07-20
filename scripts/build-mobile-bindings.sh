#!/usr/bin/env bash
# Build the UniFFI mobile artifacts:
#   bindings/kotlin/uniffi/billsim/billsim.kt   Kotlin bindings
#   bindings/swift/{billsim.swift,billsimFFI.*} Swift bindings
#   bindings/jniLibs/<abi>/libbillsim.so        Android native libraries
#
# Requirements: rustup with the Android targets, cargo-ndk, and
# ANDROID_NDK_HOME pointing at an NDK. iOS static libraries must be built
# on macOS (aarch64-apple-ios / aarch64-apple-ios-sim targets +
# `xcodebuild -create-xcframework`); the Swift bindings generated here are
# platform-independent and can be committed from any host.
set -euo pipefail
cd "$(dirname "$0")/.."

ANDROID_TARGETS=(arm64-v8a armeabi-v7a x86_64)

echo "== host cdylib (uniffi scaffolding) + bindgen =="
cargo build --release --features uniffi-bindings
cargo build --release --features bindgen-cli --bin billsim-bindgen

echo "== kotlin + swift bindings =="
./target/release/billsim-bindgen generate \
  --library target/release/libbillsim.so --language kotlin --out-dir bindings/kotlin
./target/release/billsim-bindgen generate \
  --library target/release/libbillsim.so --language swift --out-dir bindings/swift

echo "== android native libraries =="
: "${ANDROID_NDK_HOME:?set ANDROID_NDK_HOME to an installed NDK}"
target_flags=()
for abi in "${ANDROID_TARGETS[@]}"; do
  target_flags+=(-t "$abi")
done
cargo ndk "${target_flags[@]}" -o bindings/jniLibs \
  build --release --features uniffi-bindings

echo "done: bindings/{kotlin,swift,jniLibs}"
