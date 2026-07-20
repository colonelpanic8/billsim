# Mobile bindings

The mobile boundary intentionally consists of two functions:

- `simulateJson(request)`
- `computePotAimJson(request)`

They use the same serde contract as the Rust and Python APIs. Keeping one wire
contract avoids maintaining separate Swift, Kotlin, Python, and TypeScript
representations of the physics model while it evolves.

## Generate Swift and Kotlin sources

Build the host library and run the bundled UniFFI generator:

```sh
cargo build --release --features uniffi-bindings
cargo run --features bindgen-cli --bin billsim-bindgen -- \
  generate target/release/libbillsim.so \
  --language swift --out-dir target/generated/swift
cargo run --features bindgen-cli --bin billsim-bindgen -- \
  generate target/release/libbillsim.so \
  --language kotlin --out-dir target/generated/kotlin
```

The generated Swift and Kotlin bindings are build artifacts and are not
committed.

## iOS library

Install the Apple Rust targets required by the application and build a static
library for each desired architecture. Typical targets are
`aarch64-apple-ios` and `aarch64-apple-ios-sim`; Intel simulator support can
also use `x86_64-apple-ios`. Combine the libraries and generated Swift header
into an XCFramework in the application build pipeline.

## Android library

Build `cdylib` artifacts for the Android ABIs the application ships, normally
`aarch64-linux-android`, `armv7-linux-androideabi`, `x86_64-linux-android`, and
optionally `i686-linux-android`. Package each `libbillsim.so` under its matching
`jniLibs/<abi>` directory alongside the generated Kotlin source.

## React Native / Nitro seam

Railbird can keep its JavaScript-facing request and response objects. Its
Swift and Kotlin Nitro implementations only need to serialize the request,
call the generated `simulateJson` or `computePotAimJson` function off the UI
thread, and parse the response. The Rust library contains no renderer,
platform runtime, global mutable simulation state, or mobile-specific physics
fork.
