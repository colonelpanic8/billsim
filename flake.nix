{
  description = "billsim Rust development environment";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = {
    nixpkgs,
    flake-utils,
    fenix,
    ...
  }:
    flake-utils.lib.eachSystem [
      "x86_64-linux"
      "aarch64-linux"
      "aarch64-darwin"
    ] (
      system: let
        pkgs = nixpkgs.legacyPackages.${system};
        fenixPkgs = fenix.packages.${system};
        rustToolchain = fenixPkgs.combine [
          (fenixPkgs.stable.withComponents [
            "cargo"
            "clippy"
            "rust-src"
            "rustc"
            "rustfmt"
          ])
          fenixPkgs.targets.aarch64-linux-android.stable.rust-std
          fenixPkgs.targets.armv7-linux-androideabi.stable.rust-std
          fenixPkgs.targets.wasm32-unknown-unknown.stable.rust-std
          fenixPkgs.targets.x86_64-linux-android.stable.rust-std
        ];
        rustPlatform = pkgs.makeRustPlatform {
          cargo = rustToolchain;
          rustc = rustToolchain;
        };
        shellPackages = [
          rustToolchain
          pkgs.rust-analyzer
          pkgs.python311
          pkgs.uv
          pkgs.maturin
          pkgs.nodejs_24
          pkgs.wasm-pack
          pkgs.wasm-bindgen-cli
          pkgs.binaryen
          pkgs.cargo-ndk
          pkgs.just
          pkgs.pkg-config
          pkgs.git
        ];
      in {
        packages = {
          default = rustPlatform.buildRustPackage {
            pname = "billsim";
            version = "0.1.0";
            src = ./.;
            cargoLock.lockFile = ./Cargo.lock;
            cargoBuildFlags = ["--lib"];

            installPhase = ''
              runHook preInstall
              mkdir -p "$out/lib"
              find target -type f \
                \( -name libbillsim.a -o -name libbillsim.so -o -name libbillsim.dylib \) \
                -exec cp {} "$out/lib/" \;
              test -n "$(find "$out/lib" -type f -print -quit)"
              runHook postInstall
            '';
          };
          rust-toolchain = rustToolchain;
        };

        formatter = pkgs.alejandra;

        devShells.default = pkgs.mkShell {
          packages = shellPackages;

          env = {
            UV_PYTHON_DOWNLOADS = "never";
          };

          shellHook = ''
            unset PYTHONPATH
            export REPO_ROOT=$(git rev-parse --show-toplevel)
            export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath shellPackages}:${pkgs.stdenv.cc.cc.lib.outPath}/lib:''${LD_LIBRARY_PATH:-}"
          '';
        };
      }
    );
}
