{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, fenix, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
      in
      {
        devShells.default = pkgs.mkShell {
          name = "Lucy Embedded Firmware";
          packages = [
            # Rust itself is NOT provided here — see rust-toolchain.toml in each
            # firmware crate; rustup will auto-select the right toolchain per
            # directory (nightly for AVR/build-std, stable for RP2040).
            pkgs.rustup

            # rustToolchain
            pkgs.udev
            pkgs.systemd
            pkgs.flip-link
            pkgs.probe-rs-tools
            pkgs.cargo-generate

            # AVR / Arduino tooling
            pkgs.avrdude
            pkgs.pkgsCross.avr.buildPackages.gcc   # avr-gcc, for linking against avr-libc
            pkgs.ravedude

            # shared / testing
            pkgs.pkg-config
            pkgs.simavr
            pkgs.elf2uf2-rs
            pkgs.picotool

            # task runner
            pkgs.just

          ];
          nativeBuildInputs = [
            pkgs.pkg-config
          ];
          buildInputs = [
            pkgs.udev
          ];
          shellHook = ''
            export LD_LIBRARY_PATH="${pkgs.udev}/lib:$LD_LIBRARY_PATH"
            echo -e ""
            echo -e "🛡️  \033[1;36mLucy Embedded Firmware\033[1;0m"
            echo -e "----------------------------"
            echo -e "Rust toolchain: managed by rustup (per-crate rust-toolchain.toml)"
            echo -e "Run 'cargo build' inside firmwares/<target>/ to auto-select the right toolchain."
            echo -e "----------------------------"
          '';
        };
      }
    );
}
