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
        targets = [
          "thumbv6m-none-eabi"
        ];
        rustToolchain = fenix.packages.${system}.combine ([
          fenix.packages.${system}.stable.cargo
          fenix.packages.${system}.stable.rustc
          fenix.packages.${system}.stable.clippy
          fenix.packages.${system}.stable.rustfmt
          fenix.packages.${system}.stable.rust-src
        ] ++ (map (t: fenix.packages.${system}.targets.${t}.stable.rust-std) targets));
      in
      {
        devShells.default = pkgs.mkShell {
          name = "Lucy Embedded Firmware";
          packages = [
            rustToolchain
            pkgs.udev
            pkgs.systemd
            pkgs.flip-link
            pkgs.probe-rs-tools
            pkgs.elf2uf2-rs
            pkgs.picotool
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
            echo -e "cargo: $(cargo --version  | cut -d ' ' -f2)"
            echo -e "rustc: $(rustc --version | cut -d ' ' -f2)"
            echo -e "----------------------------"
          '';
        };
      }
    );
}
