{
  description = "FEH Proteus";

  inputs.nixpkgs.url = "github:nixos/nixpkgs";

  outputs = {
    self,
    nixpkgs,
  }: let
    pkgs = import nixpkgs {system = "x86_64-linux";};
  in {
    packages.x86_64-linux = rec {
      s19 = pkgs.stdenv.mkDerivation {
        name = "s19";
        src = ./.;
        buildInputs = with pkgs; [
          gcc-arm-embedded
          glibc_multi
          doxygen
        ];
        installPhase = ''
          mkdir -p $out
          cp *.s19 $out
        '';
      };
      default = s19;
    };
  };
}
