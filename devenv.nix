{ pkgs, lib, config, inputs, ... }:

{
  packages = [
  ];

  languages.rust = {
    enable = true;
    channel = "stable";
    targets = [ "thumbv7em-none-eabihf" ];
    components = [ "rust-src" ];
  };
}
